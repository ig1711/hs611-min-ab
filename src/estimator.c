/*
 * estimator.c — sub-pixel position estimators. See estimator.h.
 */

#include "estimator.h"

/* Per-channel calibration table for the vendor estimator (1/256-channel units);
 * uniform pitch until replaced with measured values. 44 entries covers both
 * axes plus the +/-1 lookups. */
#define VENDOR_CAL_N      44U

/* Vendor amplitude-band nonlinearity corrections (0x2bc..0x898 subtract,
 * 0x898..0x9f6 add). Coefficients are zero until calibrated. */
#define VENDOR_BAND1_LO   700U
#define VENDOR_BAND1_HI   0x898U
#define VENDOR_BAND2_LO   0x898U
#define VENDOR_BAND2_HI   0x9F6U

/* Log-Gaussian defaults fitted from the first stepped calibration capture. */
#define LOG_BASE_DEF      690
#define LOG_X_A1_DEF      200   /* 0.781 */
#define LOG_X_A3_DEF      224   /* 0.875 */
#define LOG_Y_A1_DEF      221   /* 0.862 */
#define LOG_Y_A3_DEF      141   /* 0.551 */

static uint32_t g_vendor_cal[VENDOR_CAL_N];
static uint8_t  g_estimator = ESTIMATOR_NCENTROID;
static uint16_t g_ncent_base;
static uint16_t g_lg_base = LOG_BASE_DEF;
static int16_t  g_lg_a1[2] = { LOG_X_A1_DEF, LOG_Y_A1_DEF };
static int16_t  g_lg_a3[2] = { LOG_X_A3_DEF, LOG_Y_A3_DEF };

/* ---- shared helpers ----------------------------------------------------- */
/* Half-maximum midpoint between the interpolated shoulder crossings: the
 * continuous fallback for a saturated flat top or a non-concave triple. */
static int32_t halfmax_position(const uint16_t *w, uint8_t m, uint8_t base,
                                uint8_t li, uint16_t vmax)
{
    uint16_t bmin = w[0];
    uint16_t level;
    uint8_t lk, rk;
    int32_t left_fp, right_fp;

    for (uint8_t i = 1U; i < m; i++) {
        if (w[i] < bmin) { bmin = w[i]; }
    }
    if (vmax <= bmin) {
        return (int32_t)(base + li + 1U) << 8;
    }
    level = (uint16_t)(((uint32_t)bmin + (uint32_t)vmax) / 2U);

    lk = li;
    while ((lk > 0U) && (w[lk - 1U] >= level)) { lk--; }
    if (lk == 0U) {
        left_fp = 0;
    } else {
        int32_t a = (int32_t)w[lk - 1U];
        int32_t b = (int32_t)w[lk];
        int32_t num = (int32_t)bmin + (int32_t)vmax - 2 * a;
        int32_t den = 2 * (b - a);
        left_fp = ((int32_t)(lk - 1U) << 8) + ((num << 8) / den);
    }

    rk = li;
    while (((uint8_t)(rk + 1U) < m) && (w[rk + 1U] >= level)) { rk++; }
    if ((uint8_t)(rk + 1U) >= m) {
        right_fp = (int32_t)(m - 1U) << 8;
    } else {
        int32_t a = (int32_t)w[rk];
        int32_t b = (int32_t)w[rk + 1U];
        int32_t num = 2 * a - ((int32_t)bmin + (int32_t)vmax);
        int32_t den = 2 * (a - b);
        right_fp = ((int32_t)rk << 8) + ((num << 8) / den);
    }

    return ((int32_t)(base + 1U) << 8) + ((left_fp + right_fp) / 2);
}

/* 3-point normalized-difference centroid; half-max fallback for a flat top. */
static int32_t position_parabola3(const uint16_t *w, uint8_t m, uint8_t base)
{
    uint8_t li = 0U;
    uint16_t vmax;
    uint8_t lo, hi;

    if (m == 0U) { return (int32_t)base << 8; }

    vmax = w[0];
    for (uint8_t i = 1U; i < m; i++) {
        if (w[i] > vmax) { vmax = w[i]; li = i; }
    }

    lo = li; hi = li;
    while ((lo > 0U) && (w[lo - 1U] == vmax)) { lo--; }
    while (((uint8_t)(hi + 1U) < m) && (w[hi + 1U] == vmax)) { hi++; }
    if (hi > lo) {
        return halfmax_position(w, m, base, li, vmax);
    }

    if ((li > 0U) && ((uint8_t)(li + 1U) < m)) {
        int32_t a = (int32_t)w[li - 1U];
        int32_t b = (int32_t)w[li];
        int32_t c = (int32_t)w[li + 1U];
        int32_t bmin = (a < b) ? a : b;
        int32_t wm, wz, wp, den, d;

        if (c < bmin) { bmin = c; }
        wm = a - bmin;
        wz = b - bmin;
        wp = c - bmin;
        den = wm + wz + wp;
        if (den != 0) {
            d = ((wp - wm) << 8) / den;
            if (d > 128) { d = 128; }
            else if (d < -128) { d = -128; }
            return ((int32_t)(base + li + 1U) << 8) + d;
        }
        return halfmax_position(w, m, base, li, vmax);
    }

    return (int32_t)(base + li + 1U) << 8;
}

/* ---- N-point centroid --------------------------------------------------- */
static int32_t position_ncentroid(const uint16_t *w, uint8_t m, uint8_t base)
{
    int64_t num = 0;
    int64_t den = 0;

    if (m == 0U) { return (int32_t)base << 8; }
    for (uint8_t i = 0U; i < m; i++) {
        int32_t v = (int32_t)w[i] - (int32_t)g_ncent_base;
        if (v > 0) {
            num += (int64_t)((int32_t)base + (int32_t)i + 1) * (int64_t)v;
            den += v;
        }
    }
    if (den == 0) {
        return position_parabola3(w, m, base);
    }
    return (int32_t)((num * 256 + (den / 2)) / den);
}

/* ---- vendor rational interpolator --------------------------------------- */
static int32_t vendor_band_corr(uint32_t amp)
{
    /* Band slopes/offsets are zero until measured; the branches are kept so the
     * correction structure matches the vendor and can be calibrated. */
    if ((amp > VENDOR_BAND1_LO) && (amp < VENDOR_BAND1_HI)) {
        return 0;
    } else if ((amp > VENDOR_BAND2_LO) && (amp < VENDOR_BAND2_HI)) {
        return 0;
    }
    return 0;
}

static int32_t position_vendor(const uint16_t *w, uint8_t m, uint8_t base)
{
    uint8_t li = 0U, lo, hi;
    uint16_t vmax;
    uint32_t c;
    int32_t wL, wR;
    int64_t dTL, dTR, num, den, delta, pos, lo_c, hi_c;

    if (m == 0U) {
        return (int32_t)base << 8;
    }

    vmax = w[0];
    for (uint8_t i = 1U; i < m; i++) {
        if (w[i] > vmax) { vmax = w[i]; li = i; }
    }

    /* exact plateau or saturated top: shoulder midpoint */
    lo = li; hi = li;
    while ((lo > 0U) && (w[lo - 1U] == vmax)) { lo--; }
    while (((uint8_t)(hi + 1U) < m) && (w[hi + 1U] == vmax)) { hi++; }
    if ((hi > lo) || (vmax >= ESTIMATOR_VENDOR_SAT)) {
        return halfmax_position(w, m, base, li, vmax);
    }

    if ((li == 0U) || ((uint8_t)(li + 1U) >= m)) {
        return halfmax_position(w, m, base, li, vmax);
    }

    c = (uint32_t)base + (uint32_t)li + 1U;             /* 1-based peak channel */
    if ((c + 1U) >= VENDOR_CAL_N) {
        return (int32_t)(base + li + 1U) << 8;
    }

    wL = (int32_t)w[li - 1U] - (int32_t)w[li];
    wR = (int32_t)w[li + 1U] - (int32_t)w[li];
    dTL = (int64_t)g_vendor_cal[c - 1U] - (int64_t)g_vendor_cal[c];
    dTR = (int64_t)g_vendor_cal[c + 1U] - (int64_t)g_vendor_cal[c];

    num = (int64_t)wR * dTL * dTL - (int64_t)wL * dTR * dTR;
    den = (int64_t)wR * dTL - (int64_t)wL * dTR;
    if (den == 0) {
        return halfmax_position(w, m, base, li, vmax);
    }
    delta = num / (2 * den);
    pos = (int64_t)g_vendor_cal[c] + delta;

    lo_c = (int64_t)g_vendor_cal[c - 1U];
    hi_c = (int64_t)g_vendor_cal[c + 1U];
    if (lo_c > hi_c) { int64_t t = lo_c; lo_c = hi_c; hi_c = t; }
    if (pos < lo_c) { pos = lo_c; } else if (pos > hi_c) { pos = hi_c; }

    pos += vendor_band_corr((uint32_t)vmax);
    if (pos < 0) { pos = 0; }
    if (pos > 0x7FFFFFFF) { pos = 0x7FFFFFFF; }
    return (int32_t)pos;
}

/* ---- Gaomon 5-point ----------------------------------------------------- */
static int32_t position_gaomon(const uint16_t *w, uint8_t m, uint8_t base)
{
    uint8_t li = 0U;
    uint16_t vmax;
    int32_t floor_lvl = (int32_t)g_ncent_base;
    int32_t s0, s1, s2, s3, s4, d, denom, off_p, off_l, off;

    if (m < 5U) {
        return position_vendor(w, m, base);
    }

    vmax = w[0];
    for (uint8_t i = 1U; i < m; i++) {
        if (w[i] > vmax) { vmax = w[i]; li = i; }
    }
    if ((li < 2U) || ((uint8_t)(li + 2U) >= m)) {
        return position_vendor(w, m, base);
    }

    s0 = (int32_t)w[li - 2U] - floor_lvl; if (s0 < 0) { s0 = 0; }
    s1 = (int32_t)w[li - 1U] - floor_lvl; if (s1 < 0) { s1 = 0; }
    s2 = (int32_t)w[li]      - floor_lvl; if (s2 < 0) { s2 = 0; }
    s3 = (int32_t)w[li + 1U] - floor_lvl; if (s3 < 0) { s3 = 0; }
    s4 = (int32_t)w[li + 2U] - floor_lvl; if (s4 < 0) { s4 = 0; }
    if ((s2 == 0) || ((floor_lvl == 0) && ((s0 == 0) || (s4 == 0)))) {
        return position_vendor(w, m, base);
    }

    d = s3 - s1;
    denom = (d < 0) ? ((s2 - s0) - d) : ((s2 - s4) + d);
    if (denom == 0) {
        return position_vendor(w, m, base);
    }
    off_p = (256 * d) / denom;

    denom = 2 * s2 - s1 - s3;
    off_l = (denom != 0) ? ((128 * d) / denom) : off_p;

    off = (off_p + off_l) / 2;
    if (off > 128) { off = 128; } else if (off < -128) { off = -128; }

    return ((int32_t)(base + li + 1U) << 8) + off;
}

/* ---- log-Gaussian 3-point ----------------------------------------------- */
/* Natural log of v >= 1 in Q16: ln(v) = k*ln2 + 2*atanh(z), z = (f-1)/(f+1).
 * The series to z^7/7 is accurate to well under 1 Q16 LSB over z <= 1/3. */
static int32_t ln_q16(uint32_t v)
{
    const int64_t ln2_q16 = 45426;
    int64_t f, z, z2, z3, z5, z7, s;
    int k = 0;

    if (v == 0U) {
        return -0x7FFFFFFF;
    }
    while ((v >> (k + 1)) != 0U) {
        k++;
    }
    f = ((int64_t)v << 16) >> k;                      /* Q16, [65536, 131072) */
    z = ((f - 65536) * 65536) / (f + 65536);          /* Q16, [0, 21845) */
    z2 = (z * z) >> 16;
    z3 = (z2 * z) >> 16;
    z5 = (z3 * z2) >> 16;
    z7 = (z5 * z2) >> 16;
    s = z + (z3 / 3) + (z5 / 5) + (z7 / 7);
    return (int32_t)((int64_t)k * ln2_q16 + 2 * s);
}

static int32_t position_loggauss(const uint16_t *w, uint8_t m, uint8_t base, uint8_t axis)
{
    uint8_t li = 0U;
    uint16_t vmax;
    int32_t L, C, R, lnL, lnC, lnR, y, z, g, u;
    int64_t a1, a3;

    if (m < 3U) {
        return position_parabola3(w, m, base);
    }
    vmax = w[0];
    for (uint8_t i = 1U; i < m; i++) {
        if (w[i] > vmax) { vmax = w[i]; li = i; }
    }
    if ((li == 0U) || ((uint8_t)(li + 1U) >= m)) {
        return position_parabola3(w, m, base);
    }

    L = (int32_t)w[li - 1U] - (int32_t)g_lg_base;
    C = (int32_t)w[li]      - (int32_t)g_lg_base;
    R = (int32_t)w[li + 1U] - (int32_t)g_lg_base;
    if ((L <= 0) || (C <= 0) || (R <= 0)) {
        return position_parabola3(w, m, base);
    }

    lnL = ln_q16((uint32_t)L);
    lnC = ln_q16((uint32_t)C);
    lnR = ln_q16((uint32_t)R);
    y = lnR - lnL;                                    /* Q16 */
    z = 2 * lnC - lnL - lnR;                          /* Q16, = 1/sigma^2 > 0 */
    if (z == 0) {
        return position_parabola3(w, m, base);
    }

    g = (int32_t)(((int64_t)y * 256) / ((int64_t)2 * z));   /* Q8, -0.5..0.5 coil */
    if (g > 128) { g = 128; } else if (g < -128) { g = -128; }

    /* per-axis anchor-preserving cubic: u = a1*g + a3*g^3 (all Q8/Q16) */
    a1 = (int64_t)g_lg_a1[(axis != 0U) ? 1U : 0U];
    a3 = (int64_t)g_lg_a3[(axis != 0U) ? 1U : 0U];
    u = (int32_t)(((a1 * (int64_t)g) + (((a3 * (int64_t)g * g * g) >> 16))) >> 8);
    if (u > 128) { u = 128; } else if (u < -128) { u = -128; }

    return ((int32_t)(base + li + 1U) << 8) + u;
}

/* ---- public API --------------------------------------------------------- */
void estimator_init(void)
{
    for (uint8_t ch = 0U; ch < VENDOR_CAL_N; ch++) {
        g_vendor_cal[ch] = (uint32_t)ch << 8;          /* uniform pitch */
    }
}

void estimator_set(uint8_t mode)
{
    g_estimator = (mode <= ESTIMATOR_LOGAUSS) ? mode : ESTIMATOR_NCENTROID;
}

uint8_t estimator_get(void)
{
    return g_estimator;
}

int32_t estimator_position(const uint16_t *w, uint8_t m, uint8_t base, uint8_t axis)
{
    switch (g_estimator) {
    case ESTIMATOR_VENDOR:
        return position_vendor(w, m, base);
    case ESTIMATOR_GAOMON:
        return position_gaomon(w, m, base);
    case ESTIMATOR_LOGAUSS:
        return position_loggauss(w, m, base, axis);
    case ESTIMATOR_NCENTROID:
    default:
        return position_ncentroid(w, m, base);
    }
}

void estimator_set_ncentroid(uint16_t noise) { g_ncent_base = noise; }
uint16_t estimator_get_ncentroid(void) { return g_ncent_base; }

void estimator_set_loggauss(uint8_t axis, uint16_t baseline, int16_t a1_q8, int16_t a3_q8)
{
    g_lg_base = baseline;
    uint8_t a = (axis != 0U) ? 1U : 0U;
    g_lg_a1[a] = a1_q8;
    g_lg_a3[a] = a3_q8;
}

uint16_t estimator_get_loggauss_base(void) { return g_lg_base; }
int16_t estimator_get_loggauss_a1(uint8_t axis) { return g_lg_a1[(axis != 0U) ? 1U : 0U]; }
int16_t estimator_get_loggauss_a3(uint8_t axis) { return g_lg_a3[(axis != 0U) ? 1U : 0U]; }
