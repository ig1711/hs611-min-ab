/*
 * scan.c — acquisition scan engine and runtime A/B settings. See scan.h.
 */

#include "scan.h"
#include "protocol.h"
#include "board.h"
#include "dwt.h"
#include "acq.h"
#include "estimator.h"
#include "coil_tables.h"

/* Keep-alive carrier periods across the frame build in the carry-over mode so
 * the drive never idles between scans. 0 disables. */
#ifndef SCAN_KEEPALIVE
#define SCAN_KEEPALIVE      128U
#endif

/* Window/tracking state. */
static uint8_t  g_win_x;                          /* axis-B window coils (0 = full) */
static uint8_t  g_win_y;                          /* axis-A window coils */
static uint8_t  g_recenter = PROTO_RECENTER_FOLLOW;
static uint8_t  g_rc_hyst = 60U;
static uint8_t  g_rc_deadband = 1U;
static uint8_t  g_rc_persist = 1U;
static uint8_t  g_rc_step = 1U;
static uint8_t  g_rc_pend_x;
static uint8_t  g_rc_pend_y;
static uint8_t  g_scan_order = PROTO_SCAN_ASC;
static uint8_t  g_warmup;
static uint16_t g_flat_tol;
static uint8_t  g_ramp_mode = PROTO_RAMP_NONE;
static int16_t  g_ramp_slope;
static uint8_t  g_rev_radius = 1U;
static uint16_t g_reacq_thr = 60U;
static uint8_t  g_coarse_stride = 3U;
static uint8_t  g_coarse_phase;
static uint16_t g_reacq_period_ms;
static uint32_t g_next_reacq_cyc;
static uint8_t  g_reacq_ts_valid;
static uint8_t  g_pacing = PROTO_PACING_AUTO;
static uint8_t  g_free_run;
static uint8_t  g_repeat_coil;                    /* 0 = off, 1..41 */

/* Tracking state updated from each scan. */
static uint8_t  g_last_bx;                        /* axis-B peak (1..41) */
static uint8_t  g_last_by;                        /* axis-A peak (1..27) */
static uint8_t  g_have_center;
static uint16_t g_prev_amp;
static int32_t  g_xfp;                            /* last X sub-pixel pos (1/256 ch) */
static int32_t  g_yfp;
static uint8_t  g_frame_toggle;                   /* mode 7 per-frame scan order */
static int32_t  g_prev_xfp;
static int32_t  g_prev_yfp;
static uint8_t  g_prev_pos_valid;
static uint8_t  g_reacquired;
static uint8_t  g_sat_retry;
static uint32_t g_scan_us;

/* Scan buffers. [0] is unused; [1..n] are 1-based coils. */
static uint16_t g_prof_b[AXIS_B_N + 1U];
static uint16_t g_prof_a[AXIS_A_N + 1U];
static uint16_t g_tmp_b[AXIS_B_N + 1U];
static uint16_t g_tmp_a[AXIS_A_N + 1U];
static uint16_t g_repeat_hist[PROTO_NX];

/* ---- small helpers ------------------------------------------------------ */
static void peak_find(const uint16_t *out, uint8_t n,
                      uint8_t *best, uint8_t *second, uint8_t *third)
{
    uint16_t v1 = 0U, v2 = 0U, v3 = 0U;

    *best = 0U; *second = 0U; *third = 0U;
    for (uint8_t i = 1U; i <= n; i++) {
        uint16_t v = out[i];
        if (v > v1) {
            v3 = v2; *third  = *second;
            v2 = v1; *second = *best;
            v1 = v;  *best   = i;
        } else if (v > v2) {
            v3 = v2; *third  = *second;
            v2 = v;  *second = i;
        } else if (v > v3) {
            v3 = v;  *third = i;
        }
    }
}

static uint16_t peak_max(const uint16_t *out, uint8_t n, uint8_t *peak)
{
    uint8_t s, t;
    peak_find(out, n, peak, &s, &t);
    return out[*peak];
}

/* Measure one 1-based coil of `table` into out[idx]. */
static void measure_idx(const uint8_t *table, uint16_t *out, uint8_t idx)
{
    const uint8_t *e = coil_entry(table, idx);
    out[idx] = acq_measure(coil_mask(e), coil_value(e));
}

/* Window bounds for `w` coils centred on `center`: [*lo, *hi] inclusive,
 * clamped to [1, n]. For even `w` the extra coil goes to the right. */
static void win_bounds(uint8_t n, uint8_t center, uint8_t w, uint8_t *lo, uint8_t *hi)
{
    uint8_t left = (uint8_t)((w - 1U) / 2U);
    uint8_t l = (center > left) ? (uint8_t)(center - left) : 1U;
    uint8_t h = (uint8_t)(l + w - 1U);
    if (h > n) {
        h = n;
        l = ((uint16_t)h + 1U >= (uint16_t)w) ? (uint8_t)(h - w + 1U) : 1U;
    }
    *lo = l;
    *hi = h;
}

/* ---- ramp mitigation ---------------------------------------------------- */
/* Mode 5: linear-in-scan-order gain. `k` is the 0-based order within a scan of
 * `w` coils; the correction is centred so the mean gain is 1 and a positive
 * slope boosts the earlier (ascending) measurements. */
static uint16_t ramp_slope_apply(uint16_t v, uint8_t k, uint8_t w)
{
    int32_t off2, num;

    if ((g_ramp_mode != PROTO_RAMP_SLOPE) || (g_ramp_slope == 0) || (w == 0U)) {
        return v;
    }
    off2 = (int32_t)(w - 1U) - (int32_t)(2U * k);
    num = 2000 + (int32_t)g_ramp_slope * off2;
    if (num < 1) { num = 1; }
    return (uint16_t)(((int32_t)v * num) / 2000);
}

/* Discard g_warmup measurements on `center` before an axis scan so the pen tank
 * and front end start from a repeatable state. */
static void warmup_axis(const uint8_t *table, uint8_t center)
{
    if ((g_warmup == 0U) || (center == 0U)) { return; }
    const uint8_t *e = coil_entry(table, center);
    for (uint8_t i = 0U; i < g_warmup; i++) {
        (void)acq_measure(coil_mask(e), coil_value(e));
    }
}

/* Pre-scan prep for one axis: warm-up reads plus (mode 2) a prime on the fixed
 * reference. */
static void prep_axis(const uint8_t *table, uint8_t center)
{
    warmup_axis(table, center);
    if (g_ramp_mode == PROTO_RAMP_PRIME_WIN) {
        acq_prime(table, center);
    }
}

/* Measure the `w` coils around `center` into `out` (no zeroing). `tmp` is the
 * second buffer for the bidirectional-average mode (4). */
static void fill_axis_window(uint8_t n, const uint8_t *table, uint16_t *out,
                             uint8_t center, uint8_t w, uint16_t *tmp)
{
    uint8_t lo, hi, k;

    if (w == 0U) { return; }
    win_bounds(n, center, w, &lo, &hi);

    if (g_ramp_mode == PROTO_RAMP_BIDI) {
        for (uint8_t idx = lo; idx <= hi; idx++) {
            measure_idx(table, out, idx);
        }
        for (uint16_t idx = hi; idx >= (uint16_t)lo; idx--) {
            measure_idx(table, tmp, (uint8_t)idx);
        }
        for (uint8_t idx = lo; idx <= hi; idx++) {
            out[idx] = (uint16_t)(((uint32_t)out[idx] + (uint32_t)tmp[idx]) / 2U);
        }
        return;
    }

    if (g_ramp_mode == PROTO_RAMP_TARGET_REV) {
        /* Forward-scan the window, then reverse-scan only the 2R+1 coils around
         * the centre and average those. */
        uint8_t clo, chi;

        for (uint8_t idx = lo; idx <= hi; idx++) {
            measure_idx(table, out, idx);
        }
        win_bounds(n, center, (uint8_t)(2U * g_rev_radius + 1U), &clo, &chi);
        if (clo < lo) { clo = lo; }
        if (chi > hi) { chi = hi; }
        for (uint16_t idx = chi; idx >= (uint16_t)clo; idx--) {
            measure_idx(table, tmp, (uint8_t)idx);
        }
        for (uint8_t idx = clo; idx <= chi; idx++) {
            out[idx] = (uint16_t)(((uint32_t)out[idx] + (uint32_t)tmp[idx]) / 2U);
        }
        return;
    }

    k = 0U;
    if (g_scan_order == PROTO_SCAN_ASC) {
        for (uint8_t idx = lo; idx <= hi; idx++, k++) {
            measure_idx(table, out, idx);
            out[idx] = ramp_slope_apply(out[idx], k, w);
        }
    } else {
        for (uint16_t idx = hi; idx >= (uint16_t)lo; idx--, k++) {
            measure_idx(table, out, (uint8_t)idx);
            out[idx] = ramp_slope_apply(out[idx], k, w);
        }
    }
}

/* Measure only the `w` coils around `center`; zero the rest so the host sees
 * exactly what was sampled. */
static void scan_axis_window(uint8_t n, const uint8_t *table, uint16_t *out,
                             uint8_t center, uint8_t w, uint16_t *tmp)
{
    for (uint8_t i = 1U; i <= n; i++) {
        out[i] = 0U;
    }
    fill_axis_window(n, table, out, center, w, tmp);
}

/* Full-grid axis scan with the bidirectional ramp modes applied. */
static void scan_axis_table(uint8_t n, const uint8_t *table, uint16_t *out, uint16_t *tmp)
{
    if ((g_ramp_mode == PROTO_RAMP_BIDI) || (g_ramp_mode == PROTO_RAMP_TARGET_REV)) {
        acq_scan_table(n, table, out, 0U);
        acq_scan_table(n, table, tmp, 1U);
        for (uint8_t i = 1U; i <= n; i++) {
            out[i] = (uint16_t)(((uint32_t)out[i] + (uint32_t)tmp[i]) / 2U);
        }
        return;
    }

    {
        uint8_t ord = (g_scan_order == PROTO_SCAN_DESC) ? 1U : 0U;
        acq_scan_table(n, table, out, ord);
        if (g_ramp_mode == PROTO_RAMP_SLOPE) {
            for (uint8_t i = 1U; i <= n; i++) {
                uint8_t k = (ord == 0U) ? (uint8_t)(i - 1U) : (uint8_t)(n - i);
                out[i] = ramp_slope_apply(out[i], k, n);
            }
        }
    }
}

/* ---- interleaved window scans (orders 2/3) ------------------------------ */
/* Next index in the outside-in order: lo, hi, lo+1, hi-1, ... */
static uint8_t outin_next(uint8_t *lo, uint8_t *hi, uint8_t *side)
{
    uint8_t v;

    if (*side == 0U) {
        v = *lo;
        if (*lo >= *hi) {
            *lo = (uint8_t)(*hi + 1U);        /* exhaust */
        } else {
            (*lo)++;
            *side = 1U;
        }
    } else {
        v = *hi;
        (*hi)--;
        *side = 0U;
    }
    return v;
}

typedef struct {
    uint8_t lo, hi, center;
    uint8_t left, right;
    uint8_t started, side;
} scan_seq_t;

static void seq_init(scan_seq_t *s, uint8_t lo, uint8_t hi, uint8_t center)
{
    if (lo > hi) {
        /* Empty range (e.g. a disabled axis window): mark the sequence
         * exhausted so seq_more() is false and seq_next() is never called.
         * Without this, center clamps to 0 and measure_idx() would read
         * table[-1]. */
        s->lo = lo;
        s->hi = hi;
        s->center = lo;
        s->left = lo;
        s->right = (uint8_t)(hi + 1U);
        s->started = 1U;
        s->side = 0U;
        return;
    }
    if (center < lo) { center = lo; }
    if (center > hi) { center = hi; }
    s->lo = lo;
    s->hi = hi;
    s->center = center;
    s->left = center;
    s->right = (uint8_t)(center + 1U);
    s->started = 0U;
    s->side = 0U;
}

static int seq_more(const scan_seq_t *s)
{
    return (s->started == 0U) || (s->left > s->lo) || (s->right <= s->hi);
}

static uint8_t seq_next(scan_seq_t *s)
{
    if (s->started == 0U) {
        s->started = 1U;
        return s->center;
    }
    if ((s->side == 0U) && (s->left > s->lo)) {
        s->left--;
        s->side = 1U;
        return s->left;
    }
    if (s->right <= s->hi) {
        uint8_t v = s->right;
        s->right++;
        s->side = 0U;
        return v;
    }
    if (s->left > s->lo) {
        s->left--;
        return s->left;
    }
    return s->center;                      /* unreachable while more() is true */
}

/* Outside-in interleaved window scan: both axes measured in layers from the
 * outermost coil inward, alternating axes. 5+5 -> X1 Y1 X5 Y5 X2 Y2 X4 Y4 X3 Y3. */
static void scan_both_outin(uint8_t zero,
                            uint8_t n_b, const uint8_t *tb, uint16_t *ob,
                            uint8_t cb, uint8_t w_b,
                            uint8_t n_a, const uint8_t *ta, uint16_t *oa,
                            uint8_t ca, uint8_t w_a)
{
    uint8_t lb, hb, la, ha, sb = 0U, sa = 0U, turn = 0U;

    if (zero != 0U) {
        for (uint8_t i = 1U; i <= n_b; i++) { ob[i] = 0U; }
        for (uint8_t i = 1U; i <= n_a; i++) { oa[i] = 0U; }
    }
    if ((w_b == 0U) && (w_a == 0U)) { return; }
    if (w_b != 0U) { win_bounds(n_b, cb, w_b, &lb, &hb); }
    else { lb = 1U; hb = 0U; }
    if (w_a != 0U) { win_bounds(n_a, ca, w_a, &la, &ha); }
    else { la = 1U; ha = 0U; }

    for (;;) {
        int did = 0;
        if ((turn == 0U) && (lb <= hb)) {
            measure_idx(tb, ob, outin_next(&lb, &hb, &sb));
            did = 1;
        } else if ((turn == 1U) && (la <= ha)) {
            measure_idx(ta, oa, outin_next(&la, &ha, &sa));
            did = 1;
        }
        if (did == 0) {
            if ((turn == 0U) && (la <= ha)) {
                measure_idx(ta, oa, outin_next(&la, &ha, &sa));
                did = 1;
            } else if ((turn == 1U) && (lb <= hb)) {
                measure_idx(tb, ob, outin_next(&lb, &hb, &sb));
                did = 1;
            }
        }
        if (did == 0) { break; }
        turn ^= 1U;
    }
}

/* Center-out interleaved window scan: both axes from the centre outward,
 * alternating axes. 5+5 -> X3 Y3 X2 Y2 X4 Y4 X1 Y1 X5 Y5. */
static void scan_both_inout(uint8_t zero,
                            uint8_t n_b, const uint8_t *tb, uint16_t *ob,
                            uint8_t cb, uint8_t w_b,
                            uint8_t n_a, const uint8_t *ta, uint16_t *oa,
                            uint8_t ca, uint8_t w_a)
{
    scan_seq_t sb, sa;
    uint8_t lo, hi, turn = 0U;

    if (zero != 0U) {
        for (uint8_t i = 1U; i <= n_b; i++) { ob[i] = 0U; }
        for (uint8_t i = 1U; i <= n_a; i++) { oa[i] = 0U; }
    }
    if ((w_b == 0U) && (w_a == 0U)) { return; }
    if (w_b != 0U) { win_bounds(n_b, cb, w_b, &lo, &hi); }
    else { lo = 1U; hi = 0U; }
    seq_init(&sb, lo, hi, cb);
    if (w_a != 0U) { win_bounds(n_a, ca, w_a, &lo, &hi); }
    else { lo = 1U; hi = 0U; }
    seq_init(&sa, lo, hi, ca);

    for (;;) {
        int more_b = seq_more(&sb);
        int more_a = seq_more(&sa);
        int did = 0;

        if (!more_b && !more_a) { break; }
        if ((turn == 0U) && more_b) {
            measure_idx(tb, ob, seq_next(&sb));
            did = 1;
        } else if ((turn == 1U) && more_a) {
            measure_idx(ta, oa, seq_next(&sa));
            did = 1;
        }
        if (did == 0) {
            if ((turn == 0U) && more_a) {
                measure_idx(ta, oa, seq_next(&sa));
                did = 1;
            } else if ((turn == 1U) && more_b) {
                measure_idx(tb, ob, seq_next(&sb));
                did = 1;
            }
        }
        if (did == 0) { break; }
        turn ^= 1U;
    }
}

/* Scan both axes' tracking windows with the configured scan order. */
static void scan_windows(uint8_t zero,
                         uint8_t n_b, const uint8_t *tb, uint16_t *ob,
                         uint8_t cb, uint8_t w_b, uint16_t *tmp_b,
                         uint8_t n_a, const uint8_t *ta, uint16_t *oa,
                         uint8_t ca, uint8_t w_a, uint16_t *tmp_a)
{
    if (g_scan_order == PROTO_SCAN_OUTIN) {
        scan_both_outin(zero, n_b, tb, ob, cb, w_b, n_a, ta, oa, ca, w_a);
        return;
    }
    if (g_scan_order == PROTO_SCAN_INOUT) {
        scan_both_inout(zero, n_b, tb, ob, cb, w_b, n_a, ta, oa, ca, w_a);
        return;
    }
    if (zero != 0U) {
        scan_axis_window(n_b, tb, ob, cb, w_b, tmp_b);
        scan_axis_window(n_a, ta, oa, ca, w_a, tmp_a);
    } else {
        fill_axis_window(n_b, tb, ob, cb, w_b, tmp_b);
        fill_axis_window(n_a, ta, oa, ca, w_a, tmp_a);
    }
}

/* Sparse scan: measure every `stride`-th coil from 1+phase, zero the rest. The
 * rotating phase guarantees a narrow peak is sampled within `stride` frames. */
static void scan_axis_coarse(uint8_t n, const uint8_t *table, uint16_t *out,
                             uint8_t stride, uint8_t phase)
{
    for (uint8_t i = 1U; i <= n; i++) {
        out[i] = 0U;
    }
    if (stride < 2U) { stride = 2U; }
    for (uint8_t idx = (uint8_t)(1U + phase); idx <= n; idx = (uint8_t)(idx + stride)) {
        measure_idx(table, out, idx);
    }
}

/* ---- re-centre strategies ----------------------------------------------- */
/* Apply the configured shift given a target coil and the current centre. */
static uint8_t recenter_move(uint8_t n, uint8_t center, int target)
{
    int delta;
    uint8_t step = g_rc_step;

    if ((step == 0U) ||
        ((((target > (int)center) ? (target - (int)center)
                                  : ((int)center - target)) <= (int)step))) {
        delta = target;
    } else {
        delta = (int)center + ((target > (int)center) ? (int)step : -(int)step);
    }
    if (delta < 1) { delta = 1; }
    if (delta > (int)n) { delta = (int)n; }
    return (uint8_t)delta;
}

/* Sticky (de-entangled) re-centre: follow the sub-pixel position, shifting only
 * after it stays outside a +/-hysteresis band (percent of a pitch) for
 * `persist` frames, and then at most `step` coils. */
static uint8_t sticky_center(uint8_t n, uint8_t center, int32_t fp, uint8_t *pending)
{
    int32_t err, hyst;
    int target;

    if ((center < 1U) || (center > n) || (fp <= 0)) {
        return center;
    }
    err = fp - ((int32_t)center << 8);
    hyst = ((int32_t)g_rc_hyst * 256) / 100;
    if ((err <= hyst) && (err >= -hyst)) {
        *pending = 0U;
        return center;
    }
    if (*pending < 0xFFU) {
        (*pending)++;
    }
    if (*pending < g_rc_persist) {
        return center;
    }
    target = (int)((fp + 128) >> 8);
    if (target < 1) { target = 1; }
    if (target > (int)n) { target = (int)n; }
    *pending = 0U;
    return recenter_move(n, center, target);
}

/* Legacy distance-deadband sticky: shift when the sub-pixel position is more
 * than `deadband` whole coils from the centre for `persist` frames. */
static uint8_t deadband_center(uint8_t n, uint8_t center, int32_t fp, uint8_t *pending)
{
    int32_t err, guard;
    int target;

    if ((center < 1U) || (center > n) || (fp <= 0)) {
        return center;
    }
    err = fp - ((int32_t)center << 8);
    guard = (int32_t)g_rc_deadband << 8;
    if ((err <= guard) && (err >= -guard)) {
        *pending = 0U;
        return center;
    }
    if (*pending < 0xFFU) {
        (*pending)++;
    }
    if (*pending < g_rc_persist) {
        return center;
    }
    target = (int)((fp + 128) >> 8);
    if (target < 1) { target = 1; }
    if (target > (int)n) { target = (int)n; }
    *pending = 0U;
    return recenter_move(n, center, target);
}

/* ---- frame build -------------------------------------------------------- */
static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint16_t frame_flags(uint16_t amax_b, uint16_t amax_a)
{
    uint16_t f = 0U;

    if ((amax_b > PROTO_NEAR_THR) || (amax_a > PROTO_NEAR_THR)) {
        f |= PROTO_FLAG_PEN;
    }
    if (g_win_x != 0U) { f |= PROTO_FLAG_WINDOWED; }
    if (g_reacquired != 0U) { f |= PROTO_FLAG_REACQUIRED; }
    if (g_sat_retry != 0U) { f |= PROTO_FLAG_SAT_RETRY; }
    if (g_recenter == PROTO_RECENTER_FOLLOW) { f |= PROTO_FLAG_FOLLOW; }
    if ((g_recenter == PROTO_RECENTER_STICKY) ||
        (g_recenter == PROTO_RECENTER_DEADBAND)) { f |= PROTO_FLAG_STICKY; }
    if (g_scan_order == PROTO_SCAN_DESC) { f |= PROTO_FLAG_DESCENDING; }
    if (g_free_run != 0U) { f |= PROTO_FLAG_CARRYOVER; }
    return f;
}

static void build_frame(uint8_t *tx)
{
    uint8_t bx = 0U, by = 0U;
    uint16_t amax_b, amax_a;
    int32_t xfp, yfp;

    amax_b = peak_max(g_prof_b, AXIS_B_N, &bx);
    amax_a = peak_max(g_prof_a, AXIS_A_N, &by);

    xfp = estimator_position(&g_prof_b[1], (uint8_t)AXIS_B_N, 0U, 0U);
    yfp = estimator_position(&g_prof_a[1], (uint8_t)AXIS_A_N, 0U, 1U);

    if (xfp < 0) { xfp = 0; } else if (xfp > 65535) { xfp = 65535; }
    if (yfp < 0) { yfp = 0; } else if (yfp > 65535) { yfp = 65535; }

    if (g_ramp_mode == PROTO_RAMP_FRAME_AVG) {
        /* Alternate scan order every frame so consecutive frames carry mirrored
         * ramp errors; average their positions (one frame of latency). */
        int32_t cx = xfp, cy = yfp;
        if (g_prev_pos_valid != 0U) {
            xfp = (xfp + g_prev_xfp) / 2;
            yfp = (yfp + g_prev_yfp) / 2;
        }
        g_prev_xfp = cx;
        g_prev_yfp = cy;
        g_prev_pos_valid = 1U;
    }
    g_xfp = xfp;
    g_yfp = yfp;

    uint8_t backend = (g_repeat_coil != 0U) ? PROTO_BACKEND_REPEAT
                                            : (uint8_t)acq_get_backend();

    tx[PROTO_OFF_MAGIC0]    = PROTO_MAGIC0;
    tx[PROTO_OFF_MAGIC1]    = PROTO_MAGIC1;
    tx[PROTO_OFF_VERSION]   = PROTO_VERSION;
    tx[PROTO_OFF_SEQ]       = 0U;                       /* transport rewrites */
    put16(&tx[PROTO_OFF_FLAGS], frame_flags(amax_b, amax_a));
    tx[PROTO_OFF_BACKEND]   = backend;
    tx[PROTO_OFF_ESTIMATOR] = estimator_get();
    put16(&tx[PROTO_OFF_XPOS], (uint16_t)xfp);
    put16(&tx[PROTO_OFF_YPOS], (uint16_t)yfp);
    tx[PROTO_OFF_FREQ]      = acq_get_freq();
    tx[PROTO_OFF_BURST]     = acq_get_burst();
    tx[PROTO_OFF_ADC_N]     = acq_get_adc_n();
    tx[PROTO_OFF_ADC_CLK]   = acq_get_adc_clk();
    tx[PROTO_OFF_WIN_X]     = g_win_x;
    tx[PROTO_OFF_WIN_Y]     = g_win_y;
    tx[PROTO_OFF_RAMP]      = g_ramp_mode;
    tx[PROTO_OFF_RECENTER]  = g_recenter;
    tx[PROTO_OFF_SCAN_ORDER] = g_scan_order;
    tx[PROTO_OFF_WARMUP]    = g_warmup;
    tx[PROTO_OFF_PEAK_X]    = bx;
    tx[PROTO_OFF_PEAK_Y]    = by;
    put32(&tx[PROTO_OFF_TIME_US], dwt_now_us());
    put32(&tx[PROTO_OFF_SCAN_US], g_scan_us);

    for (uint8_t i = 0U; i < (uint8_t)AXIS_B_N; i++) {
        put16(&tx[PROTO_OFF_AMP_X + (uint16_t)i * 2U], g_prof_b[i + 1U]);
    }
    for (uint8_t i = 0U; i < (uint8_t)AXIS_A_N; i++) {
        put16(&tx[PROTO_OFF_AMP_Y + (uint16_t)i * 2U], g_prof_a[i + 1U]);
    }
}

/* ---- repeat-coil diagnostic --------------------------------------------- */
static void repeat_step(void)
{
    const uint8_t *e = coil_entry(axis_b_table, g_repeat_coil);

    for (uint8_t i = (uint8_t)PROTO_NX - 1U; i > 0U; i--) {
        g_repeat_hist[i] = g_repeat_hist[i - 1U];
    }
    g_repeat_hist[0] = acq_measure(coil_mask(e), coil_value(e));

    for (uint8_t i = 0U; i < (uint8_t)PROTO_NX; i++) {
        g_prof_b[i + 1U] = g_repeat_hist[i];
    }
    for (uint8_t i = 0U; i < (uint8_t)AXIS_A_N; i++) {
        g_prof_a[i + 1U] = 0U;
    }
}

/* ---- pacing ------------------------------------------------------------- */
static void pacing_apply(void)
{
    if (g_pacing == PROTO_PACING_CONTINUOUS) {
        g_free_run = 1U;
    } else if (g_pacing == PROTO_PACING_SOF) {
        g_free_run = 0U;
    } else {
        g_free_run = (g_ramp_mode == PROTO_RAMP_CARRYOVER) ? 1U : 0U;
    }
}

/* ---- public API --------------------------------------------------------- */
void scan_init(void)
{
    pacing_apply();
}

void scan_step(uint8_t *frame)
{
    uint32_t start;
    uint8_t tracking;

    start = DWT->CYCCNT;
    g_sat_retry = 0U;

    if (g_ramp_mode == PROTO_RAMP_FRAME_AVG) {
        g_scan_order = (g_frame_toggle != 0U) ? PROTO_SCAN_DESC : PROTO_SCAN_ASC;
        g_frame_toggle ^= 1U;
    }

    /* While not tracking, only attempt a re-acquire every g_reacq_period_ms; in
     * between the scanner is idle and the last profile is re-sent. */
    tracking = ((g_repeat_coil == 0U) && (g_win_x != 0U) &&
                (g_have_center != 0U) && (g_prev_amp >= g_reacq_thr)) ? 1U : 0U;
    if ((g_repeat_coil == 0U) && (tracking == 0U)) {
        if (g_have_center != 0U) {              /* lock lost -> go cold */
            g_have_center = 0U;
            g_next_reacq_cyc = DWT->CYCCNT + (uint32_t)g_reacq_period_ms * 72000U;
            g_reacq_ts_valid = 1U;
        }
        if ((g_reacq_period_ms != 0U) && (g_reacq_ts_valid != 0U) &&
            ((int32_t)(DWT->CYCCNT - g_next_reacq_cyc) < 0)) {
            g_reacquired = 0U;
            g_scan_us = 0U;
            build_frame(frame);
            return;
        }
    }

    if (g_repeat_coil != 0U) {
        repeat_step();
        g_reacquired = 0U;
    } else if ((g_win_x != 0U) && (g_have_center != 0U) && (g_prev_amp >= g_reacq_thr)) {
        /* Track: narrow window around the last peak. */
        uint8_t cb = ((g_last_bx >= 1U) && (g_last_bx <= (uint8_t)AXIS_B_N))
                         ? g_last_bx : (uint8_t)(AXIS_B_N / 2U);
        uint8_t ca = ((g_last_by >= 1U) && (g_last_by <= (uint8_t)AXIS_A_N))
                         ? g_last_by : (uint8_t)(AXIS_A_N / 2U);
        uint8_t bx, by;
        uint16_t amax;

        /* Warm up on the fixed reference, not the moving window centre. */
        prep_axis(axis_b_table, (uint8_t)(AXIS_B_N / 2U + 1U));
        prep_axis(axis_a_table, (uint8_t)(AXIS_A_N / 2U + 1U));
        scan_windows(1U,
                     (uint8_t)AXIS_B_N, axis_b_table, g_prof_b, cb, g_win_x, g_tmp_b,
                     (uint8_t)AXIS_A_N, axis_a_table, g_prof_a, ca, g_win_y, g_tmp_a);

        /* Saturation guard: a peak at/above 0xF3C clips the front end and
         * flattens the profile, so re-read the window once. */
        amax = peak_max(g_prof_b, AXIS_B_N, &bx);
        if (peak_max(g_prof_a, AXIS_A_N, &by) > amax) { amax = g_prof_a[by]; }
        if (amax >= ESTIMATOR_VENDOR_SAT) {
            scan_windows(1U,
                         (uint8_t)AXIS_B_N, axis_b_table, g_prof_b, cb, g_win_x, g_tmp_b,
                         (uint8_t)AXIS_A_N, axis_a_table, g_prof_a, ca, g_win_y, g_tmp_a);
            g_sat_retry = 1U;
        }
        g_reacquired = 0U;
    } else if ((g_win_x != 0U) && (g_coarse_stride >= 2U)) {
        /* Re-acquire with a coarse scan and a rotating phase, then a fine window
         * around the best coarse sample. */
        uint8_t stride = g_coarse_stride;
        uint8_t phase = (uint8_t)(g_coarse_phase % stride);
        uint8_t bx = 0U, by = 0U;
        uint16_t amax;

        scan_axis_coarse((uint8_t)AXIS_B_N, axis_b_table, g_prof_b, stride, phase);
        scan_axis_coarse((uint8_t)AXIS_A_N, axis_a_table, g_prof_a, stride, phase);

        amax = peak_max(g_prof_b, AXIS_B_N, &bx);
        if (peak_max(g_prof_a, AXIS_A_N, &by) > amax) { amax = g_prof_a[by]; }

        if (amax > 5U) {
            uint8_t cb = (bx != 0U) ? bx : (uint8_t)(AXIS_B_N / 2U);
            uint8_t ca = (by != 0U) ? by : (uint8_t)(AXIS_A_N / 2U);
            prep_axis(axis_b_table, (uint8_t)(AXIS_B_N / 2U + 1U));
            prep_axis(axis_a_table, (uint8_t)(AXIS_A_N / 2U + 1U));
            scan_windows(0U,
                         (uint8_t)AXIS_B_N, axis_b_table, g_prof_b, cb, g_win_x, g_tmp_b,
                         (uint8_t)AXIS_A_N, axis_a_table, g_prof_a, ca, g_win_y, g_tmp_a);
        }
        g_coarse_phase = (uint8_t)((g_coarse_phase + 1U) % stride);
        g_reacquired = 1U;
        g_next_reacq_cyc = DWT->CYCCNT + (uint32_t)g_reacq_period_ms * 72000U;
        g_reacq_ts_valid = 1U;
    } else {
        /* Acquire: full grid, so the pen can reappear anywhere. */
        prep_axis(axis_b_table, (uint8_t)(AXIS_B_N / 2U + 1U));
        prep_axis(axis_a_table, (uint8_t)(AXIS_A_N / 2U + 1U));
        scan_axis_table((uint8_t)AXIS_B_N, axis_b_table, g_prof_b, g_tmp_b);
        scan_axis_table((uint8_t)AXIS_A_N, axis_a_table, g_prof_a, g_tmp_a);
        g_reacquired = (g_win_x != 0U) ? 1U : 0U;
        g_next_reacq_cyc = DWT->CYCCNT + (uint32_t)g_reacq_period_ms * 72000U;
        g_reacq_ts_valid = 1U;
    }

    g_scan_us = (DWT->CYCCNT - start) / 72U;

#if SCAN_KEEPALIVE
    if (g_free_run != 0U) {
        acq_keepalive((uint16_t)SCAN_KEEPALIVE);
    }
#endif
    build_frame(frame);
#if SCAN_KEEPALIVE
    if (g_free_run != 0U) {
        acq_keepalive_stop();
    }
#endif

    /* Update tracking state from the scan we just did. */
    if ((g_repeat_coil == 0U) && (g_win_x != 0U)) {
        uint8_t bx = 0U, by = 0U, sx = 0U, sy = 0U, t;
        uint16_t amax;

        peak_find(g_prof_b, (uint8_t)AXIS_B_N, &bx, &sx, &t);
        peak_find(g_prof_a, (uint8_t)AXIS_A_N, &by, &sy, &t);
        amax = (g_prof_b[bx] > g_prof_a[by]) ? g_prof_b[bx] : g_prof_a[by];
        g_prev_amp = amax;

        if (g_reacquired != 0U) {
            if (((bx != 0U) || (by != 0U)) && (amax >= g_reacq_thr)) {
                if (bx != 0U) { g_last_bx = bx; }
                if (by != 0U) { g_last_by = by; }
                g_have_center = 1U;
            } else {
                g_have_center = 0U;      /* stay in acquire */
            }
        } else if (g_have_center != 0U) {
            if (g_recenter == PROTO_RECENTER_STICKY) {
                if (g_xfp > 0) {
                    g_last_bx = sticky_center((uint8_t)AXIS_B_N, g_last_bx, g_xfp, &g_rc_pend_x);
                }
                if (g_yfp > 0) {
                    g_last_by = sticky_center((uint8_t)AXIS_A_N, g_last_by, g_yfp, &g_rc_pend_y);
                }
            } else if (g_recenter == PROTO_RECENTER_DEADBAND) {
                if (g_xfp > 0) {
                    g_last_bx = deadband_center((uint8_t)AXIS_B_N, g_last_bx, g_xfp, &g_rc_pend_x);
                }
                if (g_yfp > 0) {
                    g_last_by = deadband_center((uint8_t)AXIS_A_N, g_last_by, g_yfp, &g_rc_pend_y);
                }
            } else if (g_recenter == PROTO_RECENTER_FOLLOW) {
                /* Re-centre on the peak channel every frame, but hold the centre
                 * when the top two coils are within g_flat_tol (noise flip). */
                uint8_t flat_b = 0U, flat_a = 0U;
                if (g_flat_tol > 0U) {
                    if ((bx != 0U) && (sx != 0U)) {
                        flat_b = ((uint32_t)g_prof_b[bx] - (uint32_t)g_prof_b[sx]
                                      < g_flat_tol) ? 1U : 0U;
                    }
                    if ((by != 0U) && (sy != 0U)) {
                        flat_a = ((uint32_t)g_prof_a[by] - (uint32_t)g_prof_a[sy]
                                      < g_flat_tol) ? 1U : 0U;
                    }
                } else {
                    (void)sx;
                    (void)sy;
                }
                if ((bx != 0U) && (flat_b == 0U)) { g_last_bx = bx; }
                if ((by != 0U) && (flat_a == 0U)) { g_last_by = by; }
            } else {
                /* Edge-only re-centring: slide only when the peak sits on the
                 * window edge, so interior peaks leave the window alone. */
                uint8_t lo_b, hi_b, lo_a, hi_a;
                win_bounds((uint8_t)AXIS_B_N, g_last_bx, g_win_x, &lo_b, &hi_b);
                win_bounds((uint8_t)AXIS_A_N, g_last_by, g_win_y, &lo_a, &hi_a);
                if ((bx != 0U) && ((bx == lo_b) || (bx == hi_b))) { g_last_bx = bx; }
                if ((by != 0U) && ((by == lo_a) || (by == hi_a))) { g_last_by = by; }
            }
        }
    } else {
        g_reacquired = 0U;
    }
}

uint8_t scan_is_free_run(void)
{
    return g_free_run;
}

/* ---- settings ----------------------------------------------------------- */
void scan_set_window(uint8_t x_coils, uint8_t y_coils)
{
    if (x_coils > 8U) { x_coils = 8U; }
    if (y_coils > 8U) { y_coils = 8U; }
    if (x_coils == 1U) { x_coils = 2U; }        /* 1 coil is meaningless */
    if (y_coils == 1U) { y_coils = 2U; }
    g_win_x = x_coils;
    g_win_y = y_coils;
}

void scan_set_recenter(uint8_t mode, uint8_t hyst, uint8_t persist,
                       uint8_t step, uint8_t deadband)
{
    g_recenter = (mode <= PROTO_RECENTER_DEADBAND) ? mode : PROTO_RECENTER_FOLLOW;
    if (hyst != 0U) { g_rc_hyst = hyst; }
    if (persist != 0U) { g_rc_persist = persist; }
    g_rc_step = step;                           /* 0 = full snap */
    if (deadband != 0U) { g_rc_deadband = deadband; }
}

void scan_set_scan_order(uint8_t order)
{
    g_scan_order = (order <= PROTO_SCAN_INOUT) ? order : PROTO_SCAN_ASC;
}

void scan_set_warmup(uint8_t reads)
{
    g_warmup = (reads <= 8U) ? reads : 8U;
}

void scan_set_flat_tol(uint16_t tol)
{
    g_flat_tol = tol;
}

void scan_set_ramp(uint8_t mode, uint8_t prime_burst, uint8_t prime_repeats,
                   int16_t slope_per_mille, uint8_t rev_radius)
{
    g_ramp_mode = (mode <= PROTO_RAMP_FRAME_AVG) ? mode : PROTO_RAMP_NONE;
    g_ramp_slope = slope_per_mille;
    if (g_ramp_slope > 1000) { g_ramp_slope = 1000; }
    if (g_ramp_slope < -1000) { g_ramp_slope = -1000; }
    if (rev_radius != 0U) { g_rev_radius = (rev_radius > 8U) ? 8U : rev_radius; }
    acq_set_prime(prime_burst, prime_repeats);
    pacing_apply();
    acq_set_read_burst((g_ramp_mode == PROTO_RAMP_READ_STEADY) ? acq_get_prime_burst() : 0U);
    g_prev_pos_valid = 0U;
}

void scan_set_reacq(uint16_t threshold, uint8_t coarse_stride, uint16_t period_ms)
{
    uint8_t stride;

    g_reacq_thr = threshold;

    if (coarse_stride > 8U) { coarse_stride = 8U; }
    stride = (coarse_stride < 2U) ? 0U : coarse_stride;
    if (stride != g_coarse_stride) {
        g_coarse_stride = stride;
        g_coarse_phase = 0U;                 /* restart the rotating phase */
    }

    /* Keep period*72000 inside the signed 32-bit wrap-safe cycle delta. */
    if (period_ms > 29000U) { period_ms = 29000U; }
    if (period_ms != g_reacq_period_ms) {
        g_reacq_period_ms = period_ms;
        g_reacq_ts_valid = 0U;               /* re-arm the cold gate */
    }
}

void scan_set_repeat_coil(uint8_t coil)
{
    g_repeat_coil = (coil <= (uint8_t)PROTO_NX) ? coil : 0U;
}

void scan_set_pacing(uint8_t pacing)
{
    g_pacing = (pacing <= PROTO_PACING_AUTO) ? pacing : PROTO_PACING_AUTO;
    pacing_apply();
}
