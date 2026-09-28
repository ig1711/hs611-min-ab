/*
 * acq.c — runtime acquisition configuration and backend dispatch.
 */

#include "acq.h"
#include "board.h"
#include "afe.h"
#include "hw_timer.h"
#include "coil_tables.h"

static uint8_t  g_backend = ACQ_BACKEND_HW;
static uint8_t  g_freq = ACQ_DEFAULT_FREQ;
static uint16_t g_freq_arr;                  /* 0 = use the freq index table */
static uint8_t  g_burst = ACQ_DEFAULT_BURST;
static uint16_t g_settle_cyc[4] = { 144U, 72U, 432U, 288U };  /* 2/1/6/4 us */
static uint8_t  g_adc_n = ACQ_DEFAULT_ADC_N;
static uint8_t  g_adc_clk = ACQ_DEFAULT_ADC_CLK;
static uint8_t  g_recovery;
static uint16_t g_read_burst;                /* 0 = use g_burst */
static uint8_t  g_prime_burst = 64U;
static uint8_t  g_prime_repeats = 4U;

static void backend_apply(void)
{
    if (g_backend == ACQ_BACKEND_HW) {
        hw_adc_clk_set(g_adc_clk);
        hw_carrier_set(g_freq, g_freq_arr);
        hw_enable();
    } else {
        hw_disable();
    }
}

void acq_init(void)
{
    hw_timer_init();
    backend_apply();
}

uint16_t acq_measure(uint16_t mask_b, uint16_t val_c)
{
    if (g_backend == ACQ_BACKEND_HW) {
        return hw_measure(mask_b, val_c, g_burst, g_adc_n, g_settle_cyc, g_recovery,
                          g_read_burst);
    }
    return afe_measure_sw(mask_b, val_c, g_freq, g_settle_cyc);
}

void acq_scan_table(uint8_t n, const uint8_t *table, uint16_t *out, uint8_t desc)
{
    out[0] = 0U;
    if (desc == 0U) {
        for (uint8_t i = 1U; i <= n; i++) {
            const uint8_t *e = coil_entry(table, i);
            out[i] = acq_measure(coil_mask(e), coil_value(e));
        }
    } else {
        for (uint16_t i = n; i >= 1U; i--) {
            const uint8_t *e = coil_entry(table, (uint8_t)i);
            out[i] = acq_measure(coil_mask(e), coil_value(e));
        }
    }
}

void acq_prime(const uint8_t *table, uint8_t idx)
{
    const uint8_t *e;

    if (idx == 0U) { return; }
    e = coil_entry(table, idx);

    if (g_backend == ACQ_BACKEND_HW) {
        hw_prime(coil_mask(e), coil_value(e), g_prime_burst, g_prime_repeats);
    } else {
        /* No standalone carrier on the software backend: one discarded read. */
        (void)acq_measure(coil_mask(e), coil_value(e));
    }
}

void acq_keepalive(uint16_t periods)
{
    if (g_backend == ACQ_BACKEND_HW) {
        hw_keepalive(periods);
    }
}

void acq_keepalive_stop(void)
{
    if (g_backend == ACQ_BACKEND_HW) {
        hw_keepalive_stop();
    }
}

/* ---- settings ----------------------------------------------------------- */
void acq_set_backend(uint8_t backend)
{
    g_backend = (backend == ACQ_BACKEND_HW) ? ACQ_BACKEND_HW : ACQ_BACKEND_SW;
    backend_apply();
}

uint8_t acq_get_backend(void) { return g_backend; }

void acq_set_freq(uint8_t freq)
{
    if ((freq >= 1U) && (freq <= 12U)) {
        g_freq = freq;
        g_freq_arr = 0U;                    /* index table takes back over */
        if (g_backend == ACQ_BACKEND_HW) {
            hw_carrier_set(g_freq, g_freq_arr);
        }
    }
}

void acq_set_freq_arr(uint16_t arr)
{
    g_freq_arr = arr;                       /* hw_carrier_set clamps non-zero */
    if (g_backend == ACQ_BACKEND_HW) {
        hw_carrier_set(g_freq, g_freq_arr);
    }
}

void acq_set_burst(uint8_t periods)
{
    if (periods < BURST_MIN) { periods = BURST_MIN; }
    if (periods > BURST_MAX) { periods = BURST_MAX; }
    g_burst = periods;
}

static uint16_t us_to_cycles(uint8_t us)
{
    uint32_t c = (uint32_t)us * 72U;
    return (uint16_t)((c > 65535U) ? 65535U : c);
}

void acq_set_settle(uint8_t a, uint8_t b, uint8_t c, uint8_t d)
{
    /* Microsecond convenience: 1 us = 72 DWT cycles. */
    g_settle_cyc[0] = us_to_cycles(a);
    g_settle_cyc[1] = us_to_cycles(b);
    g_settle_cyc[2] = us_to_cycles(c);
    g_settle_cyc[3] = us_to_cycles(d);
}

void acq_set_settle_cycles(uint8_t site, uint16_t cycles)
{
    if (site < 4U) {
        g_settle_cyc[site] = cycles;
    }
}

void acq_set_settle_cycles_all(uint16_t a, uint16_t b, uint16_t c, uint16_t d)
{
    g_settle_cyc[0] = a;
    g_settle_cyc[1] = b;
    g_settle_cyc[2] = c;
    g_settle_cyc[3] = d;
}

void acq_set_adc(uint8_t n, uint8_t clk)
{
    if (n < 1U) { n = 1U; }
    if (n > 7U) { n = 7U; }
    g_adc_n = n;
    if (clk > 3U) { clk = 3U; }
    g_adc_clk = clk;
    /* Program the prescaler regardless of backend (the software path also uses
     * the ADC); hw_adc_clk_set only reconfigures the ADC when it is enabled. */
    hw_adc_clk_set(g_adc_clk);
}

void acq_set_recovery(uint8_t on)
{
    g_recovery = (on != 0U) ? 1U : 0U;
}

void acq_set_read_burst(uint16_t periods)
{
    if (periods == 0U) {
        g_read_burst = 0U;                  /* off */
        return;
    }
    if (periods < PRIME_MIN) { periods = PRIME_MIN; }
    if (periods > PRIME_MAX) { periods = PRIME_MAX; }
    g_read_burst = periods;
}

void acq_set_prime(uint8_t periods, uint8_t repeats)
{
    if (periods != 0U) {
        g_prime_burst = periods;
    }
    if (repeats != 0U) {
        g_prime_repeats = (repeats > 32U) ? 32U : repeats;
    }
}

uint8_t acq_get_freq(void)     { return g_freq; }
uint8_t acq_get_burst(void)
{
    return (g_backend == ACQ_BACKEND_HW) ? g_burst : (uint8_t)SW_BURST_PERIODS;
}
uint8_t acq_get_adc_n(void)    { return g_adc_n; }
uint8_t acq_get_adc_clk(void)  { return g_adc_clk; }
uint8_t acq_get_recovery(void) { return g_recovery; }
uint8_t acq_get_prime_burst(void) { return g_prime_burst; }
