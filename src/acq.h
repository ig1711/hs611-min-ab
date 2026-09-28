/*
 * acq.h — runtime acquisition configuration and backend dispatch.
 *
 * Chooses between the software AFE backend (afe.c) and the hardware-timed
 * backend (hw_timer.c), holds every timing knob the host can change, and offers
 * the scan primitives the scan engine uses.
 */

#ifndef MIN_ACQ_H
#define MIN_ACQ_H

#include <stdint.h>

#define ACQ_BACKEND_SW   0U
#define ACQ_BACKEND_HW   1U

/* Initialize the backends and apply the default configuration. */
void acq_init(void);

/* One channel measurement using the current backend/configuration. */
uint16_t acq_measure(uint16_t mask_b, uint16_t val_c);

/* Scan a coil table into out[1..n] (out[0] unused). `desc` selects descending
 * order. */
void acq_scan_table(uint8_t n, const uint8_t *table, uint16_t *out, uint8_t desc);

/* Explicit prime on one 1-based coil of `table` (ramp mode 2). */
void acq_prime(const uint8_t *table, uint8_t idx);

/* Keep the drive carrier running across a non-measurement gap. */
void acq_keepalive(uint16_t periods);
void acq_keepalive_stop(void);

/* ---- runtime settings --------------------------------------------------- */
void acq_set_backend(uint8_t backend);            /* ACQ_BACKEND_* */
uint8_t acq_get_backend(void);

void acq_set_freq(uint8_t freq);                  /* 1..12, clears raw override */
void acq_set_freq_arr(uint16_t arr);              /* 0 = index table */
void acq_set_burst(uint8_t periods);              /* 6..32 */
void acq_set_settle(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
void acq_set_adc(uint8_t n, uint8_t clk);         /* n 1..7, clk 0..3 */
void acq_set_recovery(uint8_t on);                /* 0/1 */

/* Per-read burst override used by the "per-read steady state" ramp probe:
 * non-zero replaces the configured read burst. */
void acq_set_read_burst(uint16_t periods);
void acq_set_prime(uint8_t periods, uint8_t repeats);

uint8_t  acq_get_freq(void);
uint8_t  acq_get_burst(void);
uint8_t  acq_get_adc_n(void);
uint8_t  acq_get_adc_clk(void);
uint8_t  acq_get_recovery(void);
uint8_t  acq_get_prime_burst(void);

#endif /* MIN_ACQ_H */
