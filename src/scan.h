/*
 * scan.h — acquisition scan engine and runtime A/B settings.
 *
 * Owns everything the host can change at run time except the low-level
 * acquisition timing (which lives in acq.c): the tracking window, re-centre
 * strategy, scan order, warm-up, ramp mitigation, repeat-coil diagnostic,
 * re-acquire policy and pacing. scan_step() performs one acquisition and packs
 * a complete protocol frame; see protocol.h.
 */

#ifndef MIN_SCAN_H
#define MIN_SCAN_H

#include <stdint.h>

/* Reset all runtime settings to their defaults and prime the ramp/read-burst
 * coupling. Call after acq_init(). */
void scan_init(void);

/* Run one acquisition and write a complete protocol frame into `frame`. */
void scan_step(uint8_t *frame);

/* ---- runtime settings (host commands) ----------------------------------- */
void scan_set_window(uint8_t x_coils, uint8_t y_coils);      /* 0 = full grid */
void scan_set_recenter(uint8_t mode, uint8_t hyst, uint8_t persist,
                       uint8_t step, uint8_t deadband);
void scan_set_scan_order(uint8_t order);                     /* 0..3 */
void scan_set_warmup(uint8_t reads);                         /* 0..8 */
void scan_set_flat_tol(uint16_t tol);
void scan_set_ramp(uint8_t mode, uint8_t prime_burst, uint8_t prime_repeats,
                   int16_t slope_per_mille, uint8_t rev_radius);
void scan_set_reacq(uint16_t threshold, uint8_t coarse_stride, uint16_t period_ms);
void scan_set_repeat_coil(uint8_t coil);                     /* 0 = off, 1..41 */
void scan_set_pacing(uint8_t pacing);                        /* PROTOCOL_PACING_* */

/* Effective free-run flag (used by the keep-alive carrier). */
uint8_t scan_is_free_run(void);

#endif /* MIN_SCAN_H */
