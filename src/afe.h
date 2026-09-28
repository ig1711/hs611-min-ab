/*
 * afe.h — analog front end: AFE bring-up, coil mux, software measurement.
 *
 * The software backend is the baseline for A/B comparison against the
 * hardware-timed backend in hw_timer.c. It bit-bangs the drive burst and reads
 * the ADC with a bounded software-triggered sequence (the vendor "Path B").
 */

#ifndef MIN_AFE_H
#define MIN_AFE_H

#include <stdint.h>

/* RCU + AFE GPIO init and the DWT cycle counter. Call once at boot. */
void afe_init(void);

/* Select the receive loop described by a coil-table entry. */
void afe_mux(uint16_t mask_b, uint16_t val_c);

/* One complete measurement on the software backend. settle_cyc[4] is A/B/C/D in
 * DWT cycles (72 = 1 us). */
uint16_t afe_measure_sw(uint16_t mask_b, uint16_t val_c, uint8_t freq,
                        const uint16_t settle_cyc[4]);

#endif /* MIN_AFE_H */
