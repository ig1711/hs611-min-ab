/*
 * hw_timer.h — hardware-timed acquisition backend.
 *
 * Carrier: TIMER1 CH2 (PB10, AF2) PWM at the drive period, gated by TIMER1 CH0
 * (PA5, AF2) held active for the burst. The finite N-period burst is ended by
 * TIMER1_UP DMA (channel 1) writing TIMER1_CHCTL2 (CCER) to clear the channel
 * enables on the Nth update, so the carrier is exact and no critical section is
 * needed. NOTE: GD32 packs all four channels in CCER (CH0EN = bit0, CH2EN =
 * bit8).
 *
 * ADC: ADC0 configured once (regular ch6 x N, scan mode, external trigger
 * TIMER2_TRGO) and moved by DMA channel 0 into a buffer, then median-filtered
 * in software. TIMER2 runs in single-pulse mode purely as the one-shot trigger.
 *
 * Safety: the carrier period (ARR) is clamped, CH2 duty is fixed at 50%, the
 * burst length is capped, the drive pins are returned to a safe idle state after
 * every measurement, and the timer is force-disabled after a bounded wait even
 * if the DMA did not complete.
 */

#ifndef MIN_HW_TIMER_H
#define MIN_HW_TIMER_H

#include <stdint.h>

/* Enable DMA/TIMER1/TIMER2 clocks and configure the one-shot trigger. The
 * backend is not enabled until hw_measure()/hw_enable() is called. */
void hw_timer_init(void);

/* Carrier period: use the SET_FREQ index table unless arr_override is non-zero,
 * in which case it is used directly (clamped to the safe band). */
void hw_carrier_set(uint8_t freq, uint16_t arr_override);

/* ADC clock prescaler: 0 = APB2/2 (36 MHz), 1 = /4, 2 = /6 (12 MHz), 3 = /8. */
void hw_adc_clk_set(uint8_t sel);

/* Select the hardware backend (reconfigure carrier+ADC) or park it safely. */
void hw_enable(void);
void hw_disable(void);

/* One channel measurement using the hardware backend. `burst` is the read burst
 * in carrier periods; `read_burst` overrides it when non-zero (per-read
 * steady-state ramp probe). settle[] is A/B/C/D in microseconds. */
uint16_t hw_measure(uint16_t mask_b, uint16_t val_c, uint8_t burst, uint8_t adc_n,
                    const uint8_t settle[4], uint8_t recovery,
                    uint16_t read_burst);

/* Explicit prime on one 1-based coil: `repeats` back-to-back bursts of
 * `periods` periods, no ADC (prime-per-window ramp probe). */
void hw_prime(uint16_t mask_b, uint16_t val_c, uint8_t periods, uint8_t repeats);

/* Keep the carrier running across a non-measurement gap (frame build) so the
 * pen tank does not decay; stop before the next measurement. */
void hw_keepalive(uint16_t periods);
void hw_keepalive_stop(void);

#endif /* MIN_HW_TIMER_H */
