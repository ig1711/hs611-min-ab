/*
 * dwt.h — Cortex-M4 DWT cycle counter: microsecond delays and timestamps.
 *
 * The A/B firmware uses DWT for every software delay and for the frame
 * timestamps, so no timer is spent on busy-waiting (TIMER2 is the ADC
 * one-shot trigger and TIMER1 is the carrier).
 */

#ifndef MIN_DWT_H
#define MIN_DWT_H

#include <stdint.h>
#include "gd32f3x0.h"

#define DWT_CYCLES_PER_US   72U   /* 72 MHz core clock */

static inline void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void dwt_delay_cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles) {
    }
}

static inline void dwt_delay_us(uint32_t us)
{
    dwt_delay_cycles(us * DWT_CYCLES_PER_US);
}

static inline uint32_t dwt_now_us(void)
{
    return DWT->CYCCNT / DWT_CYCLES_PER_US;
}

#endif /* MIN_DWT_H */
