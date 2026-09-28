/*
 * board.h — HS611 (GD32F350R8T6) hardware map for the A/B acquisition firmware.
 *
 * Everything tablet-specific lives here so this project can be forked for
 * another tablet by editing this file and the coil tables in coil_tables.c.
 * See docs/porting.md.
 *
 * Drive/measure pin map (from the vendor AFE):
 *   PA5   TIMER1_CH0 (AF2)  carrier gate
 *   PB10  TIMER1_CH2 (AF2)  50% PWM carrier
 *   PA4   sample/hold pulse (low = sample, high = hold)
 *   PB[0..15] / PC[6..9]    receive-coil mux (see coil_tables.c)
 *   PA1   tablet button (also the DFU entry button)
 */

#ifndef MIN_BOARD_H
#define MIN_BOARD_H

#include <stdint.h>

/* ---- coil geometry ------------------------------------------------------ */
/* Axis A: 27 receive loops (Y on the HS611). Axis B: 41 loops (X). */
#define AXIS_A_N            27U
#define AXIS_B_N            41U

/* ---- peripheral bases --------------------------------------------------- */
#define GPIOA_BASE          0x48000000U
#define GPIOB_BASE          0x48000400U
#define GPIOC_BASE          0x48000800U
#define GPIOF_BASE          0x48001400U
#define ADC0_BASE           0x40012400U

/* ---- pin masks ---------------------------------------------------------- */
#define PIN_PA4             0x00000010U   /* sample/hold */
#define PIN_PA5             0x00000020U   /* carrier gate */
#define PIN_PB10            0x00000400U   /* carrier */

/* GPIOB pins shared by the mux (idle high) and the mux-inactive mask used by
 * the coil tables. */
#define MUX_B_IDLE_MASK     0x7387U
#define MUX_C_KEEP_MASK     0xFE3FU

/* ---- default acquisition settings --------------------------------------- */
#define ACQ_DEFAULT_FREQ        9U
#define ACQ_DEFAULT_BURST       29U
#define ACQ_DEFAULT_ADC_N       3U
#define ACQ_DEFAULT_ADC_CLK     2U        /* APB2/6 = 12 MHz */

/* Safe carrier band (TIMER1 ARR in 72 MHz counts). */
#define CARRIER_ARR_MIN     100U
#define CARRIER_ARR_MAX     400U

/* Read burst band and the wider prime/steady-state band. */
#define BURST_MIN           6U
#define BURST_MAX           32U
#define PRIME_MIN           6U
#define PRIME_MAX           255U

/* Software NOP-sled carrier periods (compile-time; the A/B panel's burst
 * control is hardware-only). Build with -DSW_BURST_PERIODS=<n> or make SW_BURST=<n>. */
#ifndef SW_BURST_PERIODS
#define SW_BURST_PERIODS 29U
#endif
#if (SW_BURST_PERIODS < 6U) || (SW_BURST_PERIODS > 32U)
#error "SW_BURST_PERIODS must be 6..32 (matches BURST_MIN..BURST_MAX)"
#endif

#endif /* MIN_BOARD_H */
