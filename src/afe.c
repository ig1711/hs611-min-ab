/*
 * afe.c — analog front end (Path B: faithful vendor port).
 *
 * RCU/clock init, AFE GPIO init and the raw-register ADC are ported from the
 * vendor application:
 *   RCU/clock init   FUN_08009fb8 (+ FUN_0800a000 / FUN_08009c28 / FUN_08009adc)
 *   AFE GPIO init    FUN_08005274
 *   ADC single read  FUN_08007016 (config) / FUN_080071ea (read)
 *   ADC full read    FUN_08004f74 (settle + 3 conversions + median)
 *   channel select   FUN_0800da30
 *   excitation       FUN_08009aac / FUN_080099e0 / 0x0800a360..b214
 *
 * IMPORTANT ADC detail: on the GD32F3x0 the regular sequence rank 0 lives in
 * RSQ2 (0x34), not RSQ0.
 */

#include "afe.h"
#include "board.h"
#include "dwt.h"

#include "gd32f3x0.h"

/* ---- ADC0 registers ----------------------------------------------------- */
#define ADC0_STAT       REG32(ADC0_BASE + 0x00U)
#define ADC0_CTL0       REG32(ADC0_BASE + 0x04U)
#define ADC0_CTL1       REG32(ADC0_BASE + 0x08U)
#define ADC0_SAMPT0     REG32(ADC0_BASE + 0x0CU)
#define ADC0_SAMPT1     REG32(ADC0_BASE + 0x10U)
#define ADC0_RSQ0       REG32(ADC0_BASE + 0x2CU)
#define ADC0_RSQ1       REG32(ADC0_BASE + 0x30U)
#define ADC0_RSQ2       REG32(ADC0_BASE + 0x34U)
#define ADC0_RDATA      REG32(ADC0_BASE + 0x4CU)

/* Settle times are supplied by acq.c as DWT cycles (72 = 1 us); the host can
 * tune them sub-microsecond. The pre-amp settle (C) is the important one: too
 * long and adjacent-coil differences collapse, too short and the sample is
 * still slewing. */

/* ---- low level GPIO ----------------------------------------------------- */
static inline void gpio_mode(uint32_t base, uint32_t moder, uint32_t pupd, uint32_t mask)
{
    uint32_t m = REG32(base + 0x00U);
    uint32_t p = REG32(base + 0x0CU);
    for (uint32_t i = 0U; i < 16U; i++) {
        if ((1UL << i) & mask) {
            m = (m & ~(3UL << (i * 2U))) | (moder << (i * 2U));
            p = (p & ~(3UL << (i * 2U))) | (pupd << (i * 2U));
        }
    }
    REG32(base + 0x00U) = m;
    REG32(base + 0x0CU) = p;
}

static inline void gpio_otype_speed(uint32_t base, uint32_t otype, uint32_t ospeed, uint32_t mask)
{
    uint32_t o = REG32(base + 0x04U);
    uint32_t s = REG32(base + 0x08U);
    for (uint32_t i = 0U; i < 16U; i++) {
        if ((1UL << i) & mask) {
            o = (o & ~(1UL << i)) | (otype << i);
            s = (s & ~(3UL << (i * 2U))) | (ospeed << (i * 2U));
        }
    }
    REG32(base + 0x04U) = o;
    REG32(base + 0x08U) = s;
}

static inline void gpio_af(uint32_t base, uint32_t af, uint32_t mask)
{
    uint32_t lo = REG32(base + 0x20U);
    uint32_t hi = REG32(base + 0x24U);
    for (uint32_t i = 0U; i < 8U; i++) {
        if ((1UL << i) & mask) {
            lo = (lo & ~(0xFUL << (i * 4U))) | (af << (i * 4U));
        }
    }
    for (uint32_t i = 8U; i < 16U; i++) {
        if ((1UL << i) & mask) {
            hi = (hi & ~(0xFUL << ((i - 8U) * 4U))) | (af << ((i - 8U) * 4U));
        }
    }
    REG32(base + 0x20U) = lo;
    REG32(base + 0x24U) = hi;
}

static inline void gpio_brr(uint32_t base, uint32_t mask)  { REG32(base + 0x28U) = mask; }
static inline void gpio_bsrr(uint32_t base, uint32_t mask) { REG32(base + 0x18U) = mask; }

/* ---- RCU / clock init (vendor FUN_08009fb8) ----------------------------- */
static void rcu_enable(uint32_t id)
{
    REG32(RCU + (id >> 6)) |= (1U << (id & 0x1FU));
}

static void rcu_init_vendor(void)
{
    rcu_enable(0x71CU);                    /* APB1EN bit28: PMU */
    RCU_CFG0 = RCU_CFG0 & 0xFFFFC7FFU;     /* APB2 prescaler = /1 */
    rcu_enable(0x511U);                    /* AHBEN bit17: GPIOA */
    rcu_enable(0x512U);                    /* AHBEN bit18: GPIOB */
    rcu_enable(0x513U);                    /* AHBEN bit19: GPIOC */
    rcu_enable(0x516U);                    /* AHBEN bit22: GPIOF */
    rcu_enable(0x609U);                    /* APB2EN bit9: ADC */

    /* ADC clock = APB2/6, source AHB/APB2 divider */
    RCU_CFG0 &= 0xFFFF3FFFU;               /* clear ADCPSC (bits 14-15) */
    RCU_CFG2 &= 0x7FFEFEFFU;               /* clear ADCSEL/IRC28MDIV */
    RCU_CFG0 |= 0x00008000U;               /* ADCPSC = 2 -> APB2/6 */
    RCU_CFG2 |= 0x00000100U;               /* ADCSEL = AHB/APB2 divider */

    rcu_enable(0x716U);                    /* APB1EN bit22: I2C1 */
}

/* ---- AFE GPIO init (vendor FUN_08005274) -------------------------------- */
static void afe_gpio_init(void)
{
    /* PC3 output high; PA8 output high; ~1 s AFE power-up delay */
    gpio_mode(GPIOC_BASE, 1U, 0U, 0x8U);
    gpio_otype_speed(GPIOC_BASE, 0U, 0U, 0x8U);
    gpio_bsrr(GPIOC_BASE, 0x8U);

    gpio_mode(GPIOA_BASE, 1U, 0U, 0x100U);
    gpio_otype_speed(GPIOA_BASE, 0U, 0U, 0x100U);
    gpio_bsrr(GPIOA_BASE, 0x100U);

    for (uint8_t i = 0U; i < 10U; i++) {
        dwt_delay_us(100000U);
    }

    gpio_brr(GPIOA_BASE, 0x100U);          /* PA8 low */

    gpio_mode(GPIOA_BASE, 3U, 0U, 0x40U);  /* PA6 analog */

    gpio_mode(GPIOA_BASE, 1U, 0U, 0x8U);   /* PA3 output low */
    gpio_otype_speed(GPIOA_BASE, 0U, 1U, 0x8U);
    gpio_brr(GPIOA_BASE, 0x8U);
    gpio_brr(GPIOA_BASE, 0x8U);

    gpio_mode(GPIOB_BASE, 1U, 0U, PIN_PB10);   /* PB10 output low (drive enable) */
    gpio_otype_speed(GPIOB_BASE, 0U, 3U, PIN_PB10);
    gpio_brr(GPIOB_BASE, PIN_PB10);

    gpio_mode(GPIOA_BASE, 1U, 0U, 0x30U);  /* PA4/PA5 outputs; PA4 high, PA5 low */
    gpio_otype_speed(GPIOA_BASE, 0U, 0U, 0x30U);
    gpio_bsrr(GPIOA_BASE, PIN_PA4);
    gpio_brr(GPIOA_BASE, PIN_PA5);

    gpio_mode(GPIOC_BASE, 1U, 0U, 0x1C0U); /* PC6/7/8 outputs low */
    gpio_otype_speed(GPIOC_BASE, 0U, 0U, 0x1C0U);
    gpio_brr(GPIOC_BASE, 0x40U);
    gpio_brr(GPIOC_BASE, 0x80U);
    gpio_brr(GPIOC_BASE, 0x100U);

    gpio_mode(GPIOB_BASE, 1U, 0U, MUX_B_IDLE_MASK);  /* mux lines, ODR |= idle */
    gpio_otype_speed(GPIOB_BASE, 0U, 0U, MUX_B_IDLE_MASK);
    REG32(GPIOB_BASE + 0x14U) |= MUX_B_IDLE_MASK;

    gpio_mode(GPIOA_BASE, 2U, 0U, 0x400U); /* PA10 AF2 */
    gpio_otype_speed(GPIOA_BASE, 0U, 0U, 0x400U);
    gpio_af(GPIOA_BASE, 2U, 0x400U);

    gpio_mode(GPIOA_BASE, 3U, 0U, 0x6U);   /* PA1/PA2 analog */

    gpio_mode(GPIOC_BASE, 0U, 0U, 0x1U);   /* PC0 input (pen presence) */

    gpio_mode(GPIOF_BASE, 1U, 1U, 0xC0U);  /* PF6/PF7 output open-drain */
    gpio_otype_speed(GPIOF_BASE, 1U, 0U, 0xC0U);

    gpio_mode(GPIOC_BASE, 1U, 3U, 0x400U); /* PC10 output pull-up, pulse */
    gpio_otype_speed(GPIOC_BASE, 0U, 3U, 0x400U);
    gpio_bsrr(GPIOC_BASE, 0x400U);
    gpio_brr(GPIOC_BASE, 0x400U);
}

/* ---- raw-register ADC --------------------------------------------------- */
/* FUN_08007224: sequence entry is RSQ2 for seq 0-5, RSQ1 for 6-11, RSQ0 12-15 */
static void adc_seq_set(uint32_t seq, uint32_t ch)
{
    if (seq < 6U) {
        ADC0_RSQ2 = (ADC0_RSQ2 & ~(0x1FU << (seq * 5U))) | (ch << (seq * 5U));
    } else if (seq < 12U) {
        uint32_t s = seq - 6U;
        ADC0_RSQ1 = (ADC0_RSQ1 & ~(0x1FU << (s * 5U))) | (ch << (s * 5U));
    } else {
        uint32_t s = seq - 12U;
        ADC0_RSQ0 = (ADC0_RSQ0 & ~(0x1FU << (s * 5U))) | (ch << (s * 5U));
    }
}

/* FUN_08007224: sample time is SAMPT1 for ch 0-9, SAMPT0 for ch 10-18 */
static void adc_sampt_set(uint32_t ch, uint32_t st)
{
    if (ch < 10U) {
        ADC0_SAMPT1 = (ADC0_SAMPT1 & ~(7U << (ch * 3U))) | (st << (ch * 3U));
    } else if (ch < 19U) {
        uint32_t s = ch - 10U;
        ADC0_SAMPT0 = (ADC0_SAMPT0 & ~(7U << (s * 3U))) | (st << (s * 3U));
    }
}

/* FUN_08007016: configure one regular conversion on `ch` */
static void adc_config(uint32_t ch, uint32_t st)
{
    ADC0_RSQ0 &= ~(0xFU << 20);                          /* RL = 0 -> 1 conversion */
    adc_seq_set(0U, ch);
    adc_sampt_set(ch, st);
    ADC0_CTL1 |= (1U << 20);                             /* ETERC */
    ADC0_CTL1 = (ADC0_CTL1 & ~(7U << 17)) | (7U << 17);  /* ETSRC = SWRCST */
    ADC0_CTL1 &= ~(1U << 11);                            /* DAL = right */
    ADC0_CTL1 |= 1U;                                     /* ADCON */
    ADC0_CTL0 |= (1U << 8);                              /* SM (scan mode) */
    ADC0_CTL1 |= (1U << 22);                             /* SWRCST */
}

/* FUN_080071ea: software-trigger one conversion, wait EOC, return RDATA.
 * Bounded wait: on timeout return 0xFFFF so the main loop keeps running and the
 * failure is visible instead of hanging. */
static uint16_t adc_read(uint32_t ch, uint32_t st)
{
    uint32_t guard = 100000U;

    ADC0_RSQ0 &= ~(0xFU << 20);
    adc_seq_set(0U, ch);
    adc_sampt_set(ch, st);
    ADC0_CTL1 |= (1U << 22);                             /* SWRCST */
    while (((ADC0_STAT & 2U) == 0U) && (guard != 0U)) {
        guard--;
    }
    if (guard == 0U) {
        return 0xFFFFU;
    }
    ADC0_STAT &= ~2U;
    return (uint16_t)(ADC0_RDATA & 0xFFFFU);
}

static uint16_t median3(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) { uint16_t t = a; a = b; b = t; }
    return b;
}

/* FUN_08004f74: settle, pulse PA4, 3 conversions, median.
 * Settles are DWT cycles (72 = 1 us) so the host can tune them sub-microsecond.
 * mode 2 adds settle B, mode 1 adds settle D (coil recovery). */
static uint16_t adc_median6(uint8_t mode, const uint16_t settle_cyc[4])
{
    uint16_t v0, v1, v2, med;

    adc_config(6U, 5U);
    if (settle_cyc[0] != 0U) {
        dwt_delay_cycles(settle_cyc[0]);
    }
    if (mode == 2U && settle_cyc[1] != 0U) {
        dwt_delay_cycles(settle_cyc[1]);
    }
    gpio_brr(GPIOA_BASE, PIN_PA4);                       /* PA4 low: sample */
    if (settle_cyc[2] != 0U) {
        dwt_delay_cycles(settle_cyc[2]);                 /* pre-amp settle */
    }
    v0 = adc_read(6U, 5U);
    v1 = adc_read(6U, 5U);
    v2 = adc_read(6U, 5U);
    med = median3(v0, v1, v2);
    gpio_bsrr(GPIOA_BASE, PIN_PA4);                      /* PA4 high: hold */
    REG32(GPIOB_BASE + 0x14U) |= MUX_B_IDLE_MASK;        /* restore mux idle */
    if (mode == 1U && settle_cyc[3] != 0U) {
        dwt_delay_cycles(settle_cyc[3]);                 /* coil recovery */
    }
    return med;
}

/* ---- channel select (vendor FUN_0800da30) ------------------------------- */
void afe_mux(uint16_t mask_b, uint16_t val_c)
{
    REG32(GPIOB_BASE + 0x14U) &= mask_b;
    REG32(GPIOC_BASE + 0x14U) = (REG32(GPIOC_BASE + 0x14U) & MUX_C_KEEP_MASK) | val_c;
}

/* ---- excitation (vendor FUN_08009aac / DRIVE_BURST) --------------------- */
#define GPIOB_BRR_ADDR   0x48000428U
#define GPIOB_BSRR_ADDR  0x48000418U

/* 29 carrier periods, each a low/high half with N1/N2 NOPs. The burst is inline
 * asm with .rept so every sled entry is a real 1-cycle NOP. The period count is
 * compile-time (SW_BURST_PERIODS, board.h); the A/B panel's burst control is
 * hardware-only. */
#define SW_BURST_STR_(x) #x
#define SW_BURST_STR(x)  SW_BURST_STR_(x)
#define DRIVE_BURST(n1, n2)                                                   \
    do {                                                                      \
        __asm__ volatile(                                                     \
            "movs r4, #" SW_BURST_STR(SW_BURST_PERIODS) "\n"                  \
            "1:\n\t"                                                          \
            "mov.w r0, #0x400\n\t"                                            \
            "str r0, [%0]\n\t"                                                \
            ".rept " #n1 "\n\t"                                               \
            "nop\n\t"                                                         \
            ".endr\n\t"                                                       \
            "mov.w r0, #0x400\n\t"                                            \
            "str r0, [%1]\n\t"                                                \
            ".rept " #n2 "\n\t"                                               \
            "nop\n\t"                                                         \
            ".endr\n\t"                                                       \
            "subs r4, #1\n\t"                                                 \
            "bne 1b\n\t"                                                      \
            :                                                                 \
            : "r"(GPIOB_BRR_ADDR), "r"(GPIOB_BSRR_ADDR)                       \
            : "r0", "r4", "cc", "memory");                                    \
    } while (0)

static void drive_enable(void)
{
    gpio_mode(GPIOB_BASE, 1U, 0U, PIN_PB10);
    gpio_otype_speed(GPIOB_BASE, 0U, 3U, PIN_PB10);
    gpio_brr(GPIOB_BASE, PIN_PB10);
}

static void drive_disable(void)
{
    gpio_mode(GPIOB_BASE, 0U, 0U, PIN_PB10);             /* PB10 back to input */
}

static void excite(uint8_t freq)
{
    drive_enable();
    gpio_bsrr(GPIOA_BASE, PIN_PA5);                      /* gate on */

    switch (freq) {
    case 1:  DRIVE_BURST(73, 71); break;
    case 2:  DRIVE_BURST(72, 69); break;
    case 3:  DRIVE_BURST(70, 68); break;
    case 4:  DRIVE_BURST(69, 66); break;
    case 5:  DRIVE_BURST(67, 65); break;
    case 6:  DRIVE_BURST(66, 63); break;
    case 7:  DRIVE_BURST(65, 63); break;
    case 8:  DRIVE_BURST(64, 62); break;
    case 9:  DRIVE_BURST(63, 61); break;
    case 10: DRIVE_BURST(62, 60); break;
    case 11: DRIVE_BURST(61, 59); break;
    case 12: DRIVE_BURST(60, 57); break;
    default: break;                                      /* 0 = no drive */
    }

    gpio_bsrr(GPIOB_BASE, PIN_PB10);
    drive_disable();
    gpio_brr(GPIOA_BASE, PIN_PA5);                       /* gate off */
}

/* ---- public API --------------------------------------------------------- */
void afe_init(void)
{
    dwt_init();
    rcu_init_vendor();
    afe_gpio_init();
}

uint16_t afe_measure_sw(uint16_t mask_b, uint16_t val_c, uint8_t freq,
                        const uint16_t settle_cyc[4])
{
    uint16_t v;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();                                     /* keep USB ISRs out of the burst */

    afe_mux(mask_b, val_c);
    excite(freq);
    v = adc_median6(0U, settle_cyc);

    if (primask == 0U) {
        __enable_irq();
    }
    return v;
}
