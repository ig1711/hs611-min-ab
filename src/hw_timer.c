/*
 * hw_timer.c — hardware-timed acquisition backend. See hw_timer.h.
 */

#include "hw_timer.h"
#include "board.h"
#include "dwt.h"
#include "afe.h"

#include "gd32f3x0.h"

#define ADC0_BASE           0x40012400U

/* TIMER1_CHCTL2 (CCER); the carrier-stop DMA writes here. */
#define TIMER1_CCER         (TIMER1 + 0x20U)
#define CCER_BOTH_EN        ((1U << 0) | (1U << 8))   /* CH0EN | CH2EN */

#define ADC_MAX             7U
#define PRIME_MAX_PERIODS   255U

/* Carrier period in 72 MHz counts per frequency index, from the vendor's
 * 29-period NOP sleds (afe.c DRIVE_BURST). */
static const uint16_t freq_arr_table[13] = {
    0U, 152U, 149U, 146U, 143U, 140U, 137U,
    136U, 134U, 132U, 130U, 128U, 125U
};

static uint8_t  g_freq = ACQ_DEFAULT_FREQ;
static uint16_t g_freq_override;              /* 0 = use the table */
static uint8_t  g_adc_n = ACQ_DEFAULT_ADC_N;
static uint8_t  g_adc_clk = ACQ_DEFAULT_ADC_CLK;
static uint8_t  g_enabled;

static uint32_t g_burst_buf[PRIME_MAX_PERIODS];
static volatile uint16_t g_adc_buf[ADC_MAX];

/* ---- GPIO helpers ------------------------------------------------------- */
static void gpio_pin(uint32_t base, uint32_t pin, uint32_t moder,
                     uint32_t pupd, uint32_t otype, uint32_t ospeed)
{
    uint32_t b = 1UL << pin;
    uint32_t m = REG32(base + 0x00U);
    uint32_t p = REG32(base + 0x0CU);
    uint32_t o = REG32(base + 0x04U);
    uint32_t s = REG32(base + 0x08U);

    m = (m & ~(3UL << (pin * 2U))) | (moder << (pin * 2U));
    p = (p & ~(3UL << (pin * 2U))) | (pupd << (pin * 2U));
    o = (o & ~b) | (otype << pin);
    s = (s & ~(3UL << (pin * 2U))) | (ospeed << (pin * 2U));

    REG32(base + 0x00U) = m;
    REG32(base + 0x0CU) = p;
    REG32(base + 0x04U) = o;
    REG32(base + 0x08U) = s;
}

static void gpio_af(uint32_t base, uint32_t pin, uint32_t af)
{
    if (pin < 8U) {
        uint32_t lo = REG32(base + 0x20U);
        lo = (lo & ~(0xFUL << (pin * 4U))) | (af << (pin * 4U));
        REG32(base + 0x20U) = lo;
    } else {
        uint32_t hi = REG32(base + 0x24U);
        hi = (hi & ~(0xFUL << ((pin - 8U) * 4U))) | (af << ((pin - 8U) * 4U));
        REG32(base + 0x24U) = hi;
    }
}

/* Safe idle: PA5 driven low (gate off), PB10 high-Z (input). */
static void drive_gpio_idle(void)
{
    gpio_pin(GPIOA_BASE, 5U, 1U, 0U, 0U, 0U);
    REG32(GPIOA_BASE + 0x28U) = PIN_PA5;              /* BRR: low */
    REG32(GPIOB_BASE + 0x28U) = PIN_PB10;             /* BRR: low */
    gpio_pin(GPIOB_BASE, 10U, 0U, 0U, 0U, 3U);        /* input */
}

/* Software-backend idle: PA5/PB10 plain outputs low (afe.c drives them). */
static void drive_gpio_sw_idle(void)
{
    gpio_pin(GPIOA_BASE, 5U, 1U, 0U, 0U, 0U);
    REG32(GPIOA_BASE + 0x28U) = PIN_PA5;              /* BRR: low */
    gpio_pin(GPIOB_BASE, 10U, 1U, 0U, 0U, 3U);
    REG32(GPIOB_BASE + 0x28U) = PIN_PB10;             /* BRR: low */
}

/* ---- carrier (TIMER1) --------------------------------------------------- */
static uint32_t clamp_arr(uint32_t arr)
{
    if (arr < CARRIER_ARR_MIN) { arr = CARRIER_ARR_MIN; }
    if (arr > CARRIER_ARR_MAX) { arr = CARRIER_ARR_MAX; }
    return arr;
}

static void carrier_config(void)
{
    uint32_t arr = clamp_arr((g_freq_override != 0U)
                                 ? (uint32_t)g_freq_override
                                 : (uint32_t)freq_arr_table[g_freq]);

    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);

    timer_parameter_struct t;
    timer_struct_para_init(&t);
    t.prescaler          = 0U;
    t.alignedmode        = TIMER_COUNTER_EDGE;
    t.counterdirection   = TIMER_COUNTER_UP;
    t.period             = arr;
    t.clockdivision      = TIMER_CKDIV_DIV1;
    t.repetitioncounter  = 0U;
    timer_init(TIMER1, &t);
    timer_auto_reload_shadow_enable(TIMER1);

    timer_oc_parameter_struct oc;
    timer_channel_output_struct_para_init(&oc);
    oc.outputstate  = TIMER_CCX_DISABLE;
    oc.outputnstate = TIMER_CCXN_DISABLE;
    oc.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    oc.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    oc.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    oc.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    /* CH2 = carrier, 50% duty. */
    timer_channel_output_config(TIMER1, TIMER_CH_2, &oc);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_2, TIMER_OC_MODE_PWM0);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_2, arr / 2U);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);

    /* CH0 = gate, held active while enabled. */
    timer_channel_output_config(TIMER1, TIMER_CH_0, &oc);
    timer_channel_output_mode_config(TIMER1, TIMER_CH_0, TIMER_OC_MODE_HIGH);
    timer_channel_output_pulse_value_config(TIMER1, TIMER_CH_0, 0U);
    timer_channel_output_shadow_config(TIMER1, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);

    timer_dma_enable(TIMER1, TIMER_DMA_UPD);
}

/* Arm DMA channel 1 to clear the carrier enables on update #N. */
static void carrier_dma_arm(uint16_t n)
{
    dma_parameter_struct d;

    for (uint16_t i = 0U; i < (uint16_t)(n - 1U); i++) {
        g_burst_buf[i] = CCER_BOTH_EN;
    }
    g_burst_buf[n - 1U] = 0U;

    dma_deinit(DMA_CH1);
    dma_struct_para_init(&d);
    d.periph_addr   = TIMER1_CCER;
    d.periph_width  = DMA_PERIPHERAL_WIDTH_32BIT;
    d.periph_inc    = DMA_PERIPH_INCREASE_DISABLE;
    d.memory_addr   = (uint32_t)g_burst_buf;
    d.memory_width  = DMA_MEMORY_WIDTH_32BIT;
    d.memory_inc    = DMA_MEMORY_INCREASE_ENABLE;
    d.direction     = DMA_MEMORY_TO_PERIPHERAL;
    d.number        = n;
    d.priority      = DMA_PRIORITY_ULTRA_HIGH;
    dma_init(DMA_CH1, &d);
    dma_flag_clear(DMA_CH1, DMA_FLAG_G);
    dma_channel_enable(DMA_CH1);
}

/* Start exactly `n` carrier periods; returns immediately (DMA ends the burst on
 * the Nth update). */
static void carrier_start(uint16_t n)
{
    if (n < BURST_MIN) { n = BURST_MIN; }
    if (n > PRIME_MAX_PERIODS) { n = PRIME_MAX_PERIODS; }

    timer_disable(TIMER1);
    timer_channel_output_state_config(TIMER1, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER1, TIMER_CH_2, TIMER_CCX_DISABLE);
    TIMER_CNT(TIMER1) = 0U;
    timer_interrupt_flag_clear(TIMER1, TIMER_INTF_UPIF);

    carrier_dma_arm(n);

    gpio_pin(GPIOA_BASE, 5U, 2U, 0U, 0U, 0U);          /* PA5 -> AF2 */
    gpio_af(GPIOA_BASE, 5U, 2U);
    gpio_pin(GPIOB_BASE, 10U, 2U, 0U, 0U, 3U);         /* PB10 -> AF2 */
    gpio_af(GPIOB_BASE, 10U, 2U);

    timer_channel_output_state_config(TIMER1, TIMER_CH_0, TIMER_CCX_ENABLE);
    timer_channel_output_state_config(TIMER1, TIMER_CH_2, TIMER_CCX_ENABLE);
    timer_enable(TIMER1);
}

/* Emit exactly `n` carrier periods, then stop the outputs. */
static void carrier_burst_n(uint16_t n)
{
    uint32_t guard;

    carrier_start(n);

    /* Bounded wait for the DMA to clear the enables (carrier stop). If it does
     * not finish, the caller's stop still disables the timer. */
    guard = 200000U;
    while ((0U != dma_transfer_number_get(DMA_CH1)) && (0U != guard)) {
        guard--;
    }
}

static void carrier_stop(void)
{
    timer_disable(TIMER1);
    timer_channel_output_state_config(TIMER1, TIMER_CH_0, TIMER_CCX_DISABLE);
    timer_channel_output_state_config(TIMER1, TIMER_CH_2, TIMER_CCX_DISABLE);
    dma_channel_disable(DMA_CH1);
    drive_gpio_idle();
}

/* ---- ADC (ADC0 + DMA channel 0 + TIMER2 one-shot trigger) --------------- */
static void adc_seq_set(uint32_t seq, uint32_t ch)
{
    if (seq < 6U) {
        REG32(ADC0_BASE + 0x34U) = (REG32(ADC0_BASE + 0x34U) & ~(0x1FU << (seq * 5U))) | (ch << (seq * 5U));
    } else if (seq < 12U) {
        uint32_t s = seq - 6U;
        REG32(ADC0_BASE + 0x30U) = (REG32(ADC0_BASE + 0x30U) & ~(0x1FU << (s * 5U))) | (ch << (s * 5U));
    } else {
        uint32_t s = seq - 12U;
        REG32(ADC0_BASE + 0x2CU) = (REG32(ADC0_BASE + 0x2CU) & ~(0x1FU << (s * 5U))) | (ch << (s * 5U));
    }
}

static void adc_sampt_set(uint32_t ch, uint32_t st)
{
    if (ch < 10U) {
        REG32(ADC0_BASE + 0x10U) = (REG32(ADC0_BASE + 0x10U) & ~(7U << (ch * 3U))) | (st << (ch * 3U));
    } else if (ch < 19U) {
        uint32_t s = ch - 10U;
        REG32(ADC0_BASE + 0x0CU) = (REG32(ADC0_BASE + 0x0CU) & ~(7U << (s * 3U))) | (st << (s * 3U));
    }
}

static void adc_config(void)
{
    uint8_t n = g_adc_n;

    if (n < 1U) { n = 1U; }
    if (n > ADC_MAX) { n = ADC_MAX; }
    g_adc_n = n;

    REG32(ADC0_BASE + 0x2CU) = (REG32(ADC0_BASE + 0x2CU) & ~(0xFU << 20)) | ((uint32_t)(n - 1U) << 20);
    for (uint8_t i = 0U; i < n; i++) {
        adc_seq_set(i, 6U);
        adc_sampt_set(6U, 5U);
    }

    REG32(ADC0_BASE + 0x04U) |= (1U << 8);             /* scan mode */
    REG32(ADC0_BASE + 0x08U) &= ~(1U << 11);           /* right aligned */
    REG32(ADC0_BASE + 0x08U) |= (1U << 8);             /* DMA */
    REG32(ADC0_BASE + 0x08U) |= (1U << 20);            /* ETERC */
    REG32(ADC0_BASE + 0x08U) = (REG32(ADC0_BASE + 0x08U) & ~(7U << 17)) | (4U << 17);  /* T2_TRGO */
    REG32(ADC0_BASE + 0x08U) |= 1U;                    /* ADCON */

    dwt_delay_us(10U);
}

static void adc_dma_arm(void)
{
    dma_parameter_struct d;

    dma_deinit(DMA_CH0);
    dma_struct_para_init(&d);
    d.periph_addr   = (uint32_t)(ADC0_BASE + 0x4CU);   /* RDATA */
    d.periph_width  = DMA_PERIPHERAL_WIDTH_16BIT;
    d.periph_inc    = DMA_PERIPH_INCREASE_DISABLE;
    d.memory_addr   = (uint32_t)g_adc_buf;
    d.memory_width  = DMA_MEMORY_WIDTH_16BIT;
    d.memory_inc    = DMA_MEMORY_INCREASE_ENABLE;
    d.direction     = DMA_PERIPHERAL_TO_MEMORY;
    d.number        = g_adc_n;
    d.priority      = DMA_PRIORITY_HIGH;
    dma_init(DMA_CH0, &d);
    dma_flag_clear(DMA_CH0, DMA_FLAG_G);
    dma_channel_enable(DMA_CH0);
}

static void trigger_config(void)
{
    timer_parameter_struct t;

    rcu_periph_clock_enable(RCU_TIMER2);
    timer_deinit(TIMER2);

    timer_struct_para_init(&t);
    t.prescaler         = 71U;                         /* 1 us tick at 72 MHz */
    t.alignedmode       = TIMER_COUNTER_EDGE;
    t.counterdirection  = TIMER_COUNTER_UP;
    t.period            = 2U;                          /* fire ~2 us after start */
    t.clockdivision     = TIMER_CKDIV_DIV1;
    t.repetitioncounter = 0U;
    timer_init(TIMER2, &t);
    timer_single_pulse_mode_config(TIMER2, TIMER_SP_MODE_SINGLE);
    timer_master_output_trigger_source_select(TIMER2, TIMER_TRI_OUT_SRC_UPDATE);
    timer_disable(TIMER2);
}

/* One-shot: TIMER2 update triggers the ADC, DMA captures g_adc_n samples. */
static void adc_sample(void)
{
    uint32_t guard;

    adc_dma_arm();

    timer_disable(TIMER2);
    TIMER_CNT(TIMER2) = 0U;
    timer_interrupt_flag_clear(TIMER2, TIMER_INTF_UPIF);
    timer_enable(TIMER2);

    guard = 200000U;
    while ((0U != dma_transfer_number_get(DMA_CH0)) && (0U != guard)) {
        guard--;
    }
    dma_channel_disable(DMA_CH0);
    timer_disable(TIMER2);
}

static uint16_t adc_median(void)
{
    uint16_t b[ADC_MAX];
    uint8_t n = g_adc_n;

    for (uint8_t i = 0U; i < n; i++) {
        b[i] = g_adc_buf[i];
    }
    for (uint8_t i = 1U; i < n; i++) {                 /* insertion sort */
        uint16_t v = b[i];
        int8_t j = (int8_t)(i - 1U);
        while ((j >= 0) && (b[j] > v)) {
            b[j + 1] = b[j];
            j--;
        }
        b[j + 1] = v;
    }
    if ((n & 1U) != 0U) {
        return b[n / 2U];
    }
    return (uint16_t)(((uint32_t)b[n / 2U - 1U] + (uint32_t)b[n / 2U]) / 2U);
}

/* ---- public API --------------------------------------------------------- */
void hw_timer_init(void)
{
    rcu_periph_clock_enable(RCU_DMA);
    rcu_periph_clock_enable(RCU_TIMER1);
    rcu_periph_clock_enable(RCU_TIMER2);
    trigger_config();
    hw_adc_clk_set(g_adc_clk);
}

void hw_carrier_set(uint8_t freq, uint16_t arr_override)
{
    g_freq = freq;
    g_freq_override = (arr_override == 0U) ? 0U : (uint16_t)clamp_arr(arr_override);
    if (g_enabled) {
        carrier_config();
    }
}

void hw_adc_clk_set(uint8_t sel)
{
    if (sel > 3U) { sel = 3U; }
    g_adc_clk = sel;
    RCU_CFG0 = (RCU_CFG0 & ~(3UL << 14)) | ((uint32_t)sel << 14);
    dwt_delay_us(10U);
    if (g_enabled) {
        adc_config();
    }
}

void hw_enable(void)
{
    carrier_stop();
    timer_disable(TIMER2);
    REG32(ADC0_BASE + 0x08U) &= ~(1U << 8);            /* DMA off */

    carrier_config();
    adc_config();
    drive_gpio_idle();
    g_enabled = 1U;
}

void hw_disable(void)
{
    carrier_stop();
    timer_disable(TIMER2);
    REG32(ADC0_BASE + 0x08U) &= ~(1U << 8);            /* DMA off */
    REG32(ADC0_BASE + 0x08U) = (REG32(ADC0_BASE + 0x08U) & ~(7U << 17)) | (7U << 17);  /* SW trigger */
    drive_gpio_sw_idle();                              /* no floating PB10 on the SW path */
    g_enabled = 0U;
}

uint16_t hw_measure(uint16_t mask_b, uint16_t val_c, uint8_t burst, uint8_t adc_n,
                    const uint16_t settle_cyc[4], uint8_t recovery, uint16_t read_burst)
{
    uint16_t v;
    uint16_t n;
    uint32_t primask;

    if (adc_n != g_adc_n) {
        g_adc_n = (adc_n < 1U) ? 1U : ((adc_n > ADC_MAX) ? ADC_MAX : adc_n);
        adc_config();
    }

    n = (read_burst != 0U) ? read_burst : (uint16_t)burst;
    if (read_burst == 0U) {
        if (n < BURST_MIN) { n = BURST_MIN; }
        if (n > BURST_MAX) { n = BURST_MAX; }
    }

    primask = __get_PRIMASK();
    __disable_irq();                                   /* keep USB ISRs out of the ADC window */

    afe_mux(mask_b, val_c);

    carrier_burst_n(n);
    drive_gpio_idle();                                 /* safe idle after burst */

    if (settle_cyc[0] != 0U) {
        dwt_delay_cycles(settle_cyc[0]);
    }
    if (settle_cyc[1] != 0U) {
        dwt_delay_cycles(settle_cyc[1]);
    }

    REG32(GPIOA_BASE + 0x28U) = PIN_PA4;               /* PA4 low: sample */

    if (settle_cyc[2] != 0U) {
        dwt_delay_cycles(settle_cyc[2]);
    }

    adc_sample();
    v = adc_median();

    REG32(GPIOA_BASE + 0x18U) = PIN_PA4;               /* PA4 high: hold */
    REG32(GPIOB_BASE + 0x14U) |= MUX_B_IDLE_MASK;      /* mux idle restore */

    carrier_stop();

    if (recovery != 0U && settle_cyc[3] != 0U) {
        dwt_delay_cycles(settle_cyc[3]);
    }

    if (primask == 0U) {
        __enable_irq();
    }

    return v;
}

void hw_prime(uint16_t mask_b, uint16_t val_c, uint8_t periods, uint8_t repeats)
{
    uint16_t p = (periods < PRIME_MIN) ? PRIME_MIN : periods;
    uint8_t r = (repeats < 1U) ? 1U : ((repeats > 32U) ? 32U : repeats);

    afe_mux(mask_b, val_c);
    for (uint8_t i = 0U; i < r; i++) {
        carrier_burst_n(p);
    }
    carrier_stop();
    REG32(GPIOB_BASE + 0x14U) |= MUX_B_IDLE_MASK;      /* mux idle restore */
}

void hw_keepalive(uint16_t periods)
{
    if (g_enabled) {
        carrier_start(periods);
    }
}

void hw_keepalive_stop(void)
{
    if (g_enabled) {
        carrier_stop();
    }
}
