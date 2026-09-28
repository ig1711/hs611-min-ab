/*
 * usb_hw.c — USB hardware configuration for the A/B firmware.
 *
 * Sets the USBFS prescaler from the 72 MHz system clock, enables the USBFS
 * interrupt and wakeup EXTI, and provides the USB stack's delays from the DWT
 * cycle counter (TIMER2 is the ADC one-shot trigger, not a delay timer).
 */

#include "drv_usb_hw.h"
#include "dwt.h"

uint32_t usbfs_prescaler = 0U;

void usb_rcu_config(void)
{
    uint32_t system_clock = rcu_clock_freq_get(CK_SYS);

    if (48000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV1;
    } else if (72000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV1_5;
    } else if (96000000U == system_clock) {
        usbfs_prescaler = RCU_USBFS_CKPLL_DIV2;
    }

    rcu_usbfs_clock_config(usbfs_prescaler);
    rcu_periph_clock_enable(RCU_USBFS);
}

void usb_intr_config(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    nvic_irq_enable(USBFS_IRQn, 2U, 0U);

    rcu_periph_clock_enable(RCU_PMU);

    exti_interrupt_flag_clear(EXTI_18);
    exti_init(EXTI_18, EXTI_INTERRUPT, EXTI_TRIG_RISING);
    exti_interrupt_enable(EXTI_18);

    nvic_irq_enable(USBFS_WKUP_IRQn, 1U, 0U);
}

void usb_timer_init(void)
{
    /* No timer-based delay unit: the stack uses DWT delays. */
}

void usb_udelay(const uint32_t usec)
{
    dwt_delay_us(usec);
}

void usb_mdelay(const uint32_t msec)
{
    dwt_delay_us(msec * 1000U);
}
