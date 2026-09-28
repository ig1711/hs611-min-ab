/*
 * usb_it.c — USB interrupt handlers.
 *
 * The startup vector table references these symbols as weak aliases of
 * Default_Handler; defining them here overrides the defaults.
 */

#include "drv_usbd_int.h"
#include "gd32f3x0.h"

extern usb_core_driver g_usb_dev;

/* USBFS global interrupt: device state machine. */
void USBFS_IRQHandler(void)
{
    usbd_isr(&g_usb_dev);
}

/* USBFS wakeup: low power is disabled, so just clear the EXTI flag. */
void USBFS_WKUP_IRQHandler(void)
{
    exti_interrupt_flag_clear(EXTI_18);
}
