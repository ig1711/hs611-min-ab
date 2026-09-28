/*
 * USB core driver configuration for the HS611 from-scratch firmware.
 *
 * Adapted from the GD32F3x0 USBFS custom_hid example's usb_conf.h. The eval
 * board dependency (gd32f350r_eval.h) is removed; there is no board support
 * package here.
 */

#ifndef USB_CONF_H
#define USB_CONF_H

#include "stdlib.h"
#include "gd32f3x0.h"

/* USB FIFO size config (units are 32-bit words; total must stay <= 320) */
#define RX_FIFO_FS_SIZE             64U
#define TX0_FIFO_FS_SIZE            64U
#define TX1_FIFO_FS_SIZE            64U
#define TX2_FIFO_FS_SIZE            32U
#define TX3_FIFO_FS_SIZE            32U

#define USB_SOF_OUTPUT              1U
#define USB_LOW_POWER               0U

/* #define VBUS_SENSING_ENABLED */
/* #define USE_HOST_MODE */
#define USE_DEVICE_MODE

#ifndef USE_DEVICE_MODE
    #ifndef USE_HOST_MODE
        #error "USE_DEVICE_MODE or USE_HOST_MODE should be defined!"
    #endif
#endif /* !USE_DEVICE_MODE */

/* __packed keyword used to decrease the data type alignment to 1-byte */
#if defined (__GNUC__)
    #ifdef __packed
        #undef __packed
    #endif
    #define __packed
#elif defined (__TASKING__)
    #define __packed __unaligned
#endif /* __GNUC__ */

#endif /* USB_CONF_H */
