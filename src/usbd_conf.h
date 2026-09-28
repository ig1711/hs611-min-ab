/*
 * usbd_conf.h — USB device stack configuration for the A/B firmware.
 *
 * One vendor-specific interface with a bulk IN and a bulk OUT endpoint.
 */

#ifndef USBD_CONF_H
#define USBD_CONF_H

#include "usb_conf.h"

#define USBD_CFG_MAX_NUM            1
#define USBD_ITF_MAX_NUM            1       /* interfaces 0..1 (check is <=) */

#define USB_STR_DESC_MAX_SIZE       64

#endif /* USBD_CONF_H */
