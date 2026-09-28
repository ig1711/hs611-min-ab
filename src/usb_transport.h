/*
 * usb_transport.h — vendor-class USB streaming and command intake.
 *
 * The device presents one vendor-specific interface (class 0xFF) with a bulk IN
 * and a bulk OUT endpoint. Frames are produced by scan.c into the buffer that
 * is not in flight and sent whenever the endpoint is free; 64-byte commands
 * arrive on the bulk OUT endpoint and are applied to the scan/acquisition
 * settings.
 */

#ifndef MIN_USB_TRANSPORT_H
#define MIN_USB_TRANSPORT_H

#include <stdint.h>
#include "usbd_core.h"

extern usb_desc min_desc;
extern usb_class_core usbd_min_cb;

/* USB start-of-frame latch: returns 1 once after a SOF arrives, then clears.
 * Used by the main loop to pace acquisition on the 1 ms USB frame. */
uint8_t usb_stream_sof_take(void);

/* Buffer for the next frame (the one not currently in flight). */
uint8_t *usb_stream_buffer(void);

/* Publish the buffer written by scan_step() as ready to send. */
void usb_stream_commit(void);

/* Send the latest ready frame if the endpoint is free. */
void usb_stream_send(void);

#endif /* MIN_USB_TRANSPORT_H */
