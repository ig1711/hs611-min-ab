/*
 * main.c — hs611-min-ab entry point.
 *
 * Brings up the USB vendor-class device and the analog front end, then runs the
 * A/B acquisition loop: when the pacing allows, run one scan into the free USB
 * frame buffer, publish it, and send the latest ready frame if the endpoint is
 * free. Acquisition is decoupled from the transfer, so the front end keeps
 * running across the USB gap.
 */

#include "drv_usb_hw.h"
#include "dwt.h"
#include "usb_transport.h"
#include "scan.h"
#include "afe.h"
#include "acq.h"
#include "estimator.h"

usb_core_driver g_usb_dev;

int main(void)
{
    dwt_init();

    usb_rcu_config();
    usb_timer_init();
    usbd_init(&g_usb_dev, &min_desc, &usbd_min_cb);
    usb_intr_config();

    afe_init();          /* RCU + AFE GPIO (also re-initializes DWT) */
    estimator_init();    /* calibration defaults */
    acq_init();          /* hardware-timer backend + default settings */
    scan_init();         /* pacing defaults */

    for (;;) {
        if (USBD_CONFIGURED == g_usb_dev.dev.cur_status) {
            /* In free-run, acquire continuously; otherwise once per USB frame.
             * Only consume the SOF latch when we are actually pacing on it, so
             * switching back from free-run does not drop a pending frame. */
            uint8_t due = scan_is_free_run() ? 1U : usb_stream_sof_take();
            if (due != 0U) {
                scan_step(usb_stream_buffer());
                usb_stream_commit();
            }
            usb_stream_send();
        }
    }
}
