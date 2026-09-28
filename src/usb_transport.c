/*
 * usb_transport.c — vendor-class USB streaming and command intake.
 * See usb_transport.h and protocol.h.
 */

#include "usb_transport.h"
#include "protocol.h"
#include "board.h"
#include "dwt.h"
#include "acq.h"
#include "scan.h"
#include "estimator.h"

#include "gd32f3x0.h"
#include "usbd_enum.h"
#include "drv_usb_hw.h"

#define DBG_VID             0x256CU
#define DBG_PID             0x6111U
#define DBG_IN_EP           0x81U
#define DBG_OUT_EP          0x02U
#define DBG_EP_PACKET       64U

extern usb_core_driver g_usb_dev;

/* ---- descriptors -------------------------------------------------------- */
static const usb_desc_dev min_dev_desc = {
    .header =
    {
        .bLength          = USB_DEV_DESC_LEN,
        .bDescriptorType  = USB_DESCTYPE_DEV
    },
    .bcdUSB                = 0x0210U,
    .bDeviceClass          = 0x00U,
    .bDeviceSubClass       = 0x00U,
    .bDeviceProtocol       = 0x00U,
    .bMaxPacketSize0       = USB_FS_EP0_MAX_LEN,
    .idVendor              = DBG_VID,
    .idProduct             = DBG_PID,
    .bcdDevice             = 0x0200U,
    .iManufacturer         = STR_IDX_MFC,
    .iProduct              = STR_IDX_PRODUCT,
    .iSerialNumber         = STR_IDX_SERIAL,
    .bNumberConfigurations = USBD_CFG_MAX_NUM
};

static const uint8_t min_config_desc[32] = {
    0x09, 0x02, 0x20, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
    0x09, 0x04, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00,
    0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
    0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
};

static const uint8_t min_bos_desc[29] = {
    0x05, 0x0F, 0x1D, 0x00, 0x01,
    0x18, 0x10, 0x05, 0x00,
    0x38, 0xB6, 0x08, 0x34, 0xA9, 0x09, 0xA0, 0x47,
    0x8B, 0xFD, 0xA0, 0x76, 0x88, 0x15, 0xB6, 0x65,
    0x00, 0x01, 0x01, 0x00
};

static const usb_desc_ep min_ep_in = {
    .header = { .bLength = sizeof(usb_desc_ep), .bDescriptorType = USB_DESCTYPE_EP },
    .bEndpointAddress = DBG_IN_EP, .bmAttributes = USB_EP_ATTR_BULK,
    .wMaxPacketSize = DBG_EP_PACKET, .bInterval = 0U
};
static const usb_desc_ep min_ep_out = {
    .header = { .bLength = sizeof(usb_desc_ep), .bDescriptorType = USB_DESCTYPE_EP },
    .bEndpointAddress = DBG_OUT_EP, .bmAttributes = USB_EP_ATTR_BULK,
    .wMaxPacketSize = DBG_EP_PACKET, .bInterval = 0U
};

static const usb_desc_LANGID min_str_langid = {
    .header = { .bLength = sizeof(usb_desc_LANGID), .bDescriptorType = USB_DESCTYPE_STR },
    .wLANGID = ENG_LANGID
};
static const usb_desc_str min_str_manufacturer = {
    .header = { .bLength = USB_STRING_LEN(9U), .bDescriptorType = USB_DESCTYPE_STR },
    .unicode_string = {'H','S','6','1','1','-','D','E','V'}
};
static const usb_desc_str min_str_product = {
    .header = { .bLength = USB_STRING_LEN(15U), .bDescriptorType = USB_DESCTYPE_STR },
    .unicode_string = {'H','S','6','1','1',' ','A','/','B',' ','(','W','U','S','B',')'}
};
static usb_desc_str min_str_serial = {
    .header = { .bLength = USB_STRING_LEN(16U), .bDescriptorType = USB_DESCTYPE_STR },
    .unicode_string = {'H','S','6','1','1','-','0','0','0','0','0','0','0','0','0','0'}
};

static void *const min_strings[STR_IDX_MAX] = {
    [STR_IDX_LANGID]  = (uint8_t *)&min_str_langid,
    [STR_IDX_MFC]     = (uint8_t *)&min_str_manufacturer,
    [STR_IDX_PRODUCT] = (uint8_t *)&min_str_product,
    [STR_IDX_SERIAL]  = (uint8_t *)&min_str_serial
};

usb_desc min_desc = {
    .dev_desc    = (uint8_t *)&min_dev_desc,
    .config_desc = (uint8_t *)min_config_desc,
    .bos_desc    = (uint8_t *)min_bos_desc,
    .strings     = min_strings
};

/* ---- streaming state ---------------------------------------------------- */
static uint8_t  g_seq;
static uint8_t  g_out_buf[PROTO_CMD_LEN];
static uint8_t  g_txbuf[2][PROTO_FRAME_LEN];
static volatile uint8_t g_inflight;
static uint8_t  g_send_idx;
static volatile int8_t  g_ready_idx = -1;
static uint8_t  g_fill_idx;
static volatile uint8_t g_sof_pending;

uint8_t usb_stream_sof_take(void)
{
    if (g_sof_pending != 0U) {
        g_sof_pending = 0U;
        return 1U;
    }
    return 0U;
}

uint8_t *usb_stream_buffer(void)
{
    /* Always build into the buffer that is not currently in flight. */
    g_fill_idx = (uint8_t)(g_send_idx ^ 1U);
    return g_txbuf[g_fill_idx];
}

void usb_stream_commit(void)
{
    g_txbuf[g_fill_idx][PROTO_OFF_SEQ] = g_seq++;
    g_ready_idx = (int8_t)g_fill_idx;
}

void usb_stream_send(void)
{
    uint8_t idx;

    if ((g_inflight != 0U) || (g_ready_idx < 0)) {
        return;
    }
    idx = (uint8_t)g_ready_idx;
    if (usbd_ep_send(&g_usb_dev, DBG_IN_EP, g_txbuf[idx], PROTO_FRAME_LEN) == USBD_OK) {
        g_ready_idx = -1;
        g_send_idx = idx;
        g_inflight = 1U;
    }
}

/* ---- command dispatch --------------------------------------------------- */
static uint16_t get_u16(const uint8_t *b, uint8_t off)
{
    return (uint16_t)((uint16_t)b[off] | ((uint16_t)b[off + 1U] << 8));
}

static void apply_command(const uint8_t *b)
{
    switch (b[0]) {
    case PROTO_CMD_PING:
        break;
    case PROTO_CMD_SET_FREQ:
        acq_set_freq(b[1]);
        break;
    case PROTO_CMD_SET_FREQ_ARR:
        acq_set_freq_arr(get_u16(b, 1U));
        break;
    case PROTO_CMD_SET_BACKEND:
        acq_set_backend((b[1] == ACQ_BACKEND_HW) ? ACQ_BACKEND_HW : ACQ_BACKEND_SW);
        break;
    case PROTO_CMD_SET_BURST:
        acq_set_burst(b[1]);
        break;
    case PROTO_CMD_SET_SETTLE:
        acq_set_settle(b[1], b[2], b[3], b[4]);
        break;
    case PROTO_CMD_SET_ADC:
        acq_set_adc(b[1], b[2]);
        break;
    case PROTO_CMD_SET_RECOVERY:
        acq_set_recovery(b[1]);
        break;
    case PROTO_CMD_SET_WINDOW:
        scan_set_window(b[1], b[2]);
        break;
    case PROTO_CMD_SET_RECENTER:
        scan_set_recenter(b[1], b[2], b[3], b[4], b[5]);
        break;
    case PROTO_CMD_SET_SCAN_ORDER:
        scan_set_scan_order(b[1]);
        break;
    case PROTO_CMD_SET_ESTIMATOR:
        estimator_set(b[1]);
        break;
    case PROTO_CMD_SET_LOGAUSS:
        estimator_set_loggauss((b[1] != 0U) ? 1U : 0U, get_u16(b, 2U),
                               (int16_t)get_u16(b, 4U), (int16_t)get_u16(b, 6U));
        break;
    case PROTO_CMD_SET_NCENTROID:
        estimator_set_ncentroid(get_u16(b, 1U));
        break;
    case PROTO_CMD_SET_REACQ:
        scan_set_reacq(get_u16(b, 1U), b[3], get_u16(b, 4U));
        break;
    case PROTO_CMD_SET_RAMP:
        scan_set_ramp(b[1], b[2], b[3], (int16_t)get_u16(b, 4U), b[6]);
        break;
    case PROTO_CMD_SET_PACING:
        scan_set_pacing(b[1]);
        break;
    case PROTO_CMD_SET_WARMUP:
        scan_set_warmup(b[1]);
        break;
    case PROTO_CMD_SET_FLAT_TOL:
        scan_set_flat_tol(get_u16(b, 1U));
        break;
    case PROTO_CMD_REPEAT_COIL:
        scan_set_repeat_coil(b[1]);
        break;
    default:
        break;
    }
}

/* ---- class callbacks ---------------------------------------------------- */
static uint8_t min_core_init(usb_dev *udev, uint8_t config_index)
{
    (void)config_index;

    g_inflight = 0U;
    g_send_idx = 0U;
    g_ready_idx = -1;
    g_sof_pending = 0U;
    g_seq = 0U;

    (void)usbd_ep_setup(udev, &min_ep_in);
    (void)usbd_ep_setup(udev, &min_ep_out);
    (void)usbd_ep_recev(udev, DBG_OUT_EP, g_out_buf, PROTO_CMD_LEN);

    return USBD_OK;
}

static uint8_t min_core_deinit(usb_dev *udev, uint8_t config_index)
{
    (void)config_index;

    (void)usbd_ep_clear(udev, DBG_IN_EP);
    (void)usbd_ep_clear(udev, DBG_OUT_EP);
    g_inflight = 0U;

    return USBD_OK;
}

static uint8_t min_core_req_handler(usb_dev *udev, usb_req *req)
{
    (void)udev;
    (void)req;
    return USBD_FAIL;
}

static uint8_t min_core_data_in(usb_dev *udev, uint8_t ep_num)
{
    (void)udev;
    if (ep_num == 1U) {
        g_inflight = 0U;
    }
    return USBD_OK;
}

static uint8_t min_core_data_out(usb_dev *udev, uint8_t ep_num)
{
    if (ep_num == 2U) {
        apply_command(g_out_buf);
        (void)usbd_ep_recev(udev, DBG_OUT_EP, g_out_buf, PROTO_CMD_LEN);
    }
    return USBD_OK;
}

static uint8_t min_core_sof(usb_dev *udev)
{
    (void)udev;
    g_sof_pending = 1U;
    return USBD_OK;
}

usb_class_core usbd_min_cb = {
    .command   = 0xFFU,
    .alter_set = 0U,

    .init      = min_core_init,
    .deinit    = min_core_deinit,

    .req_proc  = min_core_req_handler,

    .data_in   = min_core_data_in,
    .data_out  = min_core_data_out,
    .SOF       = min_core_sof
};
