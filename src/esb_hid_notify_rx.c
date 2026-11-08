/*
 * Deck notification channel (receiver side), step M1 skeleton.
 *
 * The receiver exposes a third USB CDC ACM port (devicetree node
 * "deck_notify_cdc", compatible zephyr,cdc-acm-uart) dedicated to
 * notifications from a PC-side notifier program, separate from the
 * Studio forwarder port so the two never contend (design:
 * deck-notifications-design.md).
 *
 * Wire format (design 3.1): fixed 6-byte header — "DN" magic,
 * protocol version, message type, little-endian payload length —
 * followed by the payload (at most ESB_HID_NOTIFY_PAYLOAD_MAX
 * bytes). No CRC; the CDC link is treated as reliable.
 *
 * M1 scope: the channel skeleton only. The RX thread assembles
 * frames (resynchronising on the magic), answers HELLO with
 * HELLO_ACK, and logs + drops everything else. NOTIFY/RESOLVE
 * parsing, the UI callback and the notification store arrive in
 * step M2 / D1. The pending count reported in HELLO_ACK is
 * therefore always 0 for now.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <errno.h>
#include <string.h>

#include "esb_hid_notify.h"

LOG_MODULE_DECLARE(esb_hid);

#if DT_NODE_EXISTS(DT_NODELABEL(deck_notify_cdc))

static const struct device *const notify_cdc_dev = DEVICE_DT_GET(DT_NODELABEL(deck_notify_cdc));

/* Serialises frame writes: HELLO_ACK replies leave from the RX
 * thread, ANSWER/ACK will leave from the display side (M2/D2). */
static K_MUTEX_DEFINE(notify_tx_lock);

int esb_hid_notify_send(uint8_t type, const uint8_t *payload, size_t len) {
    if (len > ESB_HID_NOTIFY_PAYLOAD_MAX) {
        return -EINVAL;
    }

    const uint8_t hdr[ESB_HID_NOTIFY_HDR_LEN] = {
        ESB_HID_NOTIFY_MAGIC_0,
        ESB_HID_NOTIFY_MAGIC_1,
        ESB_HID_NOTIFY_PROTO_VER,
        type,
        (uint8_t)(len & 0xFF),
        (uint8_t)((len >> 8) & 0xFF),
    };

    k_mutex_lock(&notify_tx_lock, K_FOREVER);
    for (size_t i = 0; i < sizeof(hdr); i++) {
        uart_poll_out(notify_cdc_dev, hdr[i]);
    }
    for (size_t i = 0; i < len; i++) {
        uart_poll_out(notify_cdc_dev, payload[i]);
    }
    k_mutex_unlock(&notify_tx_lock);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Frame assembly (design 3.1): hunt the magic, then read the fixed  */
/* header, then the payload. Bytes that are not part of a frame are  */
/* skipped one by one so a noisy stream resynchronises.              */

enum notify_rx_state {
    NOTIFY_RX_MAGIC_0,
    NOTIFY_RX_MAGIC_1,
    NOTIFY_RX_VER,
    NOTIFY_RX_TYPE,
    NOTIFY_RX_LEN_LO,
    NOTIFY_RX_LEN_HI,
    NOTIFY_RX_PAYLOAD,
};

static enum notify_rx_state rx_state;
static uint8_t rx_ver;
static uint8_t rx_type;
static uint16_t rx_len;
static uint16_t rx_pos;
static uint8_t rx_payload[ESB_HID_NOTIFY_PAYLOAD_MAX];

static void notify_dispatch_frame(void) {
    if (rx_ver != ESB_HID_NOTIFY_PROTO_VER) {
        LOG_WRN("Notify frame with unsupported version %u (type 0x%02x, %u bytes), dropping",
                rx_ver, rx_type, rx_len);
        return;
    }

    switch (rx_type) {
    case ESB_HID_NOTIFY_TYPE_HELLO: {
        /* Payload: proto_ver u8, notifier name (<= 15 bytes). */
        uint8_t peer_ver = rx_len > 0 ? rx_payload[0] : 0;
        char name[16] = "";
        if (rx_len > 1) {
            size_t name_len = rx_len - 1;
            if (name_len > sizeof(name) - 1) {
                name_len = sizeof(name) - 1;
            }
            memcpy(name, &rx_payload[1], name_len);
        }
        LOG_INF("Notify HELLO from \"%s\" (proto %u)", name, peer_ver);

        /* Payload: proto_ver u8, pending count u8 (no store yet). */
        const uint8_t ack[2] = {ESB_HID_NOTIFY_PROTO_VER, 0};
        int err = esb_hid_notify_send(ESB_HID_NOTIFY_TYPE_HELLO_ACK, ack, sizeof(ack));
        if (err) {
            LOG_WRN("Notify HELLO_ACK send failed: %d", err);
        }
        break;
    }
    default:
        /* M1 skeleton: NOTIFY/RESOLVE land in M2. */
        LOG_INF("Notify frame type 0x%02x (%u bytes) dropped (M1 skeleton)", rx_type, rx_len);
        break;
    }
}

static void notify_rx_feed(uint8_t byte) {
    switch (rx_state) {
    case NOTIFY_RX_MAGIC_0:
        if (byte == ESB_HID_NOTIFY_MAGIC_0) {
            rx_state = NOTIFY_RX_MAGIC_1;
        }
        break;
    case NOTIFY_RX_MAGIC_1:
        rx_state = byte == ESB_HID_NOTIFY_MAGIC_1 ? NOTIFY_RX_VER : NOTIFY_RX_MAGIC_0;
        break;
    case NOTIFY_RX_VER:
        rx_ver = byte;
        rx_state = NOTIFY_RX_TYPE;
        break;
    case NOTIFY_RX_TYPE:
        rx_type = byte;
        rx_state = NOTIFY_RX_LEN_LO;
        break;
    case NOTIFY_RX_LEN_LO:
        rx_len = byte;
        rx_state = NOTIFY_RX_LEN_HI;
        break;
    case NOTIFY_RX_LEN_HI:
        rx_len |= (uint16_t)byte << 8;
        rx_pos = 0;
        if (rx_len > ESB_HID_NOTIFY_PAYLOAD_MAX) {
            /* Cannot trust this length; drop the frame and hunt
             * for the next magic instead of skipping bytes. */
            LOG_WRN("Notify frame length %u exceeds max %d, dropping", rx_len,
                    ESB_HID_NOTIFY_PAYLOAD_MAX);
            rx_state = NOTIFY_RX_MAGIC_0;
        } else if (rx_len == 0) {
            notify_dispatch_frame();
            rx_state = NOTIFY_RX_MAGIC_0;
        } else {
            rx_state = NOTIFY_RX_PAYLOAD;
        }
        break;
    case NOTIFY_RX_PAYLOAD:
        rx_payload[rx_pos++] = byte;
        if (rx_pos >= rx_len) {
            notify_dispatch_frame();
            rx_state = NOTIFY_RX_MAGIC_0;
        }
        break;
    }
}

/* Read the CDC DTR line, like the Studio forwarder: a notifier
 * opening the port asserts DTR. Edges are only logged at M1 (the
 * notifier re-probes with HELLO after reconnect anyway). */
static void notify_check_dtr_edge(void) {
    static bool sample_valid;
    static bool was_connected;
    uint32_t dtr = 0;

    if (uart_line_ctrl_get(notify_cdc_dev, UART_LINE_CTRL_DTR, &dtr) != 0) {
        return;
    }
    bool connected = dtr != 0;
    if (sample_valid && connected != was_connected) {
        LOG_INF("Notify client %s (DTR)", connected ? "connected" : "disconnected");
        if (!connected) {
            rx_state = NOTIFY_RX_MAGIC_0;
        }
    }
    sample_valid = true;
    was_connected = connected;
}

static void esb_notify_rx_thread(void) {
    int idle_ms = 0;

    while (1) {
        uint8_t byte;

        if (uart_poll_in(notify_cdc_dev, &byte) < 0) {
            k_sleep(K_MSEC(1));
            idle_ms++;
            if (idle_ms >= 250) {
                idle_ms = 0;
                notify_check_dtr_edge();
            }
            continue;
        }
        idle_ms = 0;
        notify_rx_feed(byte);
    }
}

K_THREAD_DEFINE(esb_hid_notify_rx_thread, 1024, esb_notify_rx_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(8), 0, 0);

static int esb_hid_notify_rx_init(void) {
    if (!device_is_ready(notify_cdc_dev)) {
        LOG_ERR("Notify CDC device not ready");
        return -ENODEV;
    }
    LOG_INF("Deck notify forwarder ready (CDC %s)", notify_cdc_dev->name);
    return 0;
}

SYS_INIT(esb_hid_notify_rx_init, APPLICATION, 98);

#else /* DT_NODE_EXISTS(DT_NODELABEL(deck_notify_cdc)) */

/* Shield did not provide the CDC node: keep call sites linkable,
 * the channel is simply absent. */
int esb_hid_notify_send(uint8_t type, const uint8_t *payload, size_t len) {
    ARG_UNUSED(type);
    ARG_UNUSED(payload);
    ARG_UNUSED(len);
    return -ENODEV;
}

#endif
