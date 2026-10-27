/*
 * HID over ESB receiver.
 *
 * Runs on a matrix-less ZMK build connected to the host over USB.
 * Received [type][seq][body] frames are forwarded to USB HID through
 * the fork's zmk_usb_hid_send_report_with_id() hook, which prepends
 * the matching report ID so the wire bytes are identical to a locally
 * generated report.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esb.h>
#include <esb_hid_status.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/hid.h>
#include <zmk/usb_hid.h>

#include "esb_hid_frame.h"
#include "esb_hid_link.h"
#include "esb_hid_studio.h"

LOG_MODULE_DECLARE(esb_hid);

static K_SEM_DEFINE(rx_sem, 0, K_SEM_MAX_LIMIT);

/* Latest status frame per device. Step 1/2: only device 0 exists. */
static struct esb_hid_device_status device_status;

bool esb_hid_get_device_status(uint8_t device_index, struct esb_hid_device_status *out) {
    if (device_index != 0 || !device_status.valid) {
        return false;
    }
    *out = device_status;
    return true;
}

static void handle_status(const uint8_t *data, size_t len) {
    if (len != sizeof(struct esb_hid_status_body)) {
        LOG_WRN("Malformed status frame (len %d)", (int)len);
        return;
    }

    const struct esb_hid_status_body *body = (const struct esb_hid_status_body *)data;

    if (!device_status.valid || device_status.battery != body->battery ||
        device_status.layer != body->layer) {
        if (body->battery == ESB_HID_BATTERY_UNKNOWN) {
            LOG_INF("Peer status: battery unknown, layer %d", body->layer);
        } else {
            LOG_INF("Peer status: battery %d%%, layer %d", body->battery, body->layer);
        }
    }

    device_status.battery = body->battery;
    device_status.layer = body->layer;
    device_status.flags = body->flags;
    device_status.proto_version = body->proto_version;
    device_status.last_update_ms = k_uptime_get();
    device_status.valid = true;
}

static uint8_t last_seq[ESB_HID_TYPE_COUNT];
static bool seq_valid[ESB_HID_TYPE_COUNT];

static const uint8_t report_ids[] = {
    ZMK_HID_REPORT_ID_KEYBOARD,
    ZMK_HID_REPORT_ID_CONSUMER,
    ZMK_HID_REPORT_ID_MOUSE,
};

/* Link indicator: blink the board's green user LED (devicetree alias
 * led1) for as long as the link is alive. Any accepted frame refreshes
 * the liveness timestamp; the TX status heartbeat (every 30 s) keeps
 * the link marked alive while the keyboard is on, so this blinks
 * continuously during normal use and stops within the timeout after
 * the TX goes away. Boards without the alias build without it. */
#if DT_NODE_EXISTS(DT_ALIAS(led1)) && DT_NODE_EXISTS(DT_ALIAS(led0))
#define ESB_HID_LINK_LED 1

#define ESB_HID_LINK_ALIVE_MS 45000 /* 1.5x the TX heartbeat period */
#define ESB_HID_LINK_BLINK_MS 250

static const struct gpio_dt_spec link_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec link_led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static int64_t link_last_frame_ms;
static bool link_led_on;
static int rx_init_err;

static void link_blink(struct k_work *work) {
    if (rx_init_err != 0) {
        /* Link start failed: blink |err| in red as two digit groups
         * (tens, gap, ones), repeating — same code as the TX diag. */
        int code = rx_init_err < 0 ? -rx_init_err : rx_init_err;
        int tens = code / 10;
        int ones = code % 10;
        int cycle = 2 * tens + 3 + 2 * ones + 8;
        static int slot;

        int s = slot++ % cycle;
        bool on = false;

        if (s < 2 * tens) {
            on = (s % 2) == 0;
        } else if (s >= 2 * tens + 3 && s < 2 * tens + 3 + 2 * ones) {
            on = ((s - (2 * tens + 3)) % 2) == 0;
        }
        gpio_pin_set_dt(&link_led_red, on ? 1 : 0);
        gpio_pin_set_dt(&link_led, 0);
        k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(200));
        return;
    }

    bool alive = link_last_frame_ms > 0 &&
                 (k_uptime_get() - link_last_frame_ms) < ESB_HID_LINK_ALIVE_MS;

    link_led_on = alive && !link_led_on;
    gpio_pin_set_dt(&link_led, link_led_on ? 1 : 0);

    k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(ESB_HID_LINK_BLINK_MS));
}

static K_WORK_DELAYABLE_DEFINE(link_blink_work, link_blink);

static void link_led_note_frame(void) { link_last_frame_ms = k_uptime_get(); }
#else
#define ESB_HID_LINK_LED 0
static void link_led_note_frame(void) {}
#endif

static void esb_hid_rx_event_handler(const struct esb_evt *evt) {
    if (evt->evt_id == ESB_EVENT_RX_RECEIVED) {
        k_sem_give(&rx_sem);
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
        /* Receiving a packet consumed one queued ACK payload; the
         * Studio forwarder may be able to load the next fragment. */
        esb_hid_studio_fwd_note_link_event();
#endif
    }
}

static void handle_payload(const struct esb_payload *payload) {
    if (payload->length < ESB_HID_FRAME_HDR_LEN) {
        return;
    }

    uint8_t type = payload->data[0];
    uint8_t seq = payload->data[1];

    if (type >= ESB_HID_TYPE_COUNT) {
        return;
    }

    /* Keyboard/consumer/status are state: drop exact duplicates
     * (e.g. a re-delivered frame after a lost ACK). Mouse is relative;
     * every frame carries new motion, so forward all of them. */
    if (type != ESB_HID_TYPE_MOUSE) {
        if (seq_valid[type] && seq == last_seq[type]) {
            return;
        }
        seq_valid[type] = true;
        last_seq[type] = seq;
    }

    link_led_note_frame();

    if (type == ESB_HID_TYPE_STUDIO) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
        esb_hid_studio_fwd_to_pc(&payload->data[ESB_HID_FRAME_HDR_LEN],
                                 payload->length - ESB_HID_FRAME_HDR_LEN);
#endif
        return;
    }

    if (type == ESB_HID_TYPE_POLL) {
        /* Polls exist only to pull ACK payloads; nothing to forward.
         * (The ACK refill itself is triggered from the RX event
         * handler, which fires for polls as well.) */
        return;
    }

    if (type == ESB_HID_TYPE_STATUS) {
        handle_status(&payload->data[ESB_HID_FRAME_HDR_LEN],
                      payload->length - ESB_HID_FRAME_HDR_LEN);
        return;
    }

    int err = zmk_usb_hid_send_report_with_id(report_ids[type],
                                              &payload->data[ESB_HID_FRAME_HDR_LEN],
                                              payload->length - ESB_HID_FRAME_HDR_LEN);
    if (err) {
        LOG_WRN("USB HID forward failed (type %d): %d", type, err);
    }
}

static void rx_thread(void) {
    struct esb_payload payload;

    while (1) {
        k_sem_take(&rx_sem, K_FOREVER);
        while (esb_read_rx_payload(&payload) == 0) {
            handle_payload(&payload);
        }
    }
}

K_THREAD_DEFINE(esb_hid_rx_thread, 1024, rx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

static void rx_link_retry(struct k_work *work);

/* Same retry contract as the TX side: while link start fails, retry
 * every 5 s so the failing stage keeps logging its error. */
static K_WORK_DELAYABLE_DEFINE(rx_link_retry_work, rx_link_retry);

static void rx_link_retry(struct k_work *work) {
    LOG_INF("esb_hid: RX link start attempt (ESBHID-3)");
    int err = esb_hid_link_start(ESB_MODE_PRX, esb_hid_rx_event_handler);

#if ESB_HID_LINK_LED
    rx_init_err = err;
#endif
    if (err) {
        LOG_ERR("ESB RX link start failed: %d (retry in 5s)", err);
        k_work_reschedule(&rx_link_retry_work, K_SECONDS(5));
    }
}

static int esb_hid_rx_init(void) {
#if ESB_HID_LINK_LED
    if (gpio_is_ready_dt(&link_led)) {
        gpio_pin_configure_dt(&link_led, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&link_led_red, GPIO_OUTPUT_INACTIVE);
        k_work_schedule(&link_blink_work, K_MSEC(ESB_HID_LINK_BLINK_MS));
    }
#endif
    /* First attempt deferred 6 s, same reason as TX: don't lose the
     * original error to the not-yet-attached USB console. */
    LOG_INF("esb_hid: RX init, first link start in 6s (ESBHID-3)");
    k_work_schedule(&rx_link_retry_work, K_SECONDS(6));
    return 0;
}

/* After the USB HID init (CONFIG_ZMK_USB_HID_INIT_PRIORITY=95) so early
 * frames are not forwarded to a not-yet-registered HID device. */
SYS_INIT(esb_hid_rx_init, APPLICATION, 96);
