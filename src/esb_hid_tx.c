/*
 * HID over ESB transmitter (keyboard side).
 *
 * Implements the zmk/esb_hid.h interface called from the fork's
 * endpoints.c. Report bodies are queued and sent by a dedicated thread
 * as [type][seq][body] frames over ESB pipe 0.
 *
 * Keyboard/consumer bodies are absolute state: if the queue is full the
 * frame is dropped, but a resync flag makes the TX thread re-send the
 * current state once the queue drains, so a key release can never be
 * lost permanently. Mouse bodies are relative and are simply dropped
 * when the queue is full.
 */

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <esb.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <zmk/esb_hid.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>

#include "esb_hid_frame.h"
#include "esb_hid_link.h"
#include "esb_hid_studio.h"

LOG_MODULE_DECLARE(esb_hid);

/* TX diagnostics on the board's user LEDs (XIAO: led0=red, led1=green,
 * led2=blue), so the radio state is visible without a console:
 * - boot: green blinks 3x once the TX thread runs (module alive)
 * - TX_SUCCESS (ACK received): blue pulse
 * - TX_FAILED (no ACK after ESB retransmits): red pulse
 * Boards without the aliases build without the diagnostics. */
#if DT_NODE_EXISTS(DT_ALIAS(led0)) && DT_NODE_EXISTS(DT_ALIAS(led1)) && DT_NODE_EXISTS(DT_ALIAS(led2))
#define ESB_HID_TX_DIAG_LED 1
static const struct gpio_dt_spec diag_led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec diag_led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec diag_led_blue = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

static void diag_blue_off(struct k_work *work) {
    ARG_UNUSED(work);
    gpio_pin_set_dt(&diag_led_blue, 0);
}
static void diag_red_off(struct k_work *work) {
    ARG_UNUSED(work);
    gpio_pin_set_dt(&diag_led_red, 0);
}
static K_WORK_DELAYABLE_DEFINE(diag_blue_work, diag_blue_off);
static K_WORK_DELAYABLE_DEFINE(diag_red_work, diag_red_off);

static void diag_pulse(const struct gpio_dt_spec *led, struct k_work_delayable *work) {
    gpio_pin_set_dt(led, 1);
    k_work_reschedule(work, K_MSEC(80));
}

static void diag_boot_blink(void) {
    const struct gpio_dt_spec *leds[] = {&diag_led_red, &diag_led_green, &diag_led_blue};

    for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
        if (gpio_is_ready_dt(leds[i])) {
            gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);
        }
    }
    for (int i = 0; i < 3; i++) {
        gpio_pin_set_dt(&diag_led_green, 1);
        k_msleep(90);
        gpio_pin_set_dt(&diag_led_green, 0);
        k_msleep(90);
    }
}

static volatile bool diag_event_seen;
static volatile int diag_init_err;

/* When link start failed, blink |err| in red as two digit groups
 * (tens, gap, ones), repeating. Lets us read the exact error code
 * off the LED without a log console. */
static bool diag_code_blink(struct k_work_delayable *dwork) {
    if (diag_init_err == 0) {
        return false;
    }
    int code = diag_init_err < 0 ? -diag_init_err : diag_init_err;
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
    gpio_pin_set_dt(&diag_led_red, on ? 1 : 0);
    gpio_pin_set_dt(&diag_led_green, 0);
    k_work_reschedule(dwork, K_MSEC(200));
    return true;
}

static void diag_alive(struct k_work *work) {
    if (diag_code_blink(k_work_delayable_from_work(work))) {
        return;
    }
    if (diag_event_seen) {
        gpio_pin_set_dt(&diag_led_green, 0);
        return;
    }
    static bool on;

    on = !on;
    gpio_pin_set_dt(&diag_led_green, on ? 1 : 0);
    k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(300));
}

static K_WORK_DELAYABLE_DEFINE(diag_alive_work, diag_alive);

static void diag_note_init(int err) { diag_init_err = err; }

static void diag_tx_event(enum esb_evt_id evt_id) {
    diag_event_seen = true;
    if (evt_id == ESB_EVENT_TX_SUCCESS) {
        diag_pulse(&diag_led_blue, &diag_blue_work);
    } else if (evt_id == ESB_EVENT_TX_FAILED) {
        diag_pulse(&diag_led_red, &diag_red_work);
    }
}
#else
#define ESB_HID_TX_DIAG_LED 0
static void diag_boot_blink(void) {}
static void diag_note_init(int err) { ARG_UNUSED(err); }
static void diag_tx_event(enum esb_evt_id evt_id) { ARG_UNUSED(evt_id); }
#endif

struct esb_hid_frame {
    uint8_t type;
    uint8_t seq;
    uint8_t len;
    uint8_t body[CONFIG_ESB_MAX_PAYLOAD_LENGTH - ESB_HID_FRAME_HDR_LEN];
};

K_MSGQ_DEFINE(esb_hid_tx_queue, sizeof(struct esb_hid_frame), 16, 4);

static bool link_ready;
static uint8_t seqs[ESB_HID_TYPE_COUNT];
static volatile bool resync_keyboard;
static volatile bool resync_consumer;

static K_SEM_DEFINE(tx_done_sem, 0, 1);
static volatile bool tx_last_ok;

static void esb_hid_tx_event_handler(const struct esb_evt *evt) {
    diag_tx_event(evt->evt_id);
    switch (evt->evt_id) {
    case ESB_EVENT_TX_SUCCESS:
        tx_last_ok = true;
        k_sem_give(&tx_done_sem);
        break;
    case ESB_EVENT_TX_FAILED:
        tx_last_ok = false;
        k_sem_give(&tx_done_sem);
        break;
    default:
        break;
    }
}

static int enqueue_report(uint8_t type, const uint8_t *body, size_t len);

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
/* ------------------------------------------------------------------ */
/* Studio over ESB (downlink via ACK payloads + polling)               */

/* Detection-based poll pacing. The keyboard never polls on a timer
 * while no Studio session exists: the receiver piggybacks a
 * ESB_HID_TYPE_STUDIO_HINT on the ACK of whatever traffic the keyboard
 * sends anyway (key/mouse reports, the 30 s status heartbeat), and
 * only a hint (or real Studio fragments) wakes the fast poll loop.
 *
 * Fast polling (20 ms) is maintained while
 *  - a hint with DOWNLINK_PENDING was seen within the active window, or
 *  - Studio fragments moved in either direction within the window.
 * A hint that only says "connected" starts a short probe window: the
 * Studio app sends its first request right after connecting, and the
 * fragments for it sit in the receiver's ACK FIFO behind the hint, so
 * a brief burst of polls pulls them in. Connected-only hints received
 * on poll ACKs do NOT extend anything, otherwise an idle-but-open
 * Studio window would keep the keyboard polling forever. Once all
 * windows lapse, polling stops completely (zero self-originated
 * traffic) until the next hint rides in on regular traffic. */
#define ESB_HID_STUDIO_POLL_FAST_MS 20
#define ESB_HID_STUDIO_ACTIVE_WINDOW_MS 5000
#define ESB_HID_STUDIO_PROBE_WINDOW_MS 1000

static volatile int64_t studio_last_activity_ms;
static volatile int64_t studio_probe_until_ms;
static int64_t studio_last_poll_ms;

static void studio_note_activity(void) { studio_last_activity_ms = k_uptime_get(); }

int esb_hid_tx_enqueue_studio(const uint8_t *data, size_t len) {
    studio_note_activity();
    return enqueue_report(ESB_HID_TYPE_STUDIO, data, len);
}

static void studio_handle_hint(uint8_t flags) {
    if (flags & ESB_HID_STUDIO_HINT_DOWNLINK_PENDING) {
        /* Real downlink data is queued at the receiver: treat like
         * fragment activity so fast polling is sustained until the
         * fragments themselves take over. */
        studio_note_activity();
    } else if (flags & ESB_HID_STUDIO_HINT_CONNECTED) {
        /* Session exists but nothing queued (yet): probe briefly so a
         * request the app sends right after connecting is pulled in
         * without waiting for the next user keypress / heartbeat. */
        studio_probe_until_ms = k_uptime_get() + ESB_HID_STUDIO_PROBE_WINDOW_MS;
    }
}

/* Drain any pending ACK payloads. Called from the TX thread (never
 * from the ESB event handler, whose context is not safe for this). */
static void studio_drain_ack_payloads(void) {
    struct esb_payload payload;

    while (esb_read_rx_payload(&payload) == 0) {
        if (payload.length > ESB_HID_FRAME_HDR_LEN &&
            payload.data[0] == ESB_HID_TYPE_STUDIO) {
            studio_note_activity();
            esb_hid_studio_on_rx_bytes(&payload.data[ESB_HID_FRAME_HDR_LEN],
                                       payload.length - ESB_HID_FRAME_HDR_LEN);
        } else if (payload.length > ESB_HID_FRAME_HDR_LEN &&
                   payload.data[0] == ESB_HID_TYPE_STUDIO_HINT) {
            studio_handle_hint(payload.data[ESB_HID_FRAME_HDR_LEN]);
        } else {
            LOG_DBG("Ignoring non-studio ACK payload (len %d)", payload.length);
        }
    }
}

static void studio_maybe_poll(void) {
    int64_t now = k_uptime_get();
    bool active = (now - studio_last_activity_ms) < ESB_HID_STUDIO_ACTIVE_WINDOW_MS ||
                  now < studio_probe_until_ms;

    if (!active) {
        /* No session detected: stay silent. The next hint arrives on
         * the ACK of regular traffic (keypress or 30 s heartbeat). */
        return;
    }

    if (now - studio_last_poll_ms >= ESB_HID_STUDIO_POLL_FAST_MS) {
        studio_last_poll_ms = now;
        enqueue_report(ESB_HID_TYPE_POLL, NULL, 0);
    }
}
#endif /* IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO) */

static int enqueue_report(uint8_t type, const uint8_t *body, size_t len) {
    if (len + ESB_HID_FRAME_HDR_LEN > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
        return -EMSGSIZE;
    }

    struct esb_hid_frame frame = {
        .type = type,
        .seq = seqs[type]++,
        .len = (uint8_t)len,
    };
    if (len > 0) {
        memcpy(frame.body, body, len);
    }

    if (k_msgq_put(&esb_hid_tx_queue, &frame, K_NO_WAIT) != 0) {
        if (type == ESB_HID_TYPE_KEYBOARD) {
            resync_keyboard = true;
        } else if (type == ESB_HID_TYPE_CONSUMER) {
            resync_consumer = true;
        }
        return -ENOSPC;
    }

    return 0;
}

/* ------------------------------------------------------------------ */
/* Status frames (battery / layer)                                     */

#define ESB_HID_STATUS_HEARTBEAT K_SECONDS(30)

/* The first few status frames after boot carry ESB_HID_STATUS_FLAG_BOOT
 * so the receiver can edge-detect this reboot and flush the Studio
 * session state it holds for the previous boot. Snapshots fire on
 * layer/battery events plus the 30 s heartbeat (the first at link
 * start +1 s); 3 flagged sends are redundancy against loss, and every
 * status frame is itself retried by send_frame(). */
static uint8_t status_boot_countdown = 3;

static void send_status_snapshot(void) {
    struct esb_hid_status_body body = {
        .battery = ESB_HID_BATTERY_UNKNOWN,
        .layer = (uint8_t)zmk_keymap_highest_layer_active(),
        .flags = (status_boot_countdown > 0) ? ESB_HID_STATUS_FLAG_BOOT : 0,
        .proto_version = ESB_HID_PROTOCOL_VERSION,
    };

    if (status_boot_countdown > 0) {
        status_boot_countdown--;
    }

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    body.battery = zmk_battery_state_of_charge();
#endif

    enqueue_report(ESB_HID_TYPE_STATUS, (const uint8_t *)&body, sizeof(body));
}

static void status_heartbeat(struct k_work *work) {
    ARG_UNUSED(work);
    send_status_snapshot();
    k_work_reschedule(k_work_delayable_from_work(work), ESB_HID_STATUS_HEARTBEAT);
}

static K_WORK_DELAYABLE_DEFINE(status_work, status_heartbeat);

static int esb_hid_tx_listener(const zmk_event_t *eh) {
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    if (as_zmk_battery_state_changed(eh) != NULL) {
        send_status_snapshot();
        return 0;
    }
#endif
    if (as_zmk_layer_state_changed(eh) != NULL) {
        send_status_snapshot();
        return 0;
    }
    return 0;
}

ZMK_LISTENER(esb_hid_tx, esb_hid_tx_listener);
ZMK_SUBSCRIPTION(esb_hid_tx, zmk_layer_state_changed);
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
ZMK_SUBSCRIPTION(esb_hid_tx, zmk_battery_state_changed);
#endif

bool zmk_esb_hid_is_ready(void) { return link_ready; }

int zmk_esb_hid_send_keyboard_report(struct zmk_hid_keyboard_report_body *body) {
    return enqueue_report(ESB_HID_TYPE_KEYBOARD, (const uint8_t *)body, sizeof(*body));
}

int zmk_esb_hid_send_consumer_report(struct zmk_hid_consumer_report_body *body) {
    return enqueue_report(ESB_HID_TYPE_CONSUMER, (const uint8_t *)body, sizeof(*body));
}

#if IS_ENABLED(CONFIG_ZMK_POINTING)
int zmk_esb_hid_send_mouse_report(struct zmk_hid_mouse_report_body *body) {
    return enqueue_report(ESB_HID_TYPE_MOUSE, (const uint8_t *)body, sizeof(*body));
}
#endif /* IS_ENABLED(CONFIG_ZMK_POINTING) */

static void send_frame(const struct esb_hid_frame *frame) {
    struct esb_payload payload = {
        .length = frame->len + ESB_HID_FRAME_HDR_LEN,
        .pipe = 0,
        .noack = 0,
    };
    payload.data[0] = frame->type;
    payload.data[1] = frame->seq;
    memcpy(&payload.data[ESB_HID_FRAME_HDR_LEN], frame->body, frame->len);

    /* Mouse and poll frames are disposable: a single attempt each.
     * Polls in particular must not stall the queue behind retries,
     * the next poll goes out one interval later anyway. */
    int attempts = (frame->type == ESB_HID_TYPE_MOUSE || frame->type == ESB_HID_TYPE_POLL) ? 1 : 3;
    for (int i = 0; i < attempts; i++) {
        tx_last_ok = false;
        k_sem_reset(&tx_done_sem);

        int err = esb_write_payload(&payload);
        if (err) {
            LOG_WRN("esb_write_payload failed: %d", err);
            return;
        }
        if (k_sem_take(&tx_done_sem, K_MSEC(50)) == 0 && tx_last_ok) {
            return;
        }
    }
    LOG_WRN("ESB TX failed (type %d)", frame->type);
}

static void tx_thread(void) {
    struct esb_hid_frame frame;

    diag_boot_blink();
#if ESB_HID_TX_DIAG_LED
    /* Keep blinking slowly until the first TX event, so "module alive
     * but radio silent" stays visible for as long as it lasts. */
    k_work_schedule(&diag_alive_work, K_MSEC(300));
#endif

    while (1) {
        if (k_msgq_get(&esb_hid_tx_queue, &frame, K_MSEC(20)) == 0) {
            send_frame(&frame);
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
            /* A successful send may have carried an ACK payload. */
            studio_drain_ack_payloads();
#endif
            continue;
        }

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
        studio_drain_ack_payloads();
        if (link_ready) {
            studio_maybe_poll();
        }
#endif

        /* Queue drained: re-send current absolute-state reports if any
         * frame had to be dropped earlier. */
        if (resync_keyboard) {
            resync_keyboard = false;
            struct zmk_hid_keyboard_report *report = zmk_hid_get_keyboard_report();
            enqueue_report(ESB_HID_TYPE_KEYBOARD, (const uint8_t *)&report->body,
                           sizeof(report->body));
        }
        if (resync_consumer) {
            resync_consumer = false;
            struct zmk_hid_consumer_report *report = zmk_hid_get_consumer_report();
            enqueue_report(ESB_HID_TYPE_CONSUMER, (const uint8_t *)&report->body,
                           sizeof(report->body));
        }
    }
}

K_THREAD_DEFINE(esb_hid_tx_thread, 1024, tx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

static void link_retry(struct k_work *work);

/* While link start keeps failing, retry every 5 s: the failing stage
 * logs its own error each attempt, so a log console attached at any
 * point sees the reason within seconds (no boot-log race). */
static K_WORK_DELAYABLE_DEFINE(link_retry_work, link_retry);

static void link_retry(struct k_work *work) {
    LOG_INF("esb_hid: TX link start attempt (ESBHID-3)");
    int err = esb_hid_link_start(ESB_MODE_PTX, esb_hid_tx_event_handler);

    diag_note_init(err);
    if (err) {
        LOG_ERR("ESB link start failed: %d (retry in 5s)", err);
        k_work_reschedule(&link_retry_work, K_SECONDS(5));
        return;
    }

    link_ready = true;
    /* The endpoint layer evaluated transports at boot, when the ESB
     * link was not ready yet, and nothing re-triggers it (no USB/BLE
     * state events in this build). Ask it to re-evaluate now, or key
     * reports are silently dropped as "no transport selected". */
    zmk_endpoint_update_current();
    k_work_schedule(&status_work, K_SECONDS(1));
}

static int esb_hid_tx_init(void) {
    /* First attempt is deferred 6 s: a SYS_INIT-time attempt logs
     * before the USB CDC console is attached, so its error (the only
     * one that shows the original failure) would be lost. */
    LOG_INF("esb_hid: TX init, first link start in 6s (ESBHID-3)");
    k_work_schedule(&link_retry_work, K_SECONDS(6));
    return 0;
}

/* Must run before ZMK endpoints init (CONFIG_APPLICATION_INIT_PRIORITY=90),
 * otherwise the endpoint selection happens while the link is not ready
 * yet and the ESB transport is never selected on the keyboard side. */
SYS_INIT(esb_hid_tx_init, APPLICATION, 80);
