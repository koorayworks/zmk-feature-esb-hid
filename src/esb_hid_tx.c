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
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <esb.h>
#include <esb_hid.h>
#include <esb_hid_status.h>
#include <zmk/battery.h>
#include <zmk/endpoints.h>
#include <zmk/esb_hid.h>
#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/endpoint_changed.h>
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

/* ------------------------------------------------------------------ */
/* Slot assignment and receiver profiles (step 2 + multi-receiver)     */

/* Own slot = ESB pipe for all normal traffic, taken from the active
 * receiver profile. Profiles are resolved once at link start from,
 * in order of preference:
 *  - the stored profile list ("esb_hid/receivers", written by
 *    pairing; multi-receiver builds only), most recent first;
 *  - the legacy settings override ("esb_hid/slot", written when a
 *    receiver assigns a slot during pairing), as a single
 *    unknown-identity profile;
 *  - CONFIG_ZMK_ESB_HID_SLOT, likewise as a single profile.
 * A setup with exactly one profile (everything that predates
 * multi-receiver support) behaves exactly like the original
 * single-slot TX: no hunting ever switches the pipe. */
static uint8_t own_slot = CONFIG_ZMK_ESB_HID_SLOT;
/* Pipe the next transmitted frame goes out on: own_slot normally,
 * ESB_HID_PAIRING_PIPE while pairing, the hunted candidate's slot
 * while hunting. Read by send_frame(). */
static volatile uint8_t tx_pipe = CONFIG_ZMK_ESB_HID_SLOT;

/* Receiver profiles, most recently paired/confirmed first.
 * active_profile indexes the profile in use; own_slot mirrors its
 * slot. An all-zero rx_id means "identity unknown": the assignment
 * came from a receiver whose firmware does not append its ID to
 * PAIR_ASSIGN, or the profile predates multi-receiver support. All
 * unknown-identity assignments share a single profile entry: without
 * an ID there is no way to tell two such receivers apart. */
#define ESB_HID_MAX_RECEIVERS 4

struct esb_hid_receiver_profile {
    uint8_t rx_id[ESB_HID_DEVICE_ID_LEN];
    uint8_t slot;
};

static struct esb_hid_receiver_profile profiles[ESB_HID_MAX_RECEIVERS];
static uint8_t profile_count = 1;
static uint8_t active_profile;

/* On-disk profile list ("esb_hid/receivers"). Explicit layout, not
 * the RAM struct. */
struct esb_hid_receivers_disk {
    uint8_t version; /* = 1 */
    uint8_t count;
    struct {
        uint8_t rx_id[ESB_HID_DEVICE_ID_LEN];
        uint8_t slot;
    } entries[ESB_HID_MAX_RECEIVERS];
};

BUILD_ASSERT(sizeof(struct esb_hid_receivers_disk) == 38,
             "receivers disk format must be 38 bytes");

#if IS_ENABLED(CONFIG_SETTINGS)
static bool slot_override_valid;
static uint8_t slot_override;
static bool receivers_staged_valid;
static struct esb_hid_receivers_disk receivers_staging;

static int esb_hid_tx_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "slot", &next) && !next) {
        if (len != sizeof(slot_override)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &slot_override, sizeof(slot_override));

        if (rc >= 0) {
            slot_override_valid = slot_override < ESB_HID_SLOT_COUNT;
            return 0;
        }
        return rc;
    }
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_MULTI_RECEIVER)
    if (settings_name_steq(name, "receivers", &next) && !next) {
        if (len != sizeof(receivers_staging)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &receivers_staging, sizeof(receivers_staging));

        if (rc < 0) {
            return rc;
        }
        if (receivers_staging.version != 1 || receivers_staging.count < 1 ||
            receivers_staging.count > ESB_HID_MAX_RECEIVERS) {
            return -EINVAL;
        }
        for (int i = 0; i < receivers_staging.count; i++) {
            if (receivers_staging.entries[i].slot >= ESB_HID_SLOT_COUNT) {
                return -EINVAL;
            }
        }
        receivers_staged_valid = true;
        return 0;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_ESB_HID_MULTI_RECEIVER) */
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(esb_hid_tx, "esb_hid", NULL, esb_hid_tx_settings_set, NULL, NULL);

/* Persist the profile list. The legacy "esb_hid/slot" key is always
 * written as a downgrade mirror of the active slot, so firmware
 * without multi-receiver support keeps using the most recently
 * active slot. */
static void profiles_save(void) {
    int err;

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_MULTI_RECEIVER)
    struct esb_hid_receivers_disk disk = {.version = 1, .count = profile_count};

    for (int i = 0; i < profile_count; i++) {
        memcpy(disk.entries[i].rx_id, profiles[i].rx_id, ESB_HID_DEVICE_ID_LEN);
        disk.entries[i].slot = profiles[i].slot;
    }
    err = settings_save_one("esb_hid/receivers", &disk, sizeof(disk));
    if (err) {
        LOG_ERR("Failed to save receiver profiles: %d", err);
    }
#endif
    err = settings_save_one("esb_hid/slot", &own_slot, sizeof(own_slot));
    if (err) {
        LOG_ERR("Failed to save active slot: %d", err);
    }
}
#else
static void profiles_save(void) {}
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

static void profiles_move_to_front(uint8_t idx) {
    if (idx == 0 || idx >= profile_count) {
        return;
    }
    struct esb_hid_receiver_profile tmp = profiles[idx];

    memmove(&profiles[1], &profiles[0], idx * sizeof(profiles[0]));
    profiles[0] = tmp;
}

/* Record a pairing assignment: the receiver identified by rx_id
 * (NULL = identity unknown, e.g. an older receiver) assigned us
 * `slot`. The matching entry is updated or a new one inserted, the
 * entry moves to the front (most recent first) and becomes active.
 * Inserting into a full list evicts the last entry, i.e. the least
 * recently paired/confirmed receiver. Persists via profiles_save().
 * Single-receiver builds keep exactly the legacy behaviour: the one
 * profile is simply overwritten. */
static void profiles_upsert(const uint8_t *rx_id, uint8_t slot) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_MULTI_RECEIVER)
    int found = -1;

    for (int i = 0; i < profile_count; i++) {
        bool match;

        if (rx_id != NULL) {
            match = memcmp(profiles[i].rx_id, rx_id, ESB_HID_DEVICE_ID_LEN) == 0;
        } else {
            match = true;
            for (int j = 0; j < ESB_HID_DEVICE_ID_LEN; j++) {
                if (profiles[i].rx_id[j] != 0) {
                    match = false;
                    break;
                }
            }
        }
        if (match) {
            found = i;
            break;
        }
    }

    if (found >= 0) {
        profiles[found].slot = slot;
        profiles_move_to_front((uint8_t)found);
    } else {
        uint8_t kept = profile_count < ESB_HID_MAX_RECEIVERS ? profile_count
                                                             : ESB_HID_MAX_RECEIVERS - 1;

        memmove(&profiles[1], &profiles[0], kept * sizeof(profiles[0]));
        if (rx_id != NULL) {
            memcpy(profiles[0].rx_id, rx_id, ESB_HID_DEVICE_ID_LEN);
        } else {
            memset(profiles[0].rx_id, 0, ESB_HID_DEVICE_ID_LEN);
        }
        profiles[0].slot = slot;
        profile_count = kept + 1;
    }
#else
    if (rx_id != NULL) {
        memcpy(profiles[0].rx_id, rx_id, ESB_HID_DEVICE_ID_LEN);
    } else {
        memset(profiles[0].rx_id, 0, ESB_HID_DEVICE_ID_LEN);
    }
    profiles[0].slot = slot;
    profile_count = 1;
#endif
    active_profile = 0;
    own_slot = slot;
    profiles_save();
}

static bool profiles_resolved;

static void resolve_own_slot(void) {
    if (profiles_resolved) {
        /* Later link-start retries must not rebuild the list: the
         * staged settings values only feed the first resolution,
         * pairing/hunting state lives in the RAM copy. */
        tx_pipe = own_slot;
        return;
    }
    profiles_resolved = true;

#if IS_ENABLED(CONFIG_SETTINGS)
    /* ZMK's main() already ran settings_load() once; load our subtree
     * again here so the values are in place even if that ordering ever
     * changes. Idempotent: the handler just re-reads the same keys. */
    settings_load_subtree("esb_hid");
#endif

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_MULTI_RECEIVER) && IS_ENABLED(CONFIG_SETTINGS)
    if (receivers_staged_valid) {
        profile_count = receivers_staging.count;
        for (int i = 0; i < profile_count; i++) {
            memcpy(profiles[i].rx_id, receivers_staging.entries[i].rx_id,
                   ESB_HID_DEVICE_ID_LEN);
            profiles[i].slot = receivers_staging.entries[i].slot;
        }
    } else
#endif
    {
        memset(profiles[0].rx_id, 0, ESB_HID_DEVICE_ID_LEN);
#if IS_ENABLED(CONFIG_SETTINGS)
        profiles[0].slot = slot_override_valid ? slot_override : CONFIG_ZMK_ESB_HID_SLOT;
#else
        profiles[0].slot = CONFIG_ZMK_ESB_HID_SLOT;
#endif
        profile_count = 1;
    }
    active_profile = 0;
    own_slot = profiles[0].slot;
    tx_pipe = own_slot;
    LOG_INF("esb_hid: TX slot %d, device name \"%s\" (%d receiver profile(s))", own_slot,
            CONFIG_ZMK_ESB_HID_DEVICE_NAME, profile_count);
}

/* Own device name as advertised on the wire: the Kconfig string
 * truncated to ESB_HID_NAME_MAX chars. Written NUL-terminated into
 * buf (ESB_HID_NAME_MAX + 1 bytes); returns the name length. */
static size_t own_name(char *buf) {
    size_t len = strlen(CONFIG_ZMK_ESB_HID_DEVICE_NAME);

    if (len > ESB_HID_NAME_MAX) {
        len = ESB_HID_NAME_MAX;
    }
    memcpy(buf, CONFIG_ZMK_ESB_HID_DEVICE_NAME, len);
    buf[len] = '\0';
    return len;
}

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

/* Pairing state (defined further below, next to the state machine). */
static bool pairing_active;
static void pairing_handle_assign(const uint8_t *body, size_t len);

/* Receiver hunting state machine (defined further below, before
 * send_frame). Pairing takes over the radio, so it cancels any
 * hunt in progress. */
static void hunt_cancel(void);

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
/* Studio helpers (defined in the Studio section below). */
static void studio_note_activity(void);
static void studio_handle_hint(uint8_t flags);
#endif

/* Drain any pending ACK payloads. Called from the TX thread (never
 * from the ESB event handler, whose context is not safe for this).
 * Routes Studio fragments/hints to the Studio glue and pairing
 * assignments to the pairing state machine. Runs with or without
 * Studio: pairing must work on non-Studio builds too. */
static void drain_ack_payloads(void) {
    struct esb_payload payload;

    while (esb_read_rx_payload(&payload) == 0) {
        if (payload.length <= ESB_HID_FRAME_HDR_LEN) {
            continue;
        }
        const uint8_t *body = &payload.data[ESB_HID_FRAME_HDR_LEN];
        size_t body_len = payload.length - ESB_HID_FRAME_HDR_LEN;

        switch (payload.data[0]) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
        case ESB_HID_TYPE_STUDIO:
            studio_note_activity();
            esb_hid_studio_on_rx_bytes(body, body_len);
            break;
        case ESB_HID_TYPE_STUDIO_HINT:
            studio_handle_hint(body[0]);
            break;
#endif
        case ESB_HID_TYPE_PAIR_ASSIGN:
            if (pairing_active) {
                pairing_handle_assign(body, body_len);
            }
            break;
        default:
            LOG_DBG("Ignoring unexpected ACK payload type %d (len %d)", payload.data[0],
                    payload.length);
            break;
        }
    }
}

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
/* ------------------------------------------------------------------ */
/* Studio over ESB (downlink via ACK payloads + polling)               */

/* Detection-based poll pacing. The keyboard never polls on a timer
 * while no Studio session exists: the receiver piggybacks a
 * ESB_HID_TYPE_STUDIO_HINT on the ACK of whatever traffic the keyboard
 * sends anyway (key/mouse reports, the 5 s status heartbeat), and
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

static void studio_maybe_poll(void) {
    int64_t now = k_uptime_get();
    bool active = (now - studio_last_activity_ms) < ESB_HID_STUDIO_ACTIVE_WINDOW_MS ||
                  now < studio_probe_until_ms;

    if (!active) {
        /* No session detected: stay silent. The next hint arrives on
         * the ACK of regular traffic (keypress or 5 s heartbeat). */
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

#define ESB_HID_STATUS_HEARTBEAT K_SECONDS(5)

/* The first few status frames after boot carry ESB_HID_STATUS_FLAG_BOOT
 * so the receiver can edge-detect this reboot and flush the Studio
 * session state it holds for the previous boot. Snapshots fire on
 * layer/battery events plus the 5 s heartbeat (the first at link
 * start +1 s); 3 flagged sends are redundancy against loss, and every
 * status frame is itself retried by send_frame(). */
static uint8_t status_boot_countdown = 3;

/* Encode the currently selected HID output transport into the status
 * flags field. Status frames are sent over ESB no matter which output
 * is selected, so without this the receiver cannot tell that e.g. the
 * user toggled `&out` to USB and the keystrokes now bypass it. */
static uint8_t status_output_flags(void) {
    uint8_t output;

    switch (zmk_endpoint_get_selected().transport) {
    case ZMK_TRANSPORT_USB:
        output = ESB_HID_STATUS_OUTPUT_USB;
        break;
    case ZMK_TRANSPORT_BLE:
        output = ESB_HID_STATUS_OUTPUT_BLE;
        break;
    case ZMK_TRANSPORT_ESB:
        output = ESB_HID_STATUS_OUTPUT_ESB;
        break;
    default:
        output = ESB_HID_STATUS_OUTPUT_NONE;
        break;
    }
    return (uint8_t)(output << ESB_HID_STATUS_FLAG_OUTPUT_SHIFT);
}

static void send_status_snapshot(void) {
    struct esb_hid_status_body body = {
        .battery = ESB_HID_BATTERY_UNKNOWN,
        .layer = (uint8_t)zmk_keymap_highest_layer_active(),
        .flags = (uint8_t)(((status_boot_countdown > 0) ? ESB_HID_STATUS_FLAG_BOOT : 0) |
                           status_output_flags()),
        .proto_version = ESB_HID_PROTOCOL_VERSION,
    };

    if (status_boot_countdown > 0) {
        status_boot_countdown--;
    }

#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
    body.battery = zmk_battery_state_of_charge();
#endif

    /* Status v2: append the [name_len][name] trailer after the fixed
     * 4-byte v1 body. Status v3: append the [layer_name_len]
     * [layer_name] trailer after the v2 name trailer. Layer names
     * are ASCII display-names, so truncating to
     * ESB_HID_LAYER_NAME_MAX bytes is fine.
     * Frame buffer sizing: 4 + 1+16 + 1+16 = 38 body bytes max. */
    uint8_t frame_body[sizeof(body) + 1 + ESB_HID_NAME_MAX + 1 + ESB_HID_LAYER_NAME_MAX];
    char name[ESB_HID_NAME_MAX + 1];
    size_t name_len = own_name(name);

    const char *layer_name_str = zmk_keymap_layer_name(body.layer);
    size_t layer_name_len = (layer_name_str != NULL) ? strlen(layer_name_str) : 0;

    if (layer_name_len > ESB_HID_LAYER_NAME_MAX) {
        layer_name_len = ESB_HID_LAYER_NAME_MAX;
    }

    memcpy(frame_body, &body, sizeof(body));
    frame_body[sizeof(body)] = (uint8_t)name_len;
    memcpy(&frame_body[sizeof(body) + 1], name, name_len);
    frame_body[sizeof(body) + 1 + name_len] = (uint8_t)layer_name_len;
    if (layer_name_len > 0) {
        memcpy(&frame_body[sizeof(body) + 1 + name_len + 1], layer_name_str, layer_name_len);
    }

    enqueue_report(ESB_HID_TYPE_STATUS, frame_body,
                   sizeof(body) + 1 + name_len + 1 + layer_name_len);
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
    if (as_zmk_endpoint_changed(eh) != NULL) {
        /* The selected output transport changed (e.g. `&out OUT_TOG`):
         * push a snapshot now so the receiver's display follows the
         * toggle immediately instead of at the next heartbeat. */
        send_status_snapshot();
        return 0;
    }
    return 0;
}

ZMK_LISTENER(esb_hid_tx, esb_hid_tx_listener);
ZMK_SUBSCRIPTION(esb_hid_tx, zmk_layer_state_changed);
ZMK_SUBSCRIPTION(esb_hid_tx, zmk_endpoint_changed);
#if IS_ENABLED(CONFIG_ZMK_BATTERY_REPORTING)
ZMK_SUBSCRIPTION(esb_hid_tx, zmk_battery_state_changed);
#endif

/* ------------------------------------------------------------------ */
/* Pairing (step 2)                                                    */

/* Pairing window: PAIR_REQUEST every ~500 ms for up to 30 s. */
#define ESB_HID_PAIRING_DURATION_MS 30000
#define ESB_HID_PAIRING_REQUEST_PERIOD_MS 500

static int64_t pairing_deadline_ms;

/* Hardware device ID (Zephyr hwinfo, nRF FICR DEVICEID), fetched
 * lazily on first use and cached. device_id_len is the number of
 * valid bytes (0 when the fetch failed / has not succeeded). */
static uint8_t device_id[ESB_HID_DEVICE_ID_LEN];
static uint8_t device_id_len;
static bool device_id_fetched;

static uint8_t fetch_device_id(void) {
    if (!device_id_fetched) {
        device_id_fetched = true;
        ssize_t ret = hwinfo_get_device_id(device_id, sizeof(device_id));

        if (ret < 1) {
            device_id_len = 0;
        } else {
            device_id_len = (uint8_t)((size_t)ret > ESB_HID_DEVICE_ID_LEN
                                          ? ESB_HID_DEVICE_ID_LEN
                                          : (size_t)ret);
        }
    }
    return device_id_len;
}

static void pairing_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(pairing_work, pairing_work_handler);

static void pairing_exit(void) {
    pairing_active = false;
    tx_pipe = own_slot;
    k_work_cancel_delayable(&pairing_work);
}

static void pairing_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    if (!pairing_active) {
        return;
    }
    if (k_uptime_get() >= pairing_deadline_ms) {
        pairing_exit();
        LOG_INF("Pairing: timed out after 30 s, back to slot %d", own_slot);
        return;
    }

    char name[ESB_HID_NAME_MAX + 1];
    size_t name_len = own_name(name);
    uint8_t id_len = fetch_device_id();
    uint8_t body[3 + ESB_HID_DEVICE_ID_LEN + ESB_HID_NAME_MAX];
    size_t body_len;

    if (id_len < 1) {
        /* No hardware ID available: fall back to the proto 1 body. */
        body[0] = 1;
        body[1] = (uint8_t)name_len;
        memcpy(&body[2], name, name_len);
        body_len = 2 + name_len;
    } else {
        body[0] = ESB_HID_PAIR_PROTO_VERSION;
        body[1] = id_len;
        memcpy(&body[2], device_id, id_len);
        body[2 + id_len] = (uint8_t)name_len;
        memcpy(&body[3 + id_len], name, name_len);
        body_len = 3 + id_len + name_len;
    }
    enqueue_report(ESB_HID_TYPE_PAIR_REQUEST, body, body_len);

    k_work_reschedule(&pairing_work, K_MSEC(ESB_HID_PAIRING_REQUEST_PERIOD_MS));
}

static void pairing_handle_assign(const uint8_t *body, size_t len) {
    if (len < 2) {
        return;
    }

    uint8_t slot = body[0];

    if (slot == ESB_HID_PAIR_SLOT_NOT_YET) {
        /* Sentinel: the receiver sees us but has not confirmed a
         * candidate yet. Keep requesting. */
        return;
    }
    if (slot >= ESB_HID_SLOT_COUNT) {
        LOG_WRN("Pairing: ignoring assignment with invalid slot %d", slot);
        return;
    }

    uint8_t name_len = body[1];

    if (name_len > ESB_HID_NAME_MAX || len < (size_t)(2 + name_len)) {
        LOG_WRN("Pairing: ignoring malformed assignment (len %d, name_len %d)", (int)len,
                name_len);
        return;
    }

    char mine[ESB_HID_NAME_MAX + 1];
    size_t my_len = own_name(mine);

    if (name_len != my_len || memcmp(&body[2], mine, my_len) != 0) {
        LOG_WRN("Pairing: assignment name \"%.*s\" does not match ours (\"%s\"), ignoring",
                name_len, (const char *)&body[2], mine);
        return;
    }

    /* Optional trailer: the assigning receiver's own device ID,
     * present iff exactly ESB_HID_DEVICE_ID_LEN bytes follow the
     * name (see esb_hid_frame.h). Older receivers append nothing and
     * the profile is recorded with an unknown identity. */
    const uint8_t *rx_id = NULL;

    if (len - (2 + name_len) == ESB_HID_DEVICE_ID_LEN) {
        rx_id = &body[2 + name_len];
    }

    profiles_upsert(rx_id, slot);
    pairing_exit();
    if (rx_id != NULL) {
        LOG_INF("Pairing: assigned slot %d for \"%s\" (receiver ID known, %d profile(s)), saved "
                "to settings",
                slot, mine, profile_count);
    } else {
        LOG_INF("Pairing: assigned slot %d for \"%s\" (receiver ID unknown), saved to settings",
                slot, mine);
    }
}

int esb_hid_tx_start_pairing(void) {
    if (!link_ready) {
        return -EAGAIN;
    }
    if (pairing_active) {
        return -EBUSY;
    }

    pairing_active = true;
    hunt_cancel();
    pairing_deadline_ms = k_uptime_get() + ESB_HID_PAIRING_DURATION_MS;
    tx_pipe = ESB_HID_PAIRING_PIPE;
    k_work_reschedule(&pairing_work, K_NO_WAIT);

    char name[ESB_HID_NAME_MAX + 1];

    own_name(name);
    uint8_t id_len = fetch_device_id();

    if (id_len > 0) {
        char id_hex[ESB_HID_DEVICE_ID_LEN * 2 + 1];

        for (int i = 0; i < id_len; i++) {
            id_hex[i * 2] = "0123456789abcdef"[device_id[i] >> 4];
            id_hex[i * 2 + 1] = "0123456789abcdef"[device_id[i] & 0x0F];
        }
        id_hex[id_len * 2] = '\0';
        LOG_INF("Pairing: started (name \"%s\", device ID %s), requesting on pipe %d for 30 s",
                name, id_hex, ESB_HID_PAIRING_PIPE);
    } else {
        LOG_INF("Pairing: started (name \"%s\"), requesting on pipe %d for 30 s", name,
                ESB_HID_PAIRING_PIPE);
    }
    return 0;
}

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

/* ------------------------------------------------------------------ */
/* Link monitor and receiver hunting (multi-receiver)                  */

/* Presence is judged by status frames only: they flow on the 5 s
 * heartbeat even while the user is idle, whereas key/mouse frames
 * can fail in bursts for unrelated reasons (queue pressure, one
 * noisy moment) and must not trigger a receiver switch on their
 * own. A receiver is declared gone after this many consecutive
 * failed status sends, which also bounds the detection time to
 * roughly 15 s -- the same order as the receiver's own link-lost
 * timeout (ESB_HID_LINK_LOST_TIMEOUT_MS). */
#define ESB_HID_LINK_FAIL_THRESHOLD 3
/* While hunting, one pass probes every profile once; passes repeat
 * at most this often, with a short gap between two candidates, so
 * having no known receiver around costs a handful of tiny frames
 * every few seconds. */
#define ESB_HID_HUNT_CYCLE_MS 3000
#define ESB_HID_HUNT_PROBE_GAP_MS 200

static bool link_up; /* the active profile's receiver is ACKing */
static uint8_t status_fail_streak;
static bool hunting;
static uint8_t hunt_candidate;  /* profile index being probed */
static uint8_t hunt_pass_start; /* first candidate of the current pass */
static bool hunt_pass_done;     /* rotation wrapped; waiting out the cycle */
static int64_t hunt_next_probe_ms;
static int64_t hunt_cycle_end_ms;

static void hunt_set_candidate(uint8_t idx) {
    hunt_candidate = idx;
    tx_pipe = profiles[idx].slot;
}

/* Start a hunt pass over all profiles. include_active (boot: nothing
 * has been confirmed yet) starts the rotation at the active profile;
 * on link loss the just-lost profile is probed last instead. */
static void hunt_start(bool include_active) {
    if (profile_count < 2 || pairing_active) {
        return;
    }
    int64_t now = k_uptime_get();

    hunting = true;
    hunt_pass_done = false;
    hunt_pass_start = include_active ? active_profile
                                     : (uint8_t)((active_profile + 1) % profile_count);
    hunt_cycle_end_ms = now + ESB_HID_HUNT_CYCLE_MS;
    hunt_next_probe_ms = 0; /* first probe immediately */
    hunt_set_candidate(hunt_pass_start);
    LOG_INF("ESB link: hunting for a known receiver (%d profiles)", profile_count);
}

static void hunt_cancel(void) {
    hunting = false;
    status_fail_streak = 0;
}

/* The hunted candidate answered: make its profile active (moved to
 * the front, so it is probed first next time) and push the current
 * absolute state to it -- it may be a different receiver than the
 * one that saw the earlier reports. */
static void hunt_confirm(void) {
    uint8_t found = hunt_candidate;
    bool reordered = (found != 0);

    hunting = false;
    link_up = true;
    status_fail_streak = 0;
    profiles_move_to_front(found);
    active_profile = 0;
    own_slot = profiles[0].slot;
    tx_pipe = own_slot;
    if (reordered) {
        profiles_save();
    }
    resync_keyboard = true;
    resync_consumer = true;
    LOG_INF("ESB link: receiver found (slot %d), link up", own_slot);
    send_status_snapshot();
}

/* Advance the rotation after an unanswered status frame. */
static void hunt_note_miss(int64_t now) {
    uint8_t next = (uint8_t)((hunt_candidate + 1) % profile_count);

    if (next == hunt_pass_start) {
        /* Rotation wrapped: the pass is over. Wait out the rest of
         * the cycle before probing again. */
        hunt_pass_done = true;
        hunt_next_probe_ms = hunt_cycle_end_ms;
        return;
    }
    hunt_set_candidate(next);
    hunt_next_probe_ms = now + ESB_HID_HUNT_PROBE_GAP_MS;
}

/* Feed one finished send into the link state machine. Called from
 * the TX thread after send_frame(): true means the receiver ACKed. */
static void link_monitor(uint8_t type, bool ok) {
    if (ok) {
        status_fail_streak = 0;
        if (hunting) {
            hunt_confirm();
        } else {
            link_up = true;
        }
        return;
    }

    if (type != ESB_HID_TYPE_STATUS) {
        return;
    }

    if (hunting) {
        if (!hunt_pass_done) {
            hunt_note_miss(k_uptime_get());
        }
        return;
    }

    if (++status_fail_streak >= ESB_HID_LINK_FAIL_THRESHOLD) {
        status_fail_streak = 0;
        if (link_up) {
            LOG_INF("ESB link: receiver stopped answering, link down");
        }
        link_up = false;
        hunt_start(false);
    }
}

/* Drive the hunt from the TX thread: send the status snapshot as
 * the probe for the current candidate and pace the passes. The
 * probe's result comes back through link_monitor(). */
static void hunt_tick(void) {
    if (!hunting || pairing_active) {
        return;
    }
    int64_t now = k_uptime_get();

    if (now < hunt_next_probe_ms) {
        return;
    }
    if (hunt_pass_done) {
        /* A new cycle begins: start the next pass at the front
         * profile (the most recently confirmed one). */
        hunt_pass_done = false;
        hunt_pass_start = 0;
        hunt_cycle_end_ms = now + ESB_HID_HUNT_CYCLE_MS;
        hunt_set_candidate(0);
    }
    send_status_snapshot();
    hunt_next_probe_ms = now + ESB_HID_HUNT_PROBE_GAP_MS;
}

static bool send_frame(const struct esb_hid_frame *frame) {
    struct esb_payload payload = {
        .length = frame->len + ESB_HID_FRAME_HDR_LEN,
        .pipe = tx_pipe,
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
        if (err == -ENOMEM) {
            /* Defensive: with the pop on failure below, the TX FIFO
             * should never fill up. If it still does, drop the queued
             * payloads instead of wedging the link for good: every
             * frame carries absolute state, so the current frame
             * supersedes anything queued. */
            while (esb_pop_tx() == 0) {
            }
            err = esb_write_payload(&payload);
        }
        if (err) {
            LOG_WRN("esb_write_payload failed: %d", err);
            return false;
        }
        if (k_sem_take(&tx_done_sem, K_MSEC(50)) == 0 && tx_last_ok) {
            return true;
        }
        /* The attempt failed (or timed out). The ESB library removes a
         * payload from the TX FIFO only on TX success; a failed one
         * stays at the FIFO head and every retry queues behind it, so
         * with the receiver away the FIFO fills up and all later
         * writes return -ENOMEM even after the receiver is back --
         * the link then only recovers by rebooting the keyboard.
         * Drop the failed head payload: the retry below (or the next
         * frame) carries newer state anyway. On a bare timeout the
         * pop may return -EBUSY while the radio is still busy; the
         * failure event that follows is handled by the next attempt. */
        esb_pop_tx();
    }
    LOG_WRN("ESB TX failed (type %d)", frame->type);
    return false;
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
            /* While pairing, normal HID/status/studio traffic is
             * paused: only PAIR_REQUEST frames leave on the pairing
             * pipe. Everything else is dropped (state reports are
             * re-sent after pairing via the resync mechanism / the
             * next change event anyway). */
            if (!pairing_active || frame.type == ESB_HID_TYPE_PAIR_REQUEST) {
                bool sent_ok = send_frame(&frame);

                link_monitor(frame.type, sent_ok);
            }
            /* A successful send may have carried an ACK payload. */
            drain_ack_payloads();
            hunt_tick();
            continue;
        }

        drain_ack_payloads();
        hunt_tick();
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO)
        if (link_ready && !pairing_active) {
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
    /* Resolve the slot (settings override or Kconfig default) before
     * the first frame goes out; harmless to redo on retries. */
    resolve_own_slot();
    int err = esb_hid_link_start(ESB_MODE_PTX, esb_hid_tx_event_handler);

    diag_note_init(err);
    if (err) {
        LOG_ERR("ESB link start failed: %d (retry in 5s)", err);
        k_work_reschedule(&link_retry_work, K_SECONDS(5));
        return;
    }

    link_ready = true;
    /* With several known receivers, find out which one is actually
     * around before settling: probe them in rotation, starting from
     * the most recently confirmed one. A single profile skips this
     * entirely (legacy behaviour). */
    hunt_start(true);
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
