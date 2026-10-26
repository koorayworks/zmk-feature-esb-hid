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

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <string.h>

#include <esb.h>
#include <zmk/esb_hid.h>
#include <zmk/hid.h>

#include "esb_hid_frame.h"
#include "esb_hid_link.h"

LOG_MODULE_DECLARE(esb_hid);

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

static int enqueue_report(uint8_t type, const uint8_t *body, size_t len) {
    if (len + ESB_HID_FRAME_HDR_LEN > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
        return -EMSGSIZE;
    }

    struct esb_hid_frame frame = {
        .type = type,
        .seq = seqs[type]++,
        .len = (uint8_t)len,
    };
    memcpy(frame.body, body, len);

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

    int attempts = (frame->type == ESB_HID_TYPE_MOUSE) ? 1 : 3;
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

    while (1) {
        if (k_msgq_get(&esb_hid_tx_queue, &frame, K_MSEC(20)) == 0) {
            send_frame(&frame);
            continue;
        }

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

static int esb_hid_tx_init(void) {
    int err = esb_hid_link_start(ESB_MODE_PTX, esb_hid_tx_event_handler);
    if (err) {
        return err;
    }

    link_ready = true;
    return 0;
}

SYS_INIT(esb_hid_tx_init, APPLICATION, 95);
