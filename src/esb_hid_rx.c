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
#include <zmk/hid.h>
#include <zmk/usb_hid.h>

#include "esb_hid_frame.h"
#include "esb_hid_link.h"

LOG_MODULE_DECLARE(esb_hid);

static K_SEM_DEFINE(rx_sem, 0, K_SEM_MAX_LIMIT);

static uint8_t last_seq[ESB_HID_TYPE_COUNT];
static bool seq_valid[ESB_HID_TYPE_COUNT];

static const uint8_t report_ids[ESB_HID_TYPE_COUNT] = {
    ZMK_HID_REPORT_ID_KEYBOARD,
    ZMK_HID_REPORT_ID_CONSUMER,
    ZMK_HID_REPORT_ID_MOUSE,
};

static void esb_event_handler(const struct esb_evt *evt) {
    if (evt->evt_id == ESB_EVENT_RX_RECEIVED) {
        k_sem_give(&rx_sem);
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

    /* Keyboard/consumer are absolute state: drop exact duplicates
     * (e.g. a re-delivered frame after a lost ACK). Mouse is relative;
     * every frame carries new motion, so forward all of them. */
    if (type != ESB_HID_TYPE_MOUSE) {
        if (seq_valid[type] && seq == last_seq[type]) {
            return;
        }
        seq_valid[type] = true;
        last_seq[type] = seq;
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

static int esb_hid_rx_init(void) {
    return esb_hid_link_start(ESB_MODE_PRX, esb_event_handler);
}

SYS_INIT(esb_hid_rx_init, APPLICATION, 95);
