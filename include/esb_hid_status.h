/*
 * Receiver-side status store for HID over ESB.
 *
 * The receiver keeps the latest status frame (battery / layer) of each
 * connected keyboard so other components (e.g. a display widget) can
 * read it. Step 1/2: only device index 0 exists.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Values of the output-transport field in the status frame flags
 * (bits 2:1, see esb_hid_frame.h): which transport the keyboard's HID
 * output is currently selected to. Status frames always travel over
 * ESB, so this is the only way to know whether keystrokes reach the
 * host through the receiver or directly. */
#define ESB_HID_STATUS_OUTPUT_ESB 0
#define ESB_HID_STATUS_OUTPUT_USB 1
#define ESB_HID_STATUS_OUTPUT_BLE 2
#define ESB_HID_STATUS_OUTPUT_NONE 3

struct esb_hid_device_status {
    uint8_t battery; /* 0-100 %, 0xFF = unknown */
    uint8_t layer;
    uint8_t flags;
    uint8_t output; /* ESB_HID_STATUS_OUTPUT_* decoded from flags */
    uint8_t proto_version;
    int64_t last_update_ms; /* k_uptime_get() of the last status frame */
    bool valid;
};

/**
 * Copy the latest known status of a device.
 *
 * @return true if a status frame has been received for the device.
 */
bool esb_hid_get_device_status(uint8_t device_index, struct esb_hid_device_status *out);
