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

struct esb_hid_device_status {
    uint8_t battery; /* 0-100 %, 0xFF = unknown */
    uint8_t layer;
    uint8_t flags;
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
