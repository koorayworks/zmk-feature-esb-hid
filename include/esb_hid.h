/*
 * Public API of the HID over ESB module (step 2: multiple devices,
 * pairing, receiver-side slot management).
 *
 * The receiver half (CONFIG_ZMK_ESB_HID_RECEIVER) exposes the slot
 * registry and the pairing window state machine to UI code (e.g. the
 * receiver's status screen). The transmitter half exposes the entry
 * point behind the &esb_pair keymap behavior.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <esb_hid_status.h>

#define ESB_HID_SLOT_COUNT 4
#ifndef ESB_HID_NAME_MAX
#define ESB_HID_NAME_MAX 16
#endif
#define ESB_HID_PAIRING_PIPE 7

#if defined(CONFIG_ZMK_ESB_HID_RECEIVER)

struct esb_hid_slot_info {
    bool used;
    char name[ESB_HID_NAME_MAX + 1];
    char layer_name[ESB_HID_LAYER_NAME_MAX + 1];
    bool status_valid;
    uint8_t battery; /* 0-100, 0xFF = unknown */
    uint8_t layer;
    uint8_t output; /* ESB_HID_STATUS_OUTPUT_* */
    bool link_up;
    int64_t last_seen_age_ms; /* -1 = never seen */
};

uint8_t esb_hid_rx_get_active_slot(void);
int esb_hid_rx_set_active_slot(uint8_t slot); /* -EINVAL if slot>=4 or slot unused */
int esb_hid_rx_get_slot_info(uint8_t slot, struct esb_hid_slot_info *out);
int esb_hid_rx_delete_slot(uint8_t slot); /* clears registry entry + saves */

enum esb_hid_pairing_state {
    ESB_HID_PAIRING_IDLE = 0,
    ESB_HID_PAIRING_SEARCHING,
    ESB_HID_PAIRING_CANDIDATE,
    ESB_HID_PAIRING_ASSIGNING,
    ESB_HID_PAIRING_DONE,
};

struct esb_hid_pairing_status {
    enum esb_hid_pairing_state state;
    uint8_t target_slot;
    char candidate_name[ESB_HID_NAME_MAX + 1];
    int32_t remaining_ms; /* window time left; 0 when IDLE */
};

int esb_hid_pairing_open(uint8_t target_slot);
int esb_hid_pairing_get_status(struct esb_hid_pairing_status *out);
int esb_hid_pairing_confirm(void);
void esb_hid_pairing_cancel(void);

#endif /* CONFIG_ZMK_ESB_HID_RECEIVER */

#if defined(CONFIG_ZMK_ESB_HID)
int esb_hid_tx_start_pairing(void); /* called by the &esb_pair behavior; 0 if started */
#endif
