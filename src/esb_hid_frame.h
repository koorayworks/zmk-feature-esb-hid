/*
 * HID over ESB wire format (step 1).
 *
 * One ESB packet carries exactly one HID report:
 *   [0] type: 0 = keyboard, 1 = consumer, 2 = mouse
 *   [1] seq:  per-type monotonically increasing sequence number.
 *           Keyboard/consumer reports are absolute state; the receiver
 *           drops duplicates by seq. Mouse reports are relative; seq is
 *           only for loss observation in step 1.
 *   [2..]   HID report body, byte-for-byte the ZMK report body that the
 *           BLE HOG transport would send.
 */

#pragma once

#define ESB_HID_TYPE_KEYBOARD 0
#define ESB_HID_TYPE_CONSUMER 1
#define ESB_HID_TYPE_MOUSE 2
#define ESB_HID_TYPE_STATUS 3
/* Studio RPC stream fragments (bidirectional). Body is a raw chunk of
 * the framed ZMK Studio RPC byte stream (SOF/EOF/ESC framing is part of
 * the stream, so fragments may split at any byte). Uplink (TX->RX)
 * frames travel like other reports; downlink (RX->TX) fragments travel
 * as ESB ACK payloads with this same frame format. */
#define ESB_HID_TYPE_STUDIO 4
/* Empty poll frame (TX->RX, no body). Exists only to give the receiver
 * a packet to attach a pending ACK payload to, so downlink Studio data
 * can be pulled without waiting for user input. */
#define ESB_HID_TYPE_POLL 5
#define ESB_HID_TYPE_COUNT 6

#define ESB_HID_FRAME_HDR_LEN 2

/* Status frame body (type ESB_HID_TYPE_STATUS).
 * Sent by the keyboard on battery/layer changes and as a periodic
 * heartbeat. State semantics: latest wins, duplicates dropped by seq.
 * Intended for the receiver's display (peer battery / layer). */
#define ESB_HID_BATTERY_UNKNOWN 0xFF
#define ESB_HID_PROTOCOL_VERSION 1

struct esb_hid_status_body {
    uint8_t battery;       /* 0-100 %, ESB_HID_BATTERY_UNKNOWN if unavailable */
    uint8_t layer;         /* highest active layer index */
    uint8_t flags;         /* reserved, send 0 */
    uint8_t proto_version; /* ESB_HID_PROTOCOL_VERSION */
};
