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
/* Studio session hint (RX->TX, ACK payloads only, never a regular
 * uplink frame). Body is a single flags byte; lets the receiver tell
 * the keyboard "a Studio client is connected / downlink data is
 * waiting" on the ACK of whatever traffic the keyboard already sends,
 * so the keyboard only starts polling once a session actually exists.
 * See esb_hid_studio_rx.c (producer) and esb_hid_tx.c (consumer). */
#define ESB_HID_TYPE_STUDIO_HINT 6
#define ESB_HID_TYPE_COUNT 7

/* Studio hint flags (body[0] of ESB_HID_TYPE_STUDIO_HINT). */
#define ESB_HID_STUDIO_HINT_CONNECTED 0x01
#define ESB_HID_STUDIO_HINT_DOWNLINK_PENDING 0x02

/* Status frame flags (esb_hid_status_body.flags). */
/* Set by the keyboard in the first few status frames after boot, so
 * the receiver can edge-detect a keyboard reboot and drop the stale
 * Studio session state it holds for the previous boot (queued ACK
 * payloads, pending downlink fragments). */
#define ESB_HID_STATUS_FLAG_BOOT 0x01

/* Selected HID output transport of the keyboard, in flags bits 2:1.
 * Status frames travel over ESB regardless of the selected output, so
 * this is how the receiver (e.g. the Deck display) can tell whether
 * the keyboard's keystrokes currently reach the host through this
 * receiver (ESB) or directly (USB cable / BLE). Keyboards with older
 * firmware send 0 = ESB, which is correct for those ESB-only builds.
 * Receivers with older firmware simply ignore these bits. */
#define ESB_HID_STATUS_FLAG_OUTPUT_SHIFT 1
#define ESB_HID_STATUS_FLAG_OUTPUT_MASK (0x03 << ESB_HID_STATUS_FLAG_OUTPUT_SHIFT)
/* Field values: ESB_HID_STATUS_OUTPUT_* in <esb_hid_status.h> (kept in
 * the public header because display code compares against them). */

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
    uint8_t flags;         /* ESB_HID_STATUS_FLAG_* */
    uint8_t proto_version; /* ESB_HID_PROTOCOL_VERSION */
};
