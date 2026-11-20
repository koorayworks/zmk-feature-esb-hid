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
/* Pairing request (TX->RX, pairing pipe 7 only).
 * Proto v1 body:
 *   [0] pairing protocol version (=1)
 *   [1] name_len
 *   [2..] device name bytes (CONFIG_ZMK_ESB_HID_DEVICE_NAME, <=16)
 * Proto v2 body:
 *   [0] pairing protocol version (=2, ESB_HID_PAIR_PROTO_VERSION)
 *   [1] id_len (<= ESB_HID_DEVICE_ID_LEN)
 *   [2..2+id_len) hardware device ID bytes
 *   [2+id_len] name_len
 *   [3+id_len..] device name bytes (<=16)
 * Receivers accept both v1 (candidate id_len = 0) and v2; other
 * versions are ignored.
 * Sent repeatedly (~every 500 ms) by a keyboard in pairing mode until
 * it adopts an assigned slot or the 30 s pairing window lapses. */
#define ESB_HID_TYPE_PAIR_REQUEST 7
/* Pairing assignment (RX->TX, ACK payloads on the pairing pipe only,
 * standard frame format). Body:
 *   [0] slot 0..3, or ESB_HID_PAIR_SLOT_NOT_YET while the receiver has
 *       not confirmed a candidate yet
 *   [1] name_len
 *   [2..2+name_len) candidate name captured at confirm time
 *   [2+name_len..2+name_len+8) OPTIONAL trailer: the receiver's own
 *       hardware device ID (ESB_HID_DEVICE_ID_LEN bytes), present iff
 *       the body is exactly 2 + name_len + ESB_HID_DEVICE_ID_LEN long.
 *       Multi-receiver keyboards (ZMK_ESB_HID_MULTI_RECEIVER) store it
 *       in their receiver profile list so they can tell their paired
 *       receivers apart. Receivers without a readable ID omit it, and
 *       keyboards with older firmware simply ignore the trailer.
 * The keyboard adopts the slot ONLY if the name equals its own device
 * name, so a stray assignment can never renumber the wrong device. */
#define ESB_HID_TYPE_PAIR_ASSIGN 8
/* Slot identity (RX->TX, ACK payloads only, never a regular uplink
 * frame). Body:
 *   [0] flags: bit0 (ESB_HID_SLOT_IDENTITY_REGISTERED) set when the
 *       pipe this payload rides on belongs to a registered slot
 *   [1..9) the hardware device ID registered at that slot
 *       (ESB_HID_DEVICE_ID_LEN bytes; all zero for legacy slots whose
 *       registration predates pairing proto v2 — the keyboard must
 *       treat that as "cannot judge", not as a mismatch)
 * The receiver periodically tells each connected keyboard which device
 * it believes is registered on that pipe, so a keyboard holding a
 * stale receiver profile (receiver-side registry was deleted, moved
 * or migrated without it) can detect the desync and drop the profile
 * instead of silently talking to a slot that no longer maps to it.
 * Keyboards with older firmware ignore this type; receivers with
 * older firmware never send it, and its ABSENCE must never be treated
 * as a mismatch. See DESIGN.md "SLOT_IDENTITY" (2026-11-15). */
#define ESB_HID_TYPE_SLOT_IDENTITY 9
/* Host state (RX->TX, ACK payloads only, never a regular uplink
 * frame). Body is a single flags byte; bit0
 * (ESB_HID_HOST_STATE_ASLEEP) is set while the USB host has the bus
 * suspended (host asleep). The receiver piggybacks it on ACKs the
 * same way as SLOT_IDENTITY (periodic + on change) so keyboards can
 * stretch their status heartbeat while nobody can be typing anyway;
 * any local key/mouse activity ends the quiet state immediately, and
 * the key frame that follows is what wakes the host (USB remote
 * wakeup via the receiver). Keyboards with older firmware ignore
 * this type in their ACK-payload switch's default case; receivers
 * with older firmware never send it. See DESIGN.md power saving
 * (2026-11-20). */
#define ESB_HID_TYPE_HOST_STATE 10
#define ESB_HID_TYPE_COUNT 11

#define ESB_HID_SLOT_IDENTITY_REGISTERED 0x01

/* Host state flags (body[0] of ESB_HID_TYPE_HOST_STATE). */
#define ESB_HID_HOST_STATE_ASLEEP 0x01

#define ESB_HID_PAIR_PROTO_VERSION 2
#define ESB_HID_PAIR_SLOT_NOT_YET 0xFF

/* Length of the hardware device ID carried in pairing proto v2.
 * Source is Zephyr hwinfo (nRF FICR DEVICEID = exactly 8 bytes). */
#define ESB_HID_DEVICE_ID_LEN 8

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
/* Status protocol v2: proto_version = 2 and a name trailer appended
 * after the fixed 4-byte body: [name_len][name bytes], name =
 * CONFIG_ZMK_ESB_HID_DEVICE_NAME truncated to ESB_HID_NAME_MAX chars.
 * Receivers parse the trailer only when proto_version >= 2 and the
 * frame is long enough; v1 frames (exactly 4 body bytes) are still
 * accepted with an empty name.
 * Status protocol v3: proto_version = 3 and a layer-name trailer
 * appended after the v2 name trailer:
 * [layer_name_len][layer_name bytes], layer name truncated to
 * ESB_HID_LAYER_NAME_MAX bytes. v1/v2 frames are still accepted,
 * with an empty layer name.
 * Status protocol v4: proto_version = 4 and a 1-byte peer battery
 * trailer appended after the v3 layer-name trailer: the state of
 * charge of the keyboard's split peer (the other half), as last
 * reported to the central, or ESB_HID_BATTERY_UNKNOWN when the
 * keyboard is not split or the peer level is unknown. v1-v3 frames
 * are still accepted, with an unknown peer battery.
 * Status protocol v5: proto_version = 5 and a 1-byte heartbeat
 * trailer appended after the v4 peer battery trailer: the status
 * heartbeat period (seconds) the keyboard is currently running —
 * it adapts to activity and to the host sleep state, and the
 * receiver derives its link-lost timeout from the declared value
 * instead of assuming 5 s. v1-v4 frames are still accepted, with
 * no declaration (the receiver falls back to the fixed timeout). */
#define ESB_HID_PROTOCOL_VERSION 5

struct esb_hid_status_body {
    uint8_t battery;       /* 0-100 %, ESB_HID_BATTERY_UNKNOWN if unavailable */
    uint8_t layer;         /* highest active layer index */
    uint8_t flags;         /* ESB_HID_STATUS_FLAG_* */
    uint8_t proto_version; /* ESB_HID_PROTOCOL_VERSION */
};
