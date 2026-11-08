/*
 * Internal glue for the Deck notification channel
 * (CONFIG_ZMK_ESB_HID_NOTIFY_RX): a dedicated USB CDC ACM port
 * (devicetree node "deck_notify_cdc") carrying framed notifications
 * between a PC-side notifier program and the receiver.
 * Protocol: deck-notifications-design.md section 3.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* Wire framing (design 3.1): "DN" magic, protocol version, type,
 * little-endian payload length, payload. No CRC — the CDC link
 * itself is treated as reliable. */
#define ESB_HID_NOTIFY_MAGIC_0 'D'
#define ESB_HID_NOTIFY_MAGIC_1 'N'
#define ESB_HID_NOTIFY_PROTO_VER 1
#define ESB_HID_NOTIFY_HDR_LEN 6
#define ESB_HID_NOTIFY_PAYLOAD_MAX 248

/* Message types (design 3.2). */
#define ESB_HID_NOTIFY_TYPE_HELLO 0x01     /* PC -> Deck */
#define ESB_HID_NOTIFY_TYPE_NOTIFY 0x02    /* PC -> Deck */
#define ESB_HID_NOTIFY_TYPE_RESOLVE 0x03   /* PC -> Deck */
#define ESB_HID_NOTIFY_TYPE_HELLO_ACK 0x81 /* Deck -> PC */
#define ESB_HID_NOTIFY_TYPE_ACK 0x82       /* Deck -> PC */
#define ESB_HID_NOTIFY_TYPE_ANSWER 0x83    /* Deck -> PC */

/* RX side: frame and send one message to the PC notifier over the
 * notify CDC port (implemented in esb_hid_notify_rx.c, only present
 * with CONFIG_ZMK_ESB_HID_NOTIFY_RX). Returns 0 on success, a
 * negative errno otherwise (-EINVAL when len exceeds the payload
 * maximum, -ENODEV when the shield provided no CDC node). */
int esb_hid_notify_send(uint8_t type, const uint8_t *payload, size_t len);
