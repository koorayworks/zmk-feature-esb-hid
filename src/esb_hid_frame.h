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
#define ESB_HID_TYPE_COUNT 3

#define ESB_HID_FRAME_HDR_LEN 2
