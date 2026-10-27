/*
 * Internal glue between the ESB HID TX/RX cores and the optional
 * ZMK Studio over ESB transport/forwarder (CONFIG_ZMK_ESB_HID_STUDIO /
 * CONFIG_ZMK_ESB_HID_STUDIO_RX).
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

/* TX side: enqueue one Studio stream fragment as a type
 * ESB_HID_TYPE_STUDIO frame on the shared TX queue (implemented in
 * esb_hid_tx.c). Also marks Studio activity for the poll pacing. */
int esb_hid_tx_enqueue_studio(const uint8_t *data, size_t len);

/* TX side: deliver bytes received via a downlink (ACK payload) Studio
 * fragment to the Studio RPC core (implemented in esb_hid_studio_tx.c,
 * only present with CONFIG_ZMK_ESB_HID_STUDIO). */
void esb_hid_studio_on_rx_bytes(const uint8_t *data, size_t len);

/* RX side: forward one uplink Studio fragment body towards the PC over
 * the USB CDC forwarder (implemented in esb_hid_studio_rx.c, only
 * present with CONFIG_ZMK_ESB_HID_STUDIO_RX). */
void esb_hid_studio_fwd_to_pc(const uint8_t *data, size_t len);

/* RX side: note that a link packet was received (an ACK payload slot
 * may just have been consumed), so queued downlink fragments should be
 * (re)loaded into the ESB ACK FIFO. */
void esb_hid_studio_fwd_note_link_event(void);
