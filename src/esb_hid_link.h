/*
 * Shared ESB link setup for the HID over ESB module (step 1:
 * fixed addresses, no pairing, no encryption).
 */

#pragma once

#include <esb.h>

/**
 * Initialize the ESB stack with the step 1 fixed link parameters and
 * start receiving when mode is ESB_MODE_PRX.
 *
 * @param mode    ESB_MODE_PTX (keyboard side) or ESB_MODE_PRX (receiver).
 * @param handler ESB event handler (called from the ESB IRQ context;
 *                keep it to semaphore/msgq operations only).
 * @return 0 on success, negative errno from the ESB stack otherwise.
 */
int esb_hid_link_start(enum esb_mode mode, esb_event_handler handler);
