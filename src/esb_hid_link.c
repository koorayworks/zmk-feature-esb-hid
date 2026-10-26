/*
 * Shared ESB link setup (step 1: fixed addresses, no pairing/encryption).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esb.h>

#include "esb_hid_link.h"

LOG_MODULE_REGISTER(esb_hid);

/* Step 1 fixed link addresses. Both sides use the same values.
 * Step 2 replaces these with per-device provisioned addresses/keys. */
static const uint8_t base_address_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t base_address_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
static const uint8_t pipe_prefixes[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8};

int esb_hid_link_start(enum esb_mode mode, esb_event_handler handler) {
    struct esb_config config = ESB_DEFAULT_CONFIG;

    config.mode = mode;
    config.event_handler = handler;
    config.tx_output_power = 4;
    config.retransmit_delay = 500;
    config.retransmit_count = 7;

    int err = esb_init(&config);
    if (err) {
        LOG_ERR("esb_init failed: %d", err);
        return err;
    }

    err = esb_set_base_address_0(base_address_0);
    if (err) {
        return err;
    }
    err = esb_set_base_address_1(base_address_1);
    if (err) {
        return err;
    }
    err = esb_set_prefixes(pipe_prefixes, ARRAY_SIZE(pipe_prefixes));
    if (err) {
        return err;
    }
    err = esb_enable_pipes(0x01);
    if (err) {
        return err;
    }

    if (mode == ESB_MODE_PRX) {
        err = esb_start_rx();
        if (err) {
            return err;
        }
    }

    LOG_INF("ESB link started (mode %d)", mode);
    return 0;
}
