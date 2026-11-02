/*
 * Shared ESB link setup (step 1: fixed addresses, no pairing/encryption).
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <esb.h>

#include "esb_hid_link.h"

LOG_MODULE_REGISTER(esb_hid);

/* Step 2 link addressing. Both sides use the same values.
 *
 * Nordic ESB addressing: pipe 0 = base0 + prefix[0]; pipe 1 =
 * base1 + prefix[1]; pipes 2..7 = base1 + prefix[n]. So with both
 * bases equal, slot s (pipe s, s = 0..3) is addressed by
 * E7E7E7E7 + prefixes[s], and the pairing pipe 7 by
 * E7E7E7E7 + prefixes[7]. Slot 0's address is unchanged from step 1,
 * so a step 1 keyboard and a step 2 receiver still find each other
 * on slot 0. Pipes 4..6 are enabled on neither side (mask below) but
 * keep distinct prefixes in case they are used later. */
static const uint8_t base_address_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t base_address_1[4] = {0xE7, 0xE7, 0xE7, 0xE7};
static const uint8_t pipe_prefixes[8] = {0xE7, 0xE6, 0xE5, 0xE4, 0xC5, 0xC6, 0xC7, 0xA5};

/* Pipes 0..3 (device slots) + pipe 7 (pairing). */
#define ESB_HID_PIPE_ENABLE_MASK 0x8F

/* Set once esb_init() has succeeded. Retries after a later step
 * failed must NOT call esb_init() again: its sub-drivers (timer)
 * are already initialized and would fail with NRFX_ERROR_ALREADY,
 * masking the real failing step on every retry. */
static bool esb_initialized;

int esb_hid_link_start(enum esb_mode mode, esb_event_handler handler) {
    int err;

    if (!esb_initialized) {
        struct esb_config config = ESB_DEFAULT_CONFIG;

        config.mode = mode;
        config.event_handler = handler;
        config.tx_output_power = 4;
        config.retransmit_delay = 500;
        config.retransmit_count = 7;

        err = esb_init(&config);
        if (err) {
            LOG_ERR("esb_init failed: %d", err);
            return err;
        }
        esb_initialized = true;
    }

    err = esb_set_base_address_0(base_address_0);
    if (err) {
        LOG_ERR("esb_set_base_address_0 failed: %d", err);
        return err;
    }
    err = esb_set_base_address_1(base_address_1);
    if (err) {
        LOG_ERR("esb_set_base_address_1 failed: %d", err);
        return err;
    }
    err = esb_set_prefixes(pipe_prefixes, ARRAY_SIZE(pipe_prefixes));
    if (err) {
        LOG_ERR("esb_set_prefixes failed: %d", err);
        return err;
    }
    err = esb_enable_pipes(ESB_HID_PIPE_ENABLE_MASK);
    if (err) {
        LOG_ERR("esb_enable_pipes failed: %d", err);
        return err;
    }

    if (mode == ESB_MODE_PRX) {
        err = esb_start_rx();
        if (err) {
            LOG_ERR("esb_start_rx failed: %d", err);
            return err;
        }
    }

    LOG_INF("ESB link started (mode %d)", mode);
    return 0;
}
