/*
 * ZMK Studio RPC transport over ESB (keyboard side).
 *
 * Registers a Studio RPC transport bound to ZMK_TRANSPORT_ESB, so the
 * Studio core selects it whenever the ESB endpoint is the selected
 * endpoint. The RPC byte stream (already SOF/EOF/ESC framed by the
 * Studio core) is chopped into <=62 byte fragments and sent as
 * ESB_HID_TYPE_STUDIO frames through the shared TX queue in
 * esb_hid_tx.c. Downlink fragments arrive as ESB ACK payloads, are
 * drained by the TX thread (esb_hid_tx.c) and handed to
 * esb_hid_studio_on_rx_bytes(), which feeds them into the RPC RX ring
 * buffer unchanged — the stream framing survives arbitrary splits.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <zmk/studio/rpc.h>

#include "esb_hid_frame.h"
#include "esb_hid_studio.h"

LOG_MODULE_DECLARE(esb_hid);

#define ESB_HID_STUDIO_FRAG_MAX (CONFIG_ESB_MAX_PAYLOAD_LENGTH - ESB_HID_FRAME_HDR_LEN)

static K_SEM_DEFINE(esb_studio_tx_sem, 0, 1);
static volatile bool esb_studio_receiving;

static int esb_studio_rx_start(void) {
    esb_studio_receiving = true;
    return 0;
}

static int esb_studio_rx_stop(void) {
    esb_studio_receiving = false;
    return 0;
}

static void esb_studio_tx_notify(struct ring_buf *tx_ring_buf, size_t written, bool msg_done,
                                 void *user_data) {
    ARG_UNUSED(tx_ring_buf);
    ARG_UNUSED(written);
    ARG_UNUSED(msg_done);
    ARG_UNUSED(user_data);
    /* The drain thread does the actual work; it also polls, so a lost
     * wakeup only costs a few ms. */
    k_sem_give(&esb_studio_tx_sem);
}

ZMK_RPC_TRANSPORT(esb_studio, ZMK_TRANSPORT_ESB, esb_studio_rx_start, esb_studio_rx_stop, NULL,
                  esb_studio_tx_notify);

void esb_hid_studio_on_rx_bytes(const uint8_t *data, size_t len) {
    struct ring_buf *rx_buf = zmk_rpc_get_rx_buf();
    uint32_t put = ring_buf_put(rx_buf, (uint8_t *)data, (uint32_t)len);

    if (put < len) {
        LOG_WRN("Studio RX ring full, dropped %d of %d bytes", (int)(len - put), (int)len);
    }
    if (put > 0) {
        zmk_rpc_rx_notify();
    }
}

static void esb_studio_drain_thread(void) {
    uint8_t frag[ESB_HID_STUDIO_FRAG_MAX];

    while (1) {
        k_sem_take(&esb_studio_tx_sem, K_MSEC(50));

        if (!esb_studio_receiving) {
            continue;
        }

        struct ring_buf *tx_buf = zmk_rpc_get_tx_buf();

        while (ring_buf_size_get(tx_buf) > 0) {
            uint32_t chunk = ring_buf_get(tx_buf, frag, sizeof(frag));

            if (chunk == 0) {
                break;
            }

            /* The main TX queue is shared with HID reports and can be
             * momentarily full; wait for space rather than dropping
             * stream bytes (losing one fragment corrupts the whole
             * RPC message). Give up only if the transport was stopped
             * meanwhile. */
            while (esb_hid_tx_enqueue_studio(frag, chunk) == -ENOSPC && esb_studio_receiving) {
                k_sleep(K_MSEC(5));
            }
        }
    }
}

K_THREAD_DEFINE(esb_hid_studio_drain_thread, 1024, esb_studio_drain_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(8), 0, 0);
