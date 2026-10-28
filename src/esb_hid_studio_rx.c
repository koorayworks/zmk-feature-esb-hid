/*
 * Studio over ESB forwarder (receiver side).
 *
 * The receiver exposes a second USB CDC ACM port (devicetree node
 * "esb_studio_cdc", compatible zephyr,cdc-acm-uart) that the PC's
 * ZMK Studio connects to over Web Serial. This file is a dumb byte
 * bridge; the receiver has no Studio core of its own:
 *
 *   PC -> CDC -> ring buffer -> fragments of at most 62 bytes ->
 *   esb_write_payload() in PRX mode, which queues them as ACK
 *   payloads. Each fragment is attached to the ACK of the next
 *   packet the keyboard sends (a report or a Studio poll frame).
 *
 *   While a Studio client is connected (CDC DTR set) and no real
 *   fragment is queued, the loader also keeps a one-byte
 *   ESB_HID_TYPE_STUDIO_HINT in the ACK FIFO, so the keyboard learns
 *   about the session from the ACK of its own regular traffic and
 *   only then starts polling (detection-based polling; see
 *   esb_hid_tx.c). Hints are stateless and idempotent.
 *
 *   Keyboard -> ESB_HID_TYPE_STUDIO frames (handled in esb_hid_rx.c)
 *   -> esb_hid_studio_fwd_to_pc() -> message queue -> CDC out.
 *
 *   Session flush: state held for one keyboard boot / CDC session
 *   must not leak into the next. When the keyboard reports a reboot
 *   (status BOOT flag, see esb_hid_rx.c) or the CDC client goes away
 *   (DTR drop), the pending downlink bytes, the uplink queue and the
 *   ACK payloads queued in the radio are all dropped. The purge runs
 *   in the ACK loader thread, which owns the downlink ring buffer
 *   claims and is the only caller of esb_write_payload() here.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

#include <string.h>

#include <esb.h>

#include "esb_hid_frame.h"
#include "esb_hid_studio.h"

LOG_MODULE_DECLARE(esb_hid);

#if DT_NODE_EXISTS(DT_NODELABEL(esb_studio_cdc))

#define ESB_HID_STUDIO_FRAG_MAX (CONFIG_ESB_MAX_PAYLOAD_LENGTH - ESB_HID_FRAME_HDR_LEN)

static const struct device *const cdc_dev = DEVICE_DT_GET(DT_NODELABEL(esb_studio_cdc));

/* PC -> keyboard bytes waiting to be loaded into the ACK FIFO. */
RING_BUF_DECLARE(esb_studio_down_buf, 2048);
static K_SEM_DEFINE(esb_studio_down_sem, 0, K_SEM_MAX_LIMIT);
static uint8_t down_seq;

/* Keyboard -> PC fragments waiting for CDC TX. */
struct esb_studio_up_chunk {
    uint8_t len;
    uint8_t data[ESB_HID_STUDIO_FRAG_MAX];
};

K_MSGQ_DEFINE(esb_studio_up_queue, sizeof(struct esb_studio_up_chunk), 16, 4);

void esb_hid_studio_fwd_to_pc(const uint8_t *data, size_t len) {
    struct esb_studio_up_chunk chunk;

    if (len > sizeof(chunk.data)) {
        len = sizeof(chunk.data);
    }
    chunk.len = (uint8_t)len;
    memcpy(chunk.data, data, len);

    if (k_msgq_put(&esb_studio_up_queue, &chunk, K_NO_WAIT) != 0) {
        LOG_WRN("Studio up-queue full, dropping fragment (%d bytes)", (int)len);
    }
}

/* Set by the link-event note: receiving a packet retired the ACK
 * FIFO head (prepare_ack_pdu_dpl pops it when preparing the next
 * ACK), so queueing one fresh hint is due. Cleared once a hint has
 * been written, which keeps at most one not-yet-delivered hint plus
 * the one currently at the FIFO head in flight; hints are stateless,
 * so an occasional duplicate or stale copy is harmless. Starts armed
 * so the first packet after boot can already carry a hint. */
static volatile bool hint_replenish_armed = true;

void esb_hid_studio_fwd_note_link_event(void) {
    hint_replenish_armed = true;
    k_sem_give(&esb_studio_down_sem);
}

/* Read the CDC DTR line. Returns false when the line state cannot be
 * read (driver without line-ctrl support); *connected is then unset. */
static bool esb_studio_cdc_dtr(bool *connected) {
    uint32_t dtr = 0;

    if (uart_line_ctrl_get(cdc_dev, UART_LINE_CTRL_DTR, &dtr) != 0) {
        return false;
    }
    *connected = dtr != 0;
    return true;
}

/* Is a Studio client attached to the CDC port? Web Serial asserts
 * DTR when the app opens the port. If the line state cannot be read,
 * assume connected: hints are cheap and the keyboard ignores them
 * when no session follows. */
static bool esb_studio_cdc_connected(void) {
    bool connected;

    if (!esb_studio_cdc_dtr(&connected)) {
        return true;
    }
    return connected;
}

/* ------------------------------------------------------------------ */
/* Session flush                                                       */

/* Posted by esb_hid_studio_fwd_flush_session(); the purge itself runs
 * in the ACK loader thread (see the file header). */
static volatile bool flush_requested;

void esb_hid_studio_fwd_flush_session(void) {
    flush_requested = true;
    k_sem_give(&esb_studio_down_sem);
}

/* Drop everything held for the current session. Loader thread only.
 *
 * The radio half needs care: esb_flush_tx() only runs while the ESB
 * state machine is IDLE, but a listening PRX sits in ESB_STATE_PRX,
 * so the ACK payloads queued in the radio's TX FIFO are cleared by
 * suspending RX first and restarting it right after. The gap is
 * sub-millisecond; a frame lost in it is covered by the sender's
 * retries / the status heartbeat. If RX was not listening (link not
 * started yet), the FIFO is cleared while idle and starting RX stays
 * with the link retry loop, as before. */
static void esb_studio_do_flush(void) {
    ring_buf_reset(&esb_studio_down_buf);
    k_msgq_purge(&esb_studio_up_queue);
    hint_replenish_armed = true;

    int err = esb_stop_rx();

    if (err == 0) {
        err = esb_flush_tx();
        if (err) {
            LOG_WRN("Studio flush: esb_flush_tx failed: %d", err);
        }
        err = esb_start_rx();
        if (err) {
            LOG_ERR("Studio flush: esb_start_rx failed: %d", err);
        }
    } else if (err == -EALREADY) {
        err = esb_flush_tx();
        if (err) {
            LOG_WRN("Studio flush: esb_flush_tx failed: %d", err);
        }
    } else {
        LOG_DBG("Studio flush: radio untouched (esb_stop_rx: %d)", err);
    }

    LOG_INF("Studio session flushed");
}

/* Flush when the CDC client goes away: a readable DTR that drops
 * from connected to not-connected. The unreadable case (fallback:
 * assume connected) never flushes, it has no edge to detect. */
static bool dtr_sample_valid;
static bool dtr_was_connected;

static void esb_studio_check_dtr_drop(void) {
    bool connected;

    if (!esb_studio_cdc_dtr(&connected)) {
        return;
    }
    if (dtr_sample_valid && dtr_was_connected && !connected) {
        LOG_INF("Studio client disconnected (DTR drop)");
        esb_studio_do_flush();
    }
    dtr_sample_valid = true;
    dtr_was_connected = connected;
}

static void esb_studio_maybe_load_hint(void) {
    if (!hint_replenish_armed || !esb_studio_cdc_connected()) {
        return;
    }

    uint8_t flags = ESB_HID_STUDIO_HINT_CONNECTED;
    if (ring_buf_size_get(&esb_studio_down_buf) > 0) {
        flags |= ESB_HID_STUDIO_HINT_DOWNLINK_PENDING;
    }

    struct esb_payload payload = {
        .length = ESB_HID_FRAME_HDR_LEN + 1,
        .pipe = 0,
        .noack = 0,
    };
    payload.data[0] = ESB_HID_TYPE_STUDIO_HINT;
    payload.data[1] = down_seq++;
    payload.data[ESB_HID_FRAME_HDR_LEN] = flags;

    if (esb_write_payload(&payload) == 0) {
        hint_replenish_armed = false;
    }
    /* On -ENOMEM (ACK FIFO full) stay armed and retry on the next
     * link event / timeout, when a slot may have freed up. */
}

/* Load queued downlink bytes into the ESB ACK payload FIFO. The FIFO
 * holds only a few payloads, so this runs whenever new CDC data
 * arrives and whenever a link packet was received (which means one
 * ACK payload was just consumed and a slot may be free again).
 * Real fragments always take priority over the session hint. */
static void esb_studio_ack_loader_thread(void) {
    uint8_t *claim_buf;

    while (1) {
        k_sem_take(&esb_studio_down_sem, K_MSEC(100));

        if (flush_requested) {
            flush_requested = false;
            esb_studio_do_flush();
        }
        esb_studio_check_dtr_drop();

        bool fifo_full = false;

        while (ring_buf_size_get(&esb_studio_down_buf) > 0) {
            uint32_t claim_len = ring_buf_get_claim(&esb_studio_down_buf, &claim_buf,
                                                    ESB_HID_STUDIO_FRAG_MAX);

            if (claim_len == 0) {
                break;
            }

            struct esb_payload payload = {
                .length = (uint8_t)(claim_len + ESB_HID_FRAME_HDR_LEN),
                .pipe = 0,
                .noack = 0,
            };
            payload.data[0] = ESB_HID_TYPE_STUDIO;
            payload.data[1] = down_seq;
            memcpy(&payload.data[ESB_HID_FRAME_HDR_LEN], claim_buf, claim_len);

            int err = esb_write_payload(&payload);

            if (err) {
                /* ACK FIFO full: keep the claim (commit 0) and wait
                 * for the next link event / timeout to retry. */
                ring_buf_get_finish(&esb_studio_down_buf, 0);
                LOG_DBG("ACK payload load deferred: %d", err);
                fifo_full = true;
                break;
            }

            ring_buf_get_finish(&esb_studio_down_buf, claim_len);
            down_seq++;
        }

        /* No fragment left the loader in this pass and the FIFO has
         * room: top up the session hint instead. If fragments are
         * still buffered, the keyboard learns about the downlink from
         * the fragments themselves once they are loaded. */
        if (!fifo_full && ring_buf_size_get(&esb_studio_down_buf) == 0) {
            esb_studio_maybe_load_hint();
        }
    }
}

K_THREAD_DEFINE(esb_hid_studio_ack_loader_thread, 1024, esb_studio_ack_loader_thread, NULL, NULL,
                NULL, K_PRIO_PREEMPT(8), 0, 0);

static void esb_studio_cdc_in_thread(void) {
    while (1) {
        uint8_t byte;

        if (uart_poll_in(cdc_dev, &byte) < 0) {
            k_sleep(K_MSEC(1));
            continue;
        }

        if (ring_buf_put(&esb_studio_down_buf, &byte, 1) < 1) {
            LOG_WRN("Studio down-buffer full, dropping CDC byte");
            continue;
        }
        k_sem_give(&esb_studio_down_sem);
    }
}

K_THREAD_DEFINE(esb_hid_studio_cdc_in_thread, 768, esb_studio_cdc_in_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(8), 0, 0);

static void esb_studio_cdc_out_thread(void) {
    struct esb_studio_up_chunk chunk;

    while (1) {
        k_msgq_get(&esb_studio_up_queue, &chunk, K_FOREVER);
        for (int i = 0; i < chunk.len; i++) {
            uart_poll_out(cdc_dev, chunk.data[i]);
        }
    }
}

K_THREAD_DEFINE(esb_hid_studio_cdc_out_thread, 768, esb_studio_cdc_out_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(8), 0, 0);

static int esb_hid_studio_rx_init(void) {
    if (!device_is_ready(cdc_dev)) {
        LOG_ERR("Studio CDC device not ready");
        return -ENODEV;
    }
    LOG_INF("Studio over ESB forwarder ready (CDC %s)", cdc_dev->name);
    return 0;
}

SYS_INIT(esb_hid_studio_rx_init, APPLICATION, 97);

#else /* DT_NODE_EXISTS(DT_NODELABEL(esb_studio_cdc)) */

/* Shield did not provide the CDC node: keep the rx.c call sites
 * linkable, forwarding is simply inert. */
void esb_hid_studio_fwd_to_pc(const uint8_t *data, size_t len) {
    ARG_UNUSED(data);
    ARG_UNUSED(len);
}

void esb_hid_studio_fwd_note_link_event(void) {}

void esb_hid_studio_fwd_flush_session(void) {}

#endif
