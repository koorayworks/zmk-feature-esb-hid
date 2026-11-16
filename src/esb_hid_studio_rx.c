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
 *
 *   Session resync: a fragment can still be in flight over ESB while
 *   the flush runs and land in the uplink queue after it; its bytes
 *   (typically a lone trailing EOF) would head the next session's
 *   stream. The CDC out thread therefore drops bytes after boot and
 *   after every flush until the first SOF (0xAB) of a framed message
 *   has been forwarded.
 *
 *   Session generations: every flush, and every session start (CDC
 *   DTR rise), bumps a generation counter. Uplink chunks are stamped
 *   with the generation at queue time and the CDC out thread drops
 *   stamps from older generations, which closes the purge races (a
 *   chunk being written when the flush runs, a fragment landing in
 *   the queue right after the purge). The out thread also never
 *   writes while no client is connected, and the CDC in thread never
 *   queues downlink bytes while no client is connected: bytes parked
 *   in the CDC TX buffer are delivered at the head of the next
 *   session, and one stale response there (an old getDeviceInfo
 *   reply) is enough to abort it with a request-ID mismatch.
 *
 *   Request-ID gate: those gates still cannot stop a complete
 *   response that the keyboard generated for a previous session's
 *   request but that is only delivered after the next session has
 *   started — it is a well-formed frame and passes the resync. So
 *   the out thread additionally parses the head of every uplink
 *   message and forwards a RequestResponse only when its echoed
 *   request_id is currently outstanding for this session (registered
 *   by a passive observer on the downlink), consuming the entry as
 *   it passes; anything else is swallowed whole. Notifications
 *   carry no request ID and pass whenever the stream itself does.
 *   See the gate section below for the exact rules.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/spinlock.h>
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
    /* Session generation at queue time (see studio_session_gen). */
    uint32_t gen;
    uint8_t data[ESB_HID_STUDIO_FRAG_MAX];
};

K_MSGQ_DEFINE(esb_studio_up_queue, sizeof(struct esb_studio_up_chunk), 16, 4);

/* ZMK Studio framing start-of-frame byte (FRAMING_SOF in the Studio
 * core). A framed stream always begins with it. */
#define ESB_HID_STUDIO_SOF_BYTE 0xAB

/* Session resync: fragments of a dead session can still be in flight
 * over ESB when the session flush runs; they land in the up-queue
 * afterwards and would be the first bytes the next client reads. A
 * lone trailing EOF (0xAD) is the common case, and strict framing
 * decoders abort the connection on a non-SOF first byte. So after
 * boot and after every session flush, the CDC out thread drops bytes
 * until the first SOF has been forwarded — the same idle-state
 * behaviour as the reference framing decoder. A live session is
 * unaffected: its stream starts with SOF by definition. */
static volatile bool studio_await_sof = true;

/* Session generation: bumped by every session flush and by every
 * session start, so chunks queued for a previous session can be told
 * apart from the current one's even when they reach the up-queue (or
 * are being written) after the flush's purge has already run. */
static volatile uint32_t studio_session_gen;

/* Session hygiene counters, reported (and reset) when the next
 * client connects, so the drops above stay observable on hardware:
 * uplink chunks dropped for a stale generation, and bytes dropped
 * because no client was connected to receive them. */
static volatile uint32_t up_stale_chunk_drops;
static volatile uint32_t up_unattended_byte_drops;
static volatile uint32_t down_unattended_byte_drops;

void esb_hid_studio_fwd_to_pc(const uint8_t *data, size_t len) {
    struct esb_studio_up_chunk chunk;

    if (len > sizeof(chunk.data)) {
        len = sizeof(chunk.data);
    }
    chunk.len = (uint8_t)len;
    chunk.gen = studio_session_gen;
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
/* Request-ID gate                                                     */

/* A passive observer on the downlink (CDC in thread) registers the
 * request_id at the head of every framed Request the client sends;
 * the CDC out thread parses the head of every uplink message on the
 * fly and forwards a RequestResponse only when its echoed
 * request_id is in that outstanding set, consuming the entry as it
 * passes. A response produced for a previous session's request —
 * still in flight when the next session starts, a complete and
 * well-formed frame that no framing-level filter can tell apart —
 * names an ID the new session never registered (or has already had
 * answered), so it is swallowed whole, as is a second copy of a
 * legitimate response. Notifications (Response field 2) carry no
 * request_id and are forwarded whenever the stream itself is.
 *
 * Wire format, verified against the sources rather than assumed:
 * framing is SOF 0xAB / ESC 0xAC / EOF 0xAD with ESC + byte for a
 * data byte equal to a framing byte (zmk msg_framing, mirrored by
 * the Corne bare-metal studio_framing). Inside the frame sits a
 * protobuf zmk.studio.Response: field 1 (tag 0x0A) wraps a
 * RequestResponse whose first field is request_id (tag 0x08,
 * varint); field 2 (tag 0x12) wraps a Notification. A downlink
 * zmk.studio.Request likewise carries request_id as field 1, ahead
 * of the subsystem oneof (fields 3/4/5). Every encoder in this
 * ecosystem emits it first — the proto declares it field 1, nanopb
 * (ZMK keyboards) encodes in field order, and the Corne hand-rolled
 * writer emits it before the subsystem (resp_begin in its
 * studio.c) — and a proto3 sender that omits a zero request_id
 * instead shows its subsystem tag first, which the parsers below
 * treat as ID 0. A head that parses neither way is dropped as
 * unparseable rather than passed on faith.
 *
 * Neither parser ever buffers a whole message (the receiver's RAM
 * is nearly full): the uplink gate holds only the raw bytes of the
 * current message head — SOF, the Response tag, the submessage
 * length varint, the request_id tag and varint, a few bytes in
 * practice — latches a pass/drop decision the moment the ID is
 * known, and streams or swallows the remainder until EOF.
 *
 * The outstanding table is keyed to the session generation: the
 * first register/consume after a generation bump starts from an
 * empty table, which is what clears it on every session flush, with
 * no extra work in the flush paths themselves. */

#define ESB_HID_STUDIO_GATE_MAX_IDS 16
#define ESB_HID_STUDIO_GATE_HDR_MAX 32

/* Studio framing bytes (see the framing note above). SOF already
 * has ESB_HID_STUDIO_SOF_BYTE. */
#define ESB_HID_STUDIO_ESC_BYTE 0xAC
#define ESB_HID_STUDIO_EOF_BYTE 0xAD

/* Protobuf tags at the message heads parsed below. */
#define GATE_TAG_REQUEST_ID 0x08     /* field 1, varint (both directions) */
#define GATE_TAG_RESPONSE_RR 0x0A    /* Response field 1: RequestResponse */
#define GATE_TAG_RESPONSE_NOTIF 0x12 /* Response field 2: Notification */

/* Gate hygiene counters, reported (and reset) with the rest of the
 * session hygiene counters at the next session start. */
static volatile uint32_t up_gate_msg_drops;
static volatile uint32_t up_gate_byte_drops;

static struct k_spinlock gate_table_lock;
static uint32_t gate_table_ids[ESB_HID_STUDIO_GATE_MAX_IDS];
static uint8_t gate_table_count;
static uint32_t gate_table_gen;

/* Caller holds gate_table_lock. */
static void gate_table_sync_gen(void) {
    if (gate_table_gen != studio_session_gen) {
        gate_table_gen = studio_session_gen;
        gate_table_count = 0;
    }
}

static void gate_register_id(uint32_t id) {
    k_spinlock_key_t key = k_spin_lock(&gate_table_lock);

    gate_table_sync_gen();
    for (int i = 0; i < gate_table_count; i++) {
        if (gate_table_ids[i] == id) {
            k_spin_unlock(&gate_table_lock, key);
            return;
        }
    }
    if (gate_table_count == ESB_HID_STUDIO_GATE_MAX_IDS) {
        /* More requests in flight than the table holds: evict the
         * oldest registration. If its response ever arrives it is
         * dropped as unknown and the client recovers by retrying,
         * while the requests it is actively waiting for stay gated
         * correctly. */
        LOG_WRN("Studio request-id table full, evicting oldest id %u", gate_table_ids[0]);
        memmove(&gate_table_ids[0], &gate_table_ids[1],
                (ESB_HID_STUDIO_GATE_MAX_IDS - 1) * sizeof(gate_table_ids[0]));
        gate_table_count--;
    }
    gate_table_ids[gate_table_count++] = id;
    k_spin_unlock(&gate_table_lock, key);
}

/* Remove id from the outstanding set; returns true when it was
 * outstanding (and this response is therefore its first). */
static bool gate_consume_id(uint32_t id) {
    k_spinlock_key_t key = k_spin_lock(&gate_table_lock);
    bool found = false;

    gate_table_sync_gen();
    for (int i = 0; i < gate_table_count; i++) {
        if (gate_table_ids[i] == id) {
            memmove(&gate_table_ids[i], &gate_table_ids[i + 1],
                    (gate_table_count - i - 1) * sizeof(gate_table_ids[0]));
            gate_table_count--;
            found = true;
            break;
        }
    }
    k_spin_unlock(&gate_table_lock, key);
    return found;
}

/* Downlink request observer (CDC in thread only): watches the bytes
 * on their way into the downlink ring and registers the request_id
 * at the head of each framed Request. Passive — it never alters,
 * delays or drops the stream itself. */
enum gate_down_state {
    GATE_DOWN_WAIT_SOF,
    GATE_DOWN_FIRST_TAG,
    GATE_DOWN_ID_VARINT,
    GATE_DOWN_IN_MSG,
};

static enum gate_down_state gate_down_state = GATE_DOWN_WAIT_SOF;
static bool gate_down_escaped;
static uint32_t gate_down_id;
static uint8_t gate_down_id_bytes;

static void gate_down_reset(void) {
    gate_down_state = GATE_DOWN_WAIT_SOF;
    gate_down_escaped = false;
}

static void gate_down_observe(uint8_t byte) {
    if (gate_down_escaped) {
        gate_down_escaped = false;
        /* byte is a literal payload byte: fall through. */
    } else if (byte == ESB_HID_STUDIO_SOF_BYTE) {
        /* A SOF always (re)starts a message head, even mid-message
         * (the framing layer's error recovery does the same). */
        gate_down_state = GATE_DOWN_FIRST_TAG;
        return;
    } else if (byte == ESB_HID_STUDIO_EOF_BYTE) {
        gate_down_state = GATE_DOWN_WAIT_SOF;
        return;
    } else if (byte == ESB_HID_STUDIO_ESC_BYTE && gate_down_state != GATE_DOWN_WAIT_SOF) {
        gate_down_escaped = true;
        return;
    } else if (gate_down_state == GATE_DOWN_WAIT_SOF) {
        return;
    }

    /* Payload byte. */
    switch (gate_down_state) {
    case GATE_DOWN_FIRST_TAG:
        if (byte == GATE_TAG_REQUEST_ID) {
            gate_down_id = 0;
            gate_down_id_bytes = 0;
            gate_down_state = GATE_DOWN_ID_VARINT;
        } else {
            /* First field is not request_id: with the field order
             * every client encoder emits, that is the proto3-omitted
             * zero ID — but only register 0 when the field really
             * is a Request subsystem (fields 3/4/5, all
             * length-delimited: tags 0x1A/0x22/0x2A). Any other head
             * is not a Request this observer understands. */
            if (byte == 0x1A || byte == 0x22 || byte == 0x2A) {
                gate_register_id(0);
            }
            gate_down_state = GATE_DOWN_IN_MSG;
        }
        break;
    case GATE_DOWN_ID_VARINT:
        gate_down_id |= (uint32_t)(byte & 0x7F) << (7 * gate_down_id_bytes);
        gate_down_id_bytes++;
        if ((byte & 0x80) == 0) {
            gate_register_id(gate_down_id);
            gate_down_state = GATE_DOWN_IN_MSG;
        } else if (gate_down_id_bytes >= 5) {
            /* Not a uint32 varint: not a parseable request head. */
            gate_down_state = GATE_DOWN_IN_MSG;
        }
        break;
    default:
        break;
    }
}

/* Uplink gate (CDC out thread only): parses each message head on
 * the fly and latches a pass/drop decision for the whole message.
 * While undecided, the head's raw bytes are held; a pass writes
 * them out and streams the rest, a drop swallows the rest until
 * EOF. The held bytes are raw wire bytes (ESC sequences included),
 * so a passed message is forwarded byte-identically. */
enum gate_up_state {
    GATE_UP_WAIT_SOF,
    GATE_UP_HDR_TAG,
    GATE_UP_HDR_RR_LEN,
    GATE_UP_HDR_ID_TAG,
    GATE_UP_HDR_ID_VAL,
    GATE_UP_PASS,
    GATE_UP_DROP,
};

static enum gate_up_state gate_up_state = GATE_UP_WAIT_SOF;
static bool gate_up_escaped;
static uint8_t gate_up_hold[ESB_HID_STUDIO_GATE_HDR_MAX];
static uint8_t gate_up_hold_len;
static uint32_t gate_up_id;
static uint8_t gate_up_varint_bytes;
/* Generation the machine state belongs to; a chunk of a newer
 * generation resets the machine before its first byte is fed. */
static uint32_t gate_up_gen;

static void gate_up_reset(void) {
    gate_up_state = GATE_UP_WAIT_SOF;
    gate_up_escaped = false;
    gate_up_hold_len = 0;
    gate_up_gen = studio_session_gen;
}

static void gate_up_decide_pass(void) {
    for (int i = 0; i < gate_up_hold_len; i++) {
        uart_poll_out(cdc_dev, gate_up_hold[i]);
    }
    gate_up_hold_len = 0;
    gate_up_state = GATE_UP_PASS;
}

static void gate_up_decide_drop(const char *why) {
    LOG_DBG("Studio uplink gate: dropping message (%s)", why);
    up_gate_msg_drops++;
    up_gate_byte_drops += gate_up_hold_len;
    gate_up_hold_len = 0;
    gate_up_state = GATE_UP_DROP;
}

static void gate_up_feed(uint8_t byte) {
    if (gate_up_state == GATE_UP_WAIT_SOF) {
        if (byte != ESB_HID_STUDIO_SOF_BYTE) {
            /* Stale byte ahead of the next frame (post-flush junk
             * or inter-message noise): drop it. */
            return;
        }
        if (studio_await_sof) {
            studio_await_sof = false;
            LOG_INF("Studio uplink resynced (first SOF forwarded)");
        }
        gate_up_hold[0] = byte;
        gate_up_hold_len = 1;
        gate_up_escaped = false;
        gate_up_state = GATE_UP_HDR_TAG;
        return;
    }

    /* Inside a message: settle the framing role of this byte first.
     * An escaped byte is literal payload even when it equals a
     * framing byte. */
    bool payload = true;

    if (gate_up_escaped) {
        gate_up_escaped = false;
    } else if (byte == ESB_HID_STUDIO_ESC_BYTE) {
        gate_up_escaped = true;
        payload = false;
    } else if (byte == ESB_HID_STUDIO_SOF_BYTE) {
        /* Unescaped SOF mid-message: the sender aborted the frame
         * (the reference framing decoder drops to its error state
         * here). Restart the head parse with this SOF. A partial
         * head that was never decided is framing garbage and stays
         * unforwarded; count its held bytes as dropped. */
        if (gate_up_state >= GATE_UP_HDR_TAG && gate_up_state <= GATE_UP_HDR_ID_VAL) {
            up_gate_byte_drops += gate_up_hold_len;
        }
        gate_up_hold[0] = byte;
        gate_up_hold_len = 1;
        gate_up_state = GATE_UP_HDR_TAG;
        return;
    } else if (byte == ESB_HID_STUDIO_EOF_BYTE) {
        if (gate_up_state == GATE_UP_PASS) {
            uart_poll_out(cdc_dev, byte);
        } else if (gate_up_state == GATE_UP_DROP) {
            up_gate_byte_drops++;
        } else {
            /* EOF inside an undecided head: a truncated message.
             * Nothing of it was forwarded; count it as dropped. */
            up_gate_msg_drops++;
            up_gate_byte_drops += gate_up_hold_len + 1;
        }
        gate_up_hold_len = 0;
        gate_up_state = GATE_UP_WAIT_SOF;
        return;
    }

    /* Ordinary byte (payload or ESC marker): route it by state. */
    if (gate_up_state == GATE_UP_PASS) {
        uart_poll_out(cdc_dev, byte);
    } else if (gate_up_state == GATE_UP_DROP) {
        up_gate_byte_drops++;
    } else {
        if (gate_up_hold_len >= sizeof(gate_up_hold)) {
            /* A head longer than any parseable request head: give
             * up on this message (this byte is swallowed too). */
            gate_up_decide_drop("overlong head");
            up_gate_byte_drops++;
            return;
        }
        gate_up_hold[gate_up_hold_len++] = byte;
    }

    if (!payload) {
        return;
    }

    /* Payload byte: advance the head parser while undecided. */
    switch (gate_up_state) {
    case GATE_UP_HDR_TAG:
        if (byte == GATE_TAG_RESPONSE_RR) {
            gate_up_varint_bytes = 0;
            gate_up_state = GATE_UP_HDR_RR_LEN;
        } else if (byte == GATE_TAG_RESPONSE_NOTIF) {
            /* Notifications carry no request_id: they pass whenever
             * the surrounding session gates let the stream through
             * at all. */
            gate_up_decide_pass();
        } else {
            gate_up_decide_drop("unknown Response head");
        }
        break;
    case GATE_UP_HDR_RR_LEN:
        gate_up_varint_bytes++;
        if ((byte & 0x80) == 0) {
            gate_up_state = GATE_UP_HDR_ID_TAG;
        } else if (gate_up_varint_bytes >= 5) {
            gate_up_decide_drop("overlong length varint");
        }
        break;
    case GATE_UP_HDR_ID_TAG:
        if (byte == GATE_TAG_REQUEST_ID) {
            gate_up_id = 0;
            gate_up_varint_bytes = 0;
            gate_up_state = GATE_UP_HDR_ID_VAL;
        } else if (gate_consume_id(0)) {
            /* request_id omitted (proto3 zero) and a request really
             * carried no ID in this session. */
            gate_up_decide_pass();
        } else {
            gate_up_decide_drop("response without request_id");
        }
        break;
    case GATE_UP_HDR_ID_VAL:
        gate_up_id |= (uint32_t)(byte & 0x7F) << (7 * gate_up_varint_bytes);
        gate_up_varint_bytes++;
        if ((byte & 0x80) == 0) {
            if (gate_consume_id(gate_up_id)) {
                gate_up_decide_pass();
            } else {
                gate_up_decide_drop("unknown or already-answered request_id");
            }
        } else if (gate_up_varint_bytes >= 5) {
            gate_up_decide_drop("overlong request_id varint");
        }
        break;
    default:
        break;
    }
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
    studio_await_sof = true;
    /* Everything queued or in flight from before this point belongs
     * to the dead session; the generation bump lets the CDC out
     * thread drop it even if it surfaces after the purge. */
    studio_session_gen++;
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

/* Uplink-only flush for a session start (DTR rise): purge whatever
 * the previous session left in the up-queue, re-arm the resync and
 * bump the generation, but leave the downlink ring and the radio
 * alone — the new client may already be sending, and its bytes are
 * the new session's own. Loader thread only. */
static void esb_studio_do_uplink_flush(void) {
    k_msgq_purge(&esb_studio_up_queue);
    studio_await_sof = true;
    studio_session_gen++;

    LOG_INF("Studio uplink flushed (new session)");
}

/* Report the session hygiene counters accumulated since the last
 * session start, then reset them. Loader thread only. */
static void esb_studio_report_hygiene(void) {
    if (up_stale_chunk_drops == 0 && up_unattended_byte_drops == 0 &&
        down_unattended_byte_drops == 0 && up_gate_msg_drops == 0) {
        return;
    }
    LOG_INF("Studio session hygiene: dropped %u stale uplink chunk(s), %u uplink byte(s) "
            "and %u downlink byte(s) while no client was connected; "
            "request-id gate dropped %u message(s), %u byte(s)",
            up_stale_chunk_drops, up_unattended_byte_drops, down_unattended_byte_drops,
            up_gate_msg_drops, up_gate_byte_drops);
    up_stale_chunk_drops = 0;
    up_unattended_byte_drops = 0;
    down_unattended_byte_drops = 0;
    up_gate_msg_drops = 0;
    up_gate_byte_drops = 0;
}

/* Watch the CDC client's DTR edges: flush when a readable DTR drops
 * from connected to not-connected, and start each new session from a
 * clean uplink state when it rises. The unreadable case (fallback:
 * assume connected) never flushes, it has no edge to detect. */
static bool dtr_sample_valid;
static bool dtr_was_connected;

static void esb_studio_check_dtr_edges(void) {
    bool connected;

    if (!esb_studio_cdc_dtr(&connected)) {
        return;
    }
    if (dtr_sample_valid && dtr_was_connected && !connected) {
        LOG_INF("Studio client disconnected (DTR drop)");
        esb_studio_do_flush();
    } else if (dtr_sample_valid && !dtr_was_connected && connected) {
        LOG_INF("Studio client connected (DTR rise)");
        esb_studio_report_hygiene();
        esb_studio_do_uplink_flush();
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
        .pipe = esb_hid_rx_studio_pipe(),
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

        /* Generation at the start of this pass: if a flush runs
         * below (posted, or a DTR edge), bytes claimed in this pass
         * straddle the flush boundary and must not be loaded. */
        uint32_t pass_gen = studio_session_gen;

        if (flush_requested) {
            flush_requested = false;
            esb_studio_do_flush();
        }
        esb_studio_check_dtr_edges();

        bool fifo_full = false;

        while (ring_buf_size_get(&esb_studio_down_buf) > 0) {
            uint32_t claim_len = ring_buf_get_claim(&esb_studio_down_buf, &claim_buf,
                                                    ESB_HID_STUDIO_FRAG_MAX);

            if (claim_len == 0) {
                break;
            }

            if (studio_session_gen != pass_gen) {
                /* A flush ran in this pass and reset the ring, so
                 * this claim is void; leave the bytes for the next
                 * pass rather than load pre-flush bytes into the
                 * ACK FIFO. */
                ring_buf_get_finish(&esb_studio_down_buf, 0);
                LOG_WRN("Studio downlink load deferred across a session flush");
                break;
            }

            struct esb_payload payload = {
                .length = (uint8_t)(claim_len + ESB_HID_FRAME_HDR_LEN),
                .pipe = esb_hid_rx_studio_pipe(),
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

        if (!esb_studio_cdc_connected()) {
            /* No client session: this byte is the tail of a dead
             * session draining out of the CDC RX buffer late. It
             * must not enter the downlink ring, where it would head
             * the next session's request stream. */
            down_unattended_byte_drops++;
            gate_down_reset();
            continue;
        }

        if (ring_buf_put(&esb_studio_down_buf, &byte, 1) < 1) {
            LOG_WRN("Studio down-buffer full, dropping CDC byte");
            continue;
        }
        /* The byte is on its way to the keyboard: let the gate
         * observer note the request_id if this is a request head. */
        gate_down_observe(byte);
        k_sem_give(&esb_studio_down_sem);
    }
}

K_THREAD_DEFINE(esb_hid_studio_cdc_in_thread, 768, esb_studio_cdc_in_thread, NULL, NULL, NULL,
                K_PRIO_PREEMPT(8), 0, 0);

static void esb_studio_cdc_out_thread(void) {
    struct esb_studio_up_chunk chunk;

    while (1) {
        k_msgq_get(&esb_studio_up_queue, &chunk, K_FOREVER);

        if (chunk.gen != studio_session_gen) {
            /* Queued for a session that has since been flushed (the
             * fragment was in flight over ESB while the purge ran,
             * or the queue was purged around it): it belongs to a
             * dead session and must never head a new one. */
            up_stale_chunk_drops++;
            continue;
        }

        if (!esb_studio_cdc_connected()) {
            /* No client: bytes written now would wait in the CDC TX
             * buffer and be delivered at the head of the NEXT
             * session, poisoning it — one stale getDeviceInfo
             * response is enough for an instant request-ID mismatch.
             * Drop them instead; a client re-issues its requests
             * when it connects. */
            up_unattended_byte_drops += chunk.len;
            continue;
        }

        if (chunk.gen != gate_up_gen) {
            /* First chunk of a new session generation: any message
             * the gate was in the middle of belonged to the dead
             * session. */
            gate_up_reset();
        }

        for (int i = 0; i < chunk.len; i++) {
            if (chunk.gen != studio_session_gen) {
                /* A flush ran while this chunk was being written:
                 * stop rather than finish a dead session's message
                 * into the next one. */
                up_stale_chunk_drops++;
                gate_up_reset();
                break;
            }
            gate_up_feed(chunk.data[i]);
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
