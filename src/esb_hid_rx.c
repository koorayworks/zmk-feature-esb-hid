/*
 * HID over ESB receiver (step 2: up to 4 keyboards + pairing).
 *
 * Runs on a matrix-less ZMK build connected to the host over USB.
 * Received [type][seq][body] frames are attributed to a device slot
 * by the ESB pipe they arrived on (pipes 0..3 = slots 0..3, pipe 7 =
 * pairing). Only frames from the active, registry-used slot are
 * forwarded to USB HID through the fork's
 * zmk_usb_hid_send_report_with_id() hook; other slots only update
 * their per-slot state (status, last-seen, cached HID bodies), so
 * switching the active slot can release the old device's keys and
 * re-inject the new device's held keys.
 *
 * The slot registry (which slots are paired, their names, the active
 * slot) is persisted in settings under "esb_hid/registry". The
 * pairing window state machine lives here too; its public API is in
 * <esb_hid.h>.
 */

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <errno.h>
#include <string.h>

#include <esb.h>
#include <esb_hid.h>
#include <esb_hid_status.h>
#include <zephyr/drivers/gpio.h>
#include <zmk/hid.h>
#include <zmk/usb_hid.h>

#include "esb_hid_frame.h"
#include "esb_hid_link.h"
#include "esb_hid_studio.h"

LOG_MODULE_DECLARE(esb_hid);

static K_SEM_DEFINE(rx_sem, 0, K_SEM_MAX_LIMIT);

/* Guards the registry, the per-slot peer state and the pairing state
 * against concurrent access from the RX thread, the pairing timer
 * and UI threads calling the public API. */
static K_MUTEX_DEFINE(esb_hid_rx_mutex);

/* ------------------------------------------------------------------ */
/* Slot registry (persisted)                                           */

struct esb_hid_registry {
    uint8_t active_slot;
    struct {
        uint8_t used;
        uint8_t dev_id[ESB_HID_DEVICE_ID_LEN];
        char name[ESB_HID_NAME_MAX + 1];
    } slots[ESB_HID_SLOT_COUNT];
};

/* On-disk registry formats (explicit; the RAM struct above is not
 * the disk layout). v1 = 73 bytes, v2 = 106 bytes. */
struct esb_hid_registry_disk_v1 {
    uint8_t active_slot;
    struct {
        uint8_t used;
        char name[ESB_HID_NAME_MAX + 1];
    } slots[ESB_HID_SLOT_COUNT];
};

struct esb_hid_registry_disk_v2 {
    uint8_t version; /* = 2 */
    uint8_t active_slot;
    struct {
        uint8_t used;
        uint8_t dev_id[ESB_HID_DEVICE_ID_LEN];
        char name[ESB_HID_NAME_MAX + 1];
    } slots[ESB_HID_SLOT_COUNT];
};

BUILD_ASSERT(sizeof(struct esb_hid_registry_disk_v1) == 73, "registry disk v1 must be 73 bytes");
BUILD_ASSERT(sizeof(struct esb_hid_registry_disk_v2) == 106, "registry disk v2 must be 106 bytes");

static struct esb_hid_registry registry;
static bool registry_from_settings;
static bool registry_loaded;
static bool registry_migrated;

static void registry_save(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    struct esb_hid_registry_disk_v2 disk = {.version = 2, .active_slot = registry.active_slot};

    for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
        disk.slots[i].used = registry.slots[i].used;
        memcpy(disk.slots[i].dev_id, registry.slots[i].dev_id, ESB_HID_DEVICE_ID_LEN);
        memcpy(disk.slots[i].name, registry.slots[i].name, ESB_HID_NAME_MAX + 1);
    }

    int err = settings_save_one("esb_hid/registry", &disk, sizeof(disk));

    if (err) {
        LOG_ERR("Failed to save ESB registry: %d", err);
    }
#endif
}

/* ------------------------------------------------------------------ */
/* USOver (US keyboard on a JP-layout host OS)                         */

/* Persisted USOver switch ("esb_hid/usover", one byte). Default off;
 * the receiver UI toggles it. Read at every keyboard report send, so
 * toggling applies to the very next report. The conversion itself
 * lives further down, next to the send path it rewrites. */
static bool usover_enabled;

static void usover_save(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    uint8_t v = usover_enabled ? 1 : 0;
    int err = settings_save_one("esb_hid/usover", &v, sizeof(v));

    if (err) {
        LOG_ERR("Failed to save USOver setting: %d", err);
    }
#endif
}

#if IS_ENABLED(CONFIG_SETTINGS)
static int esb_hid_rx_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                   void *cb_arg) {
    const char *next;

    if (settings_name_steq(name, "registry", &next) && !next) {
        if (len == sizeof(struct esb_hid_registry_disk_v1)) {
            struct esb_hid_registry_disk_v1 disk;
            int rc = read_cb(cb_arg, &disk, sizeof(disk));

            if (rc < 0) {
                return rc;
            }
            registry.active_slot = disk.active_slot;
            for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
                registry.slots[i].used = disk.slots[i].used;
                memset(registry.slots[i].dev_id, 0, ESB_HID_DEVICE_ID_LEN);
                memcpy(registry.slots[i].name, disk.slots[i].name, ESB_HID_NAME_MAX + 1);
            }
            registry_from_settings = true;
            registry_migrated = true;
            return 0;
        }
        if (len == sizeof(struct esb_hid_registry_disk_v2)) {
            struct esb_hid_registry_disk_v2 disk;
            int rc = read_cb(cb_arg, &disk, sizeof(disk));

            if (rc < 0) {
                return rc;
            }
            if (disk.version != 2) {
                return -EINVAL;
            }
            registry.active_slot = disk.active_slot;
            for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
                registry.slots[i].used = disk.slots[i].used;
                memcpy(registry.slots[i].dev_id, disk.slots[i].dev_id, ESB_HID_DEVICE_ID_LEN);
                memcpy(registry.slots[i].name, disk.slots[i].name, ESB_HID_NAME_MAX + 1);
            }
            registry_from_settings = true;
            return 0;
        }
        return -EINVAL;
    }
    if (settings_name_steq(name, "usover", &next) && !next) {
        if (len != 1) {
            return -EINVAL;
        }
        uint8_t v;
        int rc = read_cb(cb_arg, &v, sizeof(v));

        if (rc < 0) {
            return rc;
        }
        usover_enabled = (v != 0);
        return 0;
    }
    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(esb_hid_rx, "esb_hid", NULL, esb_hid_rx_settings_set, NULL, NULL);
#endif /* IS_ENABLED(CONFIG_SETTINGS) */

/* Load the registry once, from the deferred link-start path,
 * before any frame is attributed. With no saved registry the
 * receiver starts empty: slots are registered by pairing only. */
static void registry_load_once(void) {
    if (registry_loaded) {
        return;
    }
    registry_loaded = true;

#if IS_ENABLED(CONFIG_SETTINGS)
    /* ZMK's main() already ran settings_load() once; load our subtree
     * again here so the values are in place even if that ordering
     * ever changes. */
    settings_load_subtree("esb_hid");
#endif

    if (!registry_from_settings) {
        /* No saved registry: start empty. There used to be seeded
         * default names here (step 2: TentAlice/Nickey/koZakura
         * pre-registered with no device ID), but those name-keyed
         * placeholder entries are what made duplicate clearing by
         * name necessary — and name equality cannot tell two
         * physical keyboards of the same model apart. The seeds and
         * name-based clearing were removed together (2026-11-15);
         * a fresh receiver pairs its first keyboard through the
         * boot pairing window instead. */
        registry.active_slot = 0;
        LOG_INF("ESB registry: no saved registry, starting empty");
    }

    /* Names arriving from a stored blob are not trusted to be
     * NUL-terminated; force it so the strcmp/strlen users stay in
     * bounds even with a foreign or corrupted blob. */
    for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
        registry.slots[i].name[ESB_HID_NAME_MAX] = '\0';
    }

    if (registry.active_slot >= ESB_HID_SLOT_COUNT) {
        registry.active_slot = 0;
    }

    /* Repair a dangling active slot persisted by an older firmware
     * (deleting the active registration used to leave it pointing at
     * the now-unused entry, which silently disabled forwarding and
     * Studio for every other keyboard). */
    if (!registry.slots[registry.active_slot].used) {
        for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
            if (registry.slots[i].used) {
                LOG_INF("ESB registry: active slot repaired %d -> %d", registry.active_slot, i);
                registry.active_slot = (uint8_t)i;
                break;
            }
        }
    }

    if (registry_migrated && registry_from_settings) {
        LOG_INF("ESB registry: migrated v1 registry to v2, saving");
        registry_save();
    }
    LOG_INF("ESB registry: active slot %d, used [%d %d %d %d]", registry.active_slot,
            registry.slots[0].used, registry.slots[1].used, registry.slots[2].used,
            registry.slots[3].used);
    LOG_INF("USOver: %s", usover_enabled ? "on" : "off");
}

/* ------------------------------------------------------------------ */
/* Per-slot peer state                                                 */

#define ESB_HID_CACHE_BODY_MAX (CONFIG_ESB_MAX_PAYLOAD_LENGTH - ESB_HID_FRAME_HDR_LEN)

struct esb_hid_peer {
    struct esb_hid_device_status status;
    int64_t last_seen_ms; /* k_uptime_get() of last frame of any type; 0 = never */
    bool boot_armed;      /* BOOT-flag edge detection for the Studio flush */
    /* Last HID body per report type (keyboard/consumer/mouse), for
     * slot switching: release the old slot, re-inject the new one. */
    struct {
        uint8_t len;
        uint8_t body[ESB_HID_CACHE_BODY_MAX];
        bool valid;
    } hid_cache[3];
};

static struct esb_hid_peer peers[ESB_HID_SLOT_COUNT];

/* Slot identity self-check (DESIGN.md SLOT_IDENTITY, 2026-11-15):
 * the receiver periodically queues a SLOT_IDENTITY ACK payload on
 * each active slot pipe, telling the keyboard there which device is
 * registered at that slot, so a keyboard holding a stale profile can
 * notice and drop it. identity_dirty forces an early re-send after a
 * registry change for that slot. */
#define ESB_HID_IDENTITY_PERIOD_MS 10000
static int64_t identity_last_sent_ms[ESB_HID_SLOT_COUNT];
static bool identity_dirty[ESB_HID_SLOT_COUNT];
static uint8_t identity_seq;

/* Duplicate suppression, per pipe and frame type. Indexed by pipe so
 * the pairing pipe (7) gets its own row; pipes 4..6 are never enabled
 * but keep a row for safety. */
static uint8_t last_seq[8][ESB_HID_TYPE_COUNT];
static bool seq_valid[8][ESB_HID_TYPE_COUNT];

/* Forget everything the receiver holds about the device on a slot:
 * cached status and HID bodies, and the per-pipe duplicate
 * suppression for that pipe. Used when a slot's registration is
 * deleted or replaced, so no state of the previous device (held
 * keys in the cache, stale sequence numbers a successor could
 * collide with) leaks into the next one. Also re-arms the BOOT edge
 * and marks the slot identity for re-send. Callers hold
 * esb_hid_rx_mutex. */
static void peer_reset(uint8_t slot) {
    memset(&peers[slot], 0, sizeof(peers[slot]));
    peers[slot].boot_armed = true;
    for (int i = 0; i < ESB_HID_TYPE_COUNT; i++) {
        seq_valid[slot][i] = false;
    }
    identity_dirty[slot] = true;
}

bool esb_hid_get_device_status(uint8_t device_index, struct esb_hid_device_status *out) {
    if (device_index >= ESB_HID_SLOT_COUNT) {
        return false;
    }
    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (!peers[device_index].status.valid) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return false;
    }
    *out = peers[device_index].status;
    out->last_seen_ms = peers[device_index].last_seen_ms;
    k_mutex_unlock(&esb_hid_rx_mutex);
    return true;
}

uint8_t esb_hid_rx_studio_pipe(void) { return registry.active_slot; }

/* ------------------------------------------------------------------ */
/* Pairing window (receiver side)                                      */

#define ESB_HID_PAIRING_WINDOW_MS 60000
/* While ASSIGNING, the keyboard adopts the slot and stops sending
 * PAIR_REQUESTs; this much silence means it adopted. */
#define ESB_HID_PAIRING_ASSIGN_QUIET_MS 2000
/* Requests still arriving this long after confirm mean the keyboard
 * did not adopt (e.g. name mismatch): fall back to CANDIDATE. */
#define ESB_HID_PAIRING_ASSIGN_GIVEUP_MS 10000
#define ESB_HID_PAIRING_TIMER_PERIOD_MS 250
/* How long DONE lingers before the timer drops back to IDLE. The
 * Deck UI cancels DONE itself after ~1.5 s; this is for headless
 * receivers, where nothing else stops the pairing timer. */
#define ESB_HID_PAIRING_DONE_HOLD_MS 5000
/* Boot window decision delay after link start: long enough for the
 * previously active keyboard's 5 s status heartbeat to land twice. */
#define ESB_HID_BOOT_PAIRING_DELAY_MS 10000

static struct {
    enum esb_hid_pairing_state state;
    uint8_t target_slot;
    char candidate_name[ESB_HID_NAME_MAX + 1];
    uint8_t candidate_id[ESB_HID_DEVICE_ID_LEN];
    uint8_t candidate_id_len;
    int64_t deadline_ms;
    int64_t last_request_ms;
    int64_t assigning_since_ms;
    uint8_t seq;
} pairing;

static void pairing_timer_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(pairing_timer_work, pairing_timer_handler);

/* Own hardware device ID (Zephyr hwinfo, nRF FICR DEVICEID), fetched
 * lazily on first use and cached. own_id_len is ESB_HID_DEVICE_ID_LEN
 * when a full ID is available, 0 otherwise. Appended to PAIR_ASSIGN
 * payloads so multi-receiver keyboards can key their stored profile
 * by receiver identity (see esb_hid_frame.h). */
static uint8_t own_id[ESB_HID_DEVICE_ID_LEN];
static uint8_t own_id_len;
static bool own_id_fetched;

static uint8_t fetch_own_id(void) {
    if (!own_id_fetched) {
        own_id_fetched = true;
        ssize_t ret = hwinfo_get_device_id(own_id, sizeof(own_id));

        own_id_len = (ret == ESB_HID_DEVICE_ID_LEN) ? ESB_HID_DEVICE_ID_LEN : 0;
    }
    return own_id_len;
}

/* Queue one PAIR_ASSIGN ACK payload on the pairing pipe. The ESB
 * stack keeps ACK payloads in a per-pipe FIFO and attaches the head
 * to the ACK of the next packet received on that pipe, so exactly one
 * fresh payload is loaded after each pairing-pipe reception (the
 * previous one was consumed by it). Callers hold esb_hid_rx_mutex. */
static void pairing_load_payload(uint8_t slot, const char *name) {
    size_t name_len = (name != NULL) ? strlen(name) : 0;

    if (name_len > ESB_HID_NAME_MAX) {
        name_len = ESB_HID_NAME_MAX;
    }

    /* Trailer: our own device ID, so the keyboard can store this
     * assignment under this receiver's identity. Older keyboards
     * ignore the extra bytes; if no full ID is readable the payload
     * keeps its original (trailer-less) form. */
    uint8_t id_len = fetch_own_id();

    struct esb_payload payload = {
        .length = (uint8_t)(ESB_HID_FRAME_HDR_LEN + 2 + name_len + id_len),
        .pipe = ESB_HID_PAIRING_PIPE,
        .noack = 0,
    };
    payload.data[0] = ESB_HID_TYPE_PAIR_ASSIGN;
    payload.data[1] = pairing.seq++;
    payload.data[ESB_HID_FRAME_HDR_LEN] = slot;
    payload.data[ESB_HID_FRAME_HDR_LEN + 1] = (uint8_t)name_len;
    if (name_len > 0) {
        memcpy(&payload.data[ESB_HID_FRAME_HDR_LEN + 2], name, name_len);
    }
    if (id_len > 0) {
        memcpy(&payload.data[ESB_HID_FRAME_HDR_LEN + 2 + name_len], own_id, id_len);
    }

    int err = esb_write_payload(&payload);

    if (err) {
        LOG_WRN("Pairing: failed to load ACK payload: %d", err);
    }
}

static void pairing_finalize(void) {
    registry.slots[pairing.target_slot].used = 1;
    identity_dirty[pairing.target_slot] = true;
    strncpy(registry.slots[pairing.target_slot].name, pairing.candidate_name, ESB_HID_NAME_MAX);
    registry.slots[pairing.target_slot].name[ESB_HID_NAME_MAX] = '\0';
    memset(registry.slots[pairing.target_slot].dev_id, 0, ESB_HID_DEVICE_ID_LEN);
    if (pairing.candidate_id_len > 0) {
        memcpy(registry.slots[pairing.target_slot].dev_id, pairing.candidate_id,
               pairing.candidate_id_len);
    }

    /* The slot's device may have changed: drop the previous device's
     * peer state, so its cached held keys and status are never
     * re-injected or displayed as the new device's. */
    peer_reset(pairing.target_slot);

    /* Clear any duplicate registration of the same device in other
     * slots. Duplicates are recognised by hardware ID only: the
     * candidate must carry a full ID and it must match exactly.
     * Name equality is NOT used — two different keyboards can share
     * a model name, and silently clearing a stranger's registration
     * is worse than leaving a duplicate the user can see in the
     * list and delete by hand (policy decided 2026-11-15; this is
     * also what retired the seeded default names). */
    if (pairing.candidate_id_len == ESB_HID_DEVICE_ID_LEN) {
        for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
            if (i == pairing.target_slot || !registry.slots[i].used) {
                continue;
            }
            if (memcmp(registry.slots[i].dev_id, pairing.candidate_id,
                       ESB_HID_DEVICE_ID_LEN) == 0) {
                LOG_INF("Registry: \"%s\" moved slot %d -> %d (duplicate cleared)",
                        pairing.candidate_name, i, pairing.target_slot);
                registry.slots[i].used = 0;
                registry.slots[i].name[0] = '\0';
                memset(registry.slots[i].dev_id, 0, ESB_HID_DEVICE_ID_LEN);
                peer_reset(i);
                if (registry.active_slot == i && registry.active_slot != pairing.target_slot) {
                    registry.active_slot = pairing.target_slot;
                }
            }
        }
    }

    /* A dangling active slot (left behind by deleting the active
     * registration before this fix, or by an emptied registry) heals
     * here: the fresh registration becomes active. */
    if (!registry.slots[registry.active_slot].used) {
        registry.active_slot = pairing.target_slot;
    }

    registry_save();

    if (pairing.target_slot == registry.active_slot) {
        /* The device behind the Studio pipe was just (re)registered:
         * the session state belongs to its previous incarnation, so
         * drop it like a slot switch does. */
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
        esb_hid_studio_fwd_flush_session();
#endif
    }

    pairing.state = ESB_HID_PAIRING_DONE;
    /* DONE is a display state; give it a short deadline so the
     * pairing timer expires it back to IDLE instead of rescheduling
     * forever on receivers nobody cancels it on (headless builds). */
    pairing.deadline_ms = k_uptime_get() + ESB_HID_PAIRING_DONE_HOLD_MS;
    LOG_INF("Pairing: slot %d registered to \"%s\"", pairing.target_slot, pairing.candidate_name);
}

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING)
/* Set while the pairing window that was opened automatically at
 * boot (headless receiver) is live: its first candidate is assigned
 * without a manual confirm step. */
static bool boot_pairing_armed;
#endif

/* Caller holds esb_hid_rx_mutex and pairing.state ==
 * ESB_HID_PAIRING_CANDIDATE. */
static void pairing_confirm_locked(void) {
    pairing.state = ESB_HID_PAIRING_ASSIGNING;
    pairing.assigning_since_ms = k_uptime_get();
    pairing_load_payload(pairing.target_slot, pairing.candidate_name);
    LOG_INF("Pairing: assigning slot %d to \"%s\"", pairing.target_slot, pairing.candidate_name);
}

static void pairing_timer_handler(struct k_work *work) {
    ARG_UNUSED(work);

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (pairing.state == ESB_HID_PAIRING_IDLE) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return;
    }

    int64_t now = k_uptime_get();

    if (pairing.state == ESB_HID_PAIRING_ASSIGNING &&
        now - pairing.last_request_ms > ESB_HID_PAIRING_ASSIGN_QUIET_MS) {
        /* The keyboard stopped requesting: it adopted the slot. */
        pairing_finalize();
    } else if (pairing.state == ESB_HID_PAIRING_ASSIGNING &&
               now - pairing.assigning_since_ms > ESB_HID_PAIRING_ASSIGN_GIVEUP_MS) {
        LOG_WRN("Pairing: \"%s\" did not adopt slot %d, back to candidate",
                pairing.candidate_name, pairing.target_slot);
        pairing.state = ESB_HID_PAIRING_CANDIDATE;
        pairing_load_payload(ESB_HID_PAIR_SLOT_NOT_YET, NULL);
    } else if (now >= pairing.deadline_ms) {
        if (pairing.state == ESB_HID_PAIRING_DONE) {
            LOG_INF("Pairing: done, back to idle");
        } else {
            LOG_INF("Pairing: window timed out, back to idle");
        }
        pairing.state = ESB_HID_PAIRING_IDLE;
        pairing.candidate_name[0] = '\0';
        memset(pairing.candidate_id, 0, ESB_HID_DEVICE_ID_LEN);
        pairing.candidate_id_len = 0;
        k_mutex_unlock(&esb_hid_rx_mutex);
        return;
    }

    k_mutex_unlock(&esb_hid_rx_mutex);
    k_work_reschedule(&pairing_timer_work, K_MSEC(ESB_HID_PAIRING_TIMER_PERIOD_MS));
}

/* A PAIR_REQUEST arrived on the pairing pipe. Caller holds
 * esb_hid_rx_mutex. */
static void pairing_on_request(const uint8_t *body, size_t len) {
    if (len < 2) {
        return;
    }

    /* Parse per proto version with strict length validation.
     * v1: [proto=1][name_len][name], candidate id_len = 0.
     * v2: [proto=2][id_len][id bytes][name_len][name]. */
    uint8_t candidate_id[ESB_HID_DEVICE_ID_LEN] = {0};
    uint8_t candidate_id_len = 0;
    size_t name_offset;

    if (body[0] == 1) {
        name_offset = 2;
    } else if (body[0] == 2) {
        candidate_id_len = body[1];
        if (candidate_id_len > ESB_HID_DEVICE_ID_LEN) {
            LOG_WRN("Pairing: ignoring request with id_len %d", candidate_id_len);
            return;
        }
        if (len < (size_t)(2 + candidate_id_len + 1)) {
            return;
        }
        memcpy(candidate_id, &body[2], candidate_id_len);
        name_offset = 3 + candidate_id_len;
    } else {
        LOG_WRN("Pairing: ignoring request with proto version %d", body[0]);
        return;
    }
    if (pairing.state == ESB_HID_PAIRING_IDLE || pairing.state == ESB_HID_PAIRING_DONE) {
        /* No window open: nothing to do (no ACK payload is queued
         * for this pipe either, so the keyboard gets empty ACKs and
         * gives up on its own after 30 s). */
        return;
    }

    size_t name_len = body[name_offset - 1];

    if (name_len > len - name_offset) {
        name_len = len - name_offset;
    }
    if (name_len > ESB_HID_NAME_MAX) {
        name_len = ESB_HID_NAME_MAX;
    }

    /* Once a candidate exists, only that keyboard's own requests
     * count. A request from a different keyboard must neither
     * replace the candidate the user is looking at (they would
     * confirm one keyboard and register another) nor refresh
     * last_request_ms (the ASSIGNING silence check would never
     * elapse while a second keyboard nearby keeps asking). The
     * match is by hardware ID when the candidate carries one,
     * otherwise by name — an ID-less (proto v1) keyboard has no
     * stronger identity to check against. */
    if (pairing.state == ESB_HID_PAIRING_CANDIDATE ||
        pairing.state == ESB_HID_PAIRING_ASSIGNING) {
        bool matches;

        if (pairing.candidate_id_len > 0) {
            matches = candidate_id_len == pairing.candidate_id_len &&
                      memcmp(candidate_id, pairing.candidate_id, candidate_id_len) == 0;
        } else {
            matches = name_len == strlen(pairing.candidate_name) &&
                      memcmp(&body[name_offset], pairing.candidate_name, name_len) == 0;
        }
        if (!matches) {
            return;
        }
    }

    pairing.last_request_ms = k_uptime_get();

    if (pairing.state == ESB_HID_PAIRING_SEARCHING ||
        pairing.state == ESB_HID_PAIRING_CANDIDATE) {
        memset(pairing.candidate_id, 0, ESB_HID_DEVICE_ID_LEN);
        memcpy(pairing.candidate_id, candidate_id, candidate_id_len);
        pairing.candidate_id_len = candidate_id_len;
        memcpy(pairing.candidate_name, &body[name_offset], name_len);
        pairing.candidate_name[name_len] = '\0';
        if (pairing.state == ESB_HID_PAIRING_SEARCHING) {
            pairing.state = ESB_HID_PAIRING_CANDIDATE;
            LOG_INF("Pairing: candidate \"%s\" for slot %d", pairing.candidate_name,
                    pairing.target_slot);
        }
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING_AUTO)
        if (boot_pairing_armed) {
            /* Boot window in auto mode (headless): there is no UI
             * to confirm with, so the first candidate is assigned
             * immediately. With a UI (AUTO off) the candidate just
             * stays pending until the UI confirms it. */
            pairing_confirm_locked();
            return;
        }
#endif
        /* Sentinel reload: tell the keyboard "not yet". */
        pairing_load_payload(ESB_HID_PAIR_SLOT_NOT_YET, NULL);
    } else if (pairing.state == ESB_HID_PAIRING_ASSIGNING) {
        /* Keep the assignment in front of the keyboard until it
         * adopts it (and goes quiet) or the give-up timer fires. */
        pairing_load_payload(pairing.target_slot, pairing.candidate_name);
    }
}

int esb_hid_pairing_open(uint8_t target_slot) {
    if (target_slot >= ESB_HID_SLOT_COUNT) {
        return -EINVAL;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (pairing.state != ESB_HID_PAIRING_IDLE && pairing.state != ESB_HID_PAIRING_DONE) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return -EBUSY;
    }

    pairing.state = ESB_HID_PAIRING_SEARCHING;
    pairing.target_slot = target_slot;
    pairing.candidate_name[0] = '\0';
    memset(pairing.candidate_id, 0, ESB_HID_DEVICE_ID_LEN);
    pairing.candidate_id_len = 0;
    pairing.deadline_ms = k_uptime_get() + ESB_HID_PAIRING_WINDOW_MS;
    pairing.last_request_ms = 0;
    pairing_load_payload(ESB_HID_PAIR_SLOT_NOT_YET, NULL);
    k_work_reschedule(&pairing_timer_work, K_MSEC(ESB_HID_PAIRING_TIMER_PERIOD_MS));
    k_mutex_unlock(&esb_hid_rx_mutex);

    LOG_INF("Pairing: window opened for slot %d (60 s)", target_slot);
    return 0;
}

int esb_hid_pairing_get_status(struct esb_hid_pairing_status *out) {
    if (out == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    out->state = pairing.state;
    out->target_slot = pairing.target_slot;
    strncpy(out->candidate_name, pairing.candidate_name, ESB_HID_NAME_MAX);
    out->candidate_name[ESB_HID_NAME_MAX] = '\0';
    if (pairing.state == ESB_HID_PAIRING_IDLE) {
        out->remaining_ms = 0;
    } else {
        int64_t left = pairing.deadline_ms - k_uptime_get();
        out->remaining_ms = (int32_t)(left > 0 ? left : 0);
    }
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

int esb_hid_pairing_confirm(void) {
    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (pairing.state != ESB_HID_PAIRING_CANDIDATE) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return -EINVAL;
    }

    pairing_confirm_locked();
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

void esb_hid_pairing_cancel(void) {
    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    pairing.state = ESB_HID_PAIRING_IDLE;
    pairing.candidate_name[0] = '\0';
    memset(pairing.candidate_id, 0, ESB_HID_DEVICE_ID_LEN);
    pairing.candidate_id_len = 0;
    k_work_cancel_delayable(&pairing_timer_work);
    k_mutex_unlock(&esb_hid_rx_mutex);
    LOG_INF("Pairing: cancelled");
}

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING)
/* Headless receivers have no UI to open the pairing window with, so
 * open it once the link is up: target the first free slot; if every
 * slot is already registered there is nothing to do. The decision is
 * deferred (ESB_HID_BOOT_PAIRING_DELAY_MS after link start) and is
 * skipped entirely when the keyboard this receiver was last used
 * with is already on the air: that is a normal power-on, not a
 * pairing situation. Runs once. */
static void boot_pairing_maybe_open(void) {
    static bool attempted;

    if (attempted) {
        return;
    }
    attempted = true;

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    /* Suppression means "the keyboard this receiver is used with is
     * already on the air" — that only holds while the active slot
     * is still registered. Peer state is updated for every pipe,
     * registered or not, so after the active registration was
     * deleted, the deleted keyboard's ongoing frames would
     * otherwise suppress the window forever (and a headless
     * receiver would have no way back in). */
    if (registry.active_slot < ESB_HID_SLOT_COUNT && registry.slots[registry.active_slot].used &&
        peers[registry.active_slot].last_seen_ms != 0) {
        LOG_INF("Pairing: active keyboard already on air, no boot window");
        k_mutex_unlock(&esb_hid_rx_mutex);
        return;
    }

    int free_slot = -1;

    for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
        if (!registry.slots[i].used) {
            free_slot = i;
            break;
        }
    }
    k_mutex_unlock(&esb_hid_rx_mutex);

    if (free_slot >= 0) {
        boot_pairing_armed = true;
        if (esb_hid_pairing_open(free_slot) == 0) {
            LOG_INF("Pairing: boot window opened for free slot %d (%s)", free_slot,
                    IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING_AUTO)
                        ? "auto-assign"
                        : "waiting for on-device confirm");
        } else {
            boot_pairing_armed = false;
        }
        return;
    }
    LOG_INF("Pairing: all slots registered, no boot window (headless)");
}

static void boot_pairing_work_handler(struct k_work *work) {
    ARG_UNUSED(work);
    boot_pairing_maybe_open();
}

static K_WORK_DELAYABLE_DEFINE(boot_pairing_work, boot_pairing_work_handler);
#endif

/* ------------------------------------------------------------------ */
/* Public slot API                                                     */

uint8_t esb_hid_rx_get_active_slot(void) { return registry.active_slot; }

static const uint8_t report_ids[] = {
    ZMK_HID_REPORT_ID_KEYBOARD,
    ZMK_HID_REPORT_ID_CONSUMER,
    ZMK_HID_REPORT_ID_MOUSE,
};

/* USOver conversion table: (trigger usage, trigger needs shift) ->
 * (replacement usage, replacement needs shift). This is the Keyboard
 * Quantizer's us_key_on_jp_os_overrides list (its UJ OVR mode)
 * transcribed to HID usages, with the JP_* keycodes resolved through
 * QMK's keymap_japanese.h. The host OS is set to the JP layout while
 * the keyboard sends US usages; each entry rewrites one punctuation
 * key so the character printed on the keycap comes out. Unshifted
 * GRV is deliberately NOT overridden: the JP host OS already treats
 * that usage as the half/full-width (Zenkaku/Hankaku) key, and the
 * user wants the IME toggle kept (the Quantizer's UJ HW variant
 * makes the same choice). Shifted GRV still maps to ~ above. */
struct usover_entry {
    uint8_t trigger;
    bool trigger_shift;
    uint8_t replacement;
    bool replacement_shift;
};

static const struct usover_entry usover_table[] = {
    {0x35, true, 0x2E, true},   /* ~ -> JP_TILD */
    {0x1F, true, 0x2F, false},  /* @ -> JP_AT */
    {0x23, true, 0x2E, false},  /* ^ -> JP_CIRC */
    {0x24, true, 0x23, true},   /* & -> JP_AMPR */
    {0x25, true, 0x34, true},   /* * -> JP_ASTR */
    {0x26, true, 0x25, true},   /* ( -> JP_LPRN */
    {0x27, true, 0x26, true},   /* ) -> JP_RPRN */
    {0x2D, true, 0x87, true},   /* _ -> JP_UNDS */
    {0x2E, false, 0x2D, true},  /* = -> JP_EQL */
    {0x2E, true, 0x33, true},   /* + -> JP_PLUS */
    {0x2F, false, 0x30, false}, /* [ -> JP_LBRC */
    {0x2F, true, 0x30, true},   /* { -> JP_LCBR */
    {0x30, false, 0x32, false}, /* ] -> JP_RBRC */
    {0x30, true, 0x32, true},   /* } -> JP_RCBR */
    {0x31, false, 0x87, false}, /* backslash -> JP_BSLS */
    {0x31, true, 0x89, true},   /* | -> JP_PIPE */
    {0x33, true, 0x34, false},  /* : -> JP_COLN */
    {0x34, false, 0x24, true},  /* ' -> JP_QUOT */
    {0x34, true, 0x1F, true},   /* " -> JP_DQUO */
};

#define USOVER_MOD_LSHIFT 0x02
#define USOVER_MOD_RSHIFT 0x20
#define USOVER_MOD_SHIFT_MASK (USOVER_MOD_LSHIFT | USOVER_MOD_RSHIFT)
/* Keyboard bodies are a modifier byte plus the NKRO bitmap (20 bytes
 * at the largest usage ZMK reports); anything longer is not a plain
 * keyboard body and is passed through untransformed. */
#define USOVER_BODY_MAX 32

/* The key bitmap in a ZMK keyboard report body starts after the
 * modifier byte AND a reserved byte (zmk_hid_keyboard_report_body:
 * modifiers, _reserved, keys[]). Indexing from byte 1 instead shifts
 * every conversion by 8 usages — that was the first USOver bug. */
#define USOVER_BITMAP_OFFSET 2

/* Rewrite a keyboard report body (modifier byte + NKRO bitmap) per
 * the USOver table. Triggers are matched against a snapshot of the
 * original bitmap, so a replacement usage that is itself a trigger
 * (e.g. [ -> ]) is never rewritten a second time. If several
 * overridden keys are held at once and their replacements disagree
 * on shift, the last match wins the shift bits — the compromise a
 * single-report forwarder has to make; letters, digits and
 * unshifted keys are unaffected. */
static void usover_transform(uint8_t *body, size_t len) {
    if (len < 2 || len > USOVER_BODY_MAX) {
        return;
    }
    uint8_t orig[USOVER_BODY_MAX];

    memcpy(orig, body, len);
    bool shift = (orig[0] & USOVER_MOD_SHIFT_MASK) != 0;
    bool applied = false;
    bool out_shift = shift;

    for (size_t i = 0; i < ARRAY_SIZE(usover_table); i++) {
        const struct usover_entry *e = &usover_table[i];
        size_t tbyte = USOVER_BITMAP_OFFSET + e->trigger / 8;

        if (tbyte >= len || !(orig[tbyte] & BIT(e->trigger % 8)) ||
            e->trigger_shift != shift) {
            continue;
        }
        body[tbyte] &= (uint8_t)~BIT(e->trigger % 8);
        size_t rbyte = USOVER_BITMAP_OFFSET + e->replacement / 8;

        if (rbyte < len) {
            body[rbyte] |= (uint8_t)BIT(e->replacement % 8);
        }
        applied = true;
        out_shift = e->replacement_shift;
    }
    if (applied) {
        body[0] = (body[0] & (uint8_t)~USOVER_MOD_SHIFT_MASK) |
                  (out_shift ? USOVER_MOD_LSHIFT : 0);
    }
}

/* The one choke point for HID reports on their way to the host:
 * live forwarding, slot-switch re-injection and release all pass
 * through here, so USOver applies uniformly. Callers hold
 * esb_hid_rx_mutex. */
static int rx_send_hid(uint8_t type, const uint8_t *body, size_t len) {
    uint8_t buf[ESB_HID_CACHE_BODY_MAX];

    if (type == ESB_HID_TYPE_KEYBOARD && len <= sizeof(buf)) {
        /* The report this receiver declares to the host is its own,
         * and a transmitter's body can be shorter (a build without
         * the extended NKRO usages sends 19 bytes where this build
         * declares 21): zero-pad to this build's body size so the
         * declared bitmap is always sent in full — the USOver
         * replacements at INT1/INT3 live in those tail bytes. */
        size_t send_len = len;

        if (send_len < sizeof(struct zmk_hid_keyboard_report_body)) {
            send_len = sizeof(struct zmk_hid_keyboard_report_body);
        }
        memset(buf, 0, send_len);
        memcpy(buf, body, len);
        if (usover_enabled) {
            usover_transform(buf, send_len);
        }
        body = buf;
        len = send_len;
    }
    return zmk_usb_hid_send_report_with_id(report_ids[type], body, len);
}

int esb_hid_rx_set_active_slot(uint8_t slot) {
    if (slot >= ESB_HID_SLOT_COUNT) {
        return -EINVAL;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (!registry.slots[slot].used) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return -EINVAL;
    }
    if (slot == registry.active_slot) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return 0;
    }

    uint8_t old_slot = registry.active_slot;

    /* (a) Release everything the old slot's device may still hold:
     * send a zeroed body for each HID type it ever sent. */
    for (int type = 0; type < 3; type++) {
        if (peers[old_slot].hid_cache[type].valid) {
            uint8_t zeroed[ESB_HID_CACHE_BODY_MAX] = {0};

            rx_send_hid(type, zeroed,
                                            peers[old_slot].hid_cache[type].len);
        }
    }

    /* (b) The Studio session belongs to the old device: flush it.
     * This also purges the radio's ACK payload FIFO on all pipes, so
     * no stale downlink fragment can reach the new device. */
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
    esb_hid_studio_fwd_flush_session();
#endif

    /* (c) Switch and persist. Studio ACK payloads follow the active
     * slot automatically via esb_hid_rx_studio_pipe(). */
    registry.active_slot = slot;
    registry_save();

    /* (d) Re-inject the new slot's held keyboard/consumer state so
     * keys it is physically holding keep working. Mouse is relative
     * and is never re-injected. */
    for (int type = 0; type < 2; type++) {
        if (!peers[slot].hid_cache[type].valid) {
            continue;
        }
        bool nonzero = false;

        for (int i = 0; i < peers[slot].hid_cache[type].len; i++) {
            if (peers[slot].hid_cache[type].body[i] != 0) {
                nonzero = true;
                break;
            }
        }
        if (nonzero) {
            rx_send_hid(type, peers[slot].hid_cache[type].body,
                                            peers[slot].hid_cache[type].len);
        }
    }

    LOG_INF("Active slot: %d -> %d (\"%s\")", old_slot, slot, registry.slots[slot].name);
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

bool esb_hid_rx_usover_enabled(void) { return usover_enabled; }

int esb_hid_rx_usover_set(bool enabled) {
    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    if (enabled == usover_enabled) {
        k_mutex_unlock(&esb_hid_rx_mutex);
        return 0;
    }
    usover_enabled = enabled;
    usover_save();
    LOG_INF("USOver: %s", enabled ? "on" : "off");

    /* Re-issue the active keyboard's held state under the new mode:
     * release everything first (an all-zero body releases in any
     * mapping), then re-inject the cached body through the send
     * path so it is transformed under the new setting. Without this,
     * a key held across the toggle would stay stuck in the mapping
     * it was pressed with. */
    uint8_t active = registry.active_slot;

    if (active < ESB_HID_SLOT_COUNT && registry.slots[active].used &&
        peers[active].hid_cache[ESB_HID_TYPE_KEYBOARD].valid) {
        size_t len = peers[active].hid_cache[ESB_HID_TYPE_KEYBOARD].len;
        uint8_t zeroed[ESB_HID_CACHE_BODY_MAX] = {0};
        bool nonzero = false;

        for (size_t i = 0; i < len; i++) {
            if (peers[active].hid_cache[ESB_HID_TYPE_KEYBOARD].body[i] != 0) {
                nonzero = true;
                break;
            }
        }
        rx_send_hid(ESB_HID_TYPE_KEYBOARD, zeroed, len);
        if (nonzero) {
            rx_send_hid(ESB_HID_TYPE_KEYBOARD,
                        peers[active].hid_cache[ESB_HID_TYPE_KEYBOARD].body, len);
        }
    }
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

int esb_hid_rx_get_slot_info(uint8_t slot, struct esb_hid_slot_info *out) {
    if (slot >= ESB_HID_SLOT_COUNT || out == NULL) {
        return -EINVAL;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    out->used = registry.slots[slot].used != 0;
    strncpy(out->name, registry.slots[slot].name, ESB_HID_NAME_MAX);
    out->name[ESB_HID_NAME_MAX] = '\0';
    if (out->name[0] == '\0' && peers[slot].status.valid) {
        /* Unregistered slot: fall back to the broadcast name so the
         * UI can still show who is talking. */
        strncpy(out->name, peers[slot].status.name, ESB_HID_NAME_MAX);
        out->name[ESB_HID_NAME_MAX] = '\0';
    }
    if (peers[slot].status.valid) {
        strncpy(out->layer_name, peers[slot].status.layer_name, ESB_HID_LAYER_NAME_MAX);
        out->layer_name[ESB_HID_LAYER_NAME_MAX] = '\0';
    } else {
        out->layer_name[0] = '\0';
    }
    out->status_valid = peers[slot].status.valid;
    out->battery = peers[slot].status.valid ? peers[slot].status.battery : 0xFF;
    out->battery_peer = peers[slot].status.valid ? peers[slot].status.battery_peer : 0xFF;
    out->layer = peers[slot].status.layer;
    out->output = peers[slot].status.output;
    if (peers[slot].last_seen_ms == 0) {
        out->link_up = false;
        out->last_seen_age_ms = -1;
    } else {
        int64_t age = k_uptime_get() - peers[slot].last_seen_ms;

        out->last_seen_age_ms = age;
        out->link_up = age < ESB_HID_LINK_LOST_TIMEOUT_MS;
    }
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

int esb_hid_rx_delete_slot(uint8_t slot) {
    if (slot >= ESB_HID_SLOT_COUNT) {
        return -EINVAL;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    bool was_active = (slot == registry.active_slot);

    if (was_active) {
        /* Release everything this device may still hold on the host
         * (same contract as set_active_slot): it will not get a
         * chance to send the releases itself once unregistered. */
        for (int type = 0; type < 3; type++) {
            if (peers[slot].hid_cache[type].valid) {
                uint8_t zeroed[ESB_HID_CACHE_BODY_MAX] = {0};

                rx_send_hid(type, zeroed,
                                                peers[slot].hid_cache[type].len);
            }
        }
    }
    registry.slots[slot].used = 0;
    registry.slots[slot].name[0] = '\0';
    memset(registry.slots[slot].dev_id, 0, ESB_HID_DEVICE_ID_LEN);
    peer_reset(slot);
    if (was_active) {
        /* The active slot must not dangle at an unused entry: the
         * forwarding gate and the Studio pipe both follow it, so a
         * dangling active silently kills input and Studio for every
         * remaining keyboard. Hand active to the first remaining
         * registered slot; if none is left, pairing_finalize adopts
         * the next registration as active. */
        for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
            if (registry.slots[i].used) {
                registry.active_slot = (uint8_t)i;
                break;
            }
        }
    }
    registry_save();
    if (was_active) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
        /* The Studio session belonged to the deleted device: flush
         * it (this also purges the queued ACK payloads on all
         * pipes), so no downlink fragment addressed to it can reach
         * the successor on the Studio pipe — the same contract
         * set_active_slot keeps when switching away from a device. */
        esb_hid_studio_fwd_flush_session();
#endif
    }
    LOG_INF("ESB registry: slot %d deleted", slot);
    k_mutex_unlock(&esb_hid_rx_mutex);
    return 0;
}

/* ------------------------------------------------------------------ */
/* Frame handling                                                      */

static void handle_status(uint8_t slot, const uint8_t *data, size_t len) {
    if (len < sizeof(struct esb_hid_status_body)) {
        LOG_WRN("Malformed status frame (len %d)", (int)len);
        return;
    }

    const struct esb_hid_status_body *body = (const struct esb_hid_status_body *)data;
    struct esb_hid_peer *peer = &peers[slot];

    uint8_t output = (uint8_t)((body->flags & ESB_HID_STATUS_FLAG_OUTPUT_MASK) >>
                               ESB_HID_STATUS_FLAG_OUTPUT_SHIFT);

    /* Status v2 name trailer: [name_len][name] after the fixed body.
     * Status v3 layer-name trailer: [layer_name_len][layer_name]
     * after the v2 name trailer. Status v4 peer battery trailer:
     * 1 byte after the layer-name trailer. */
    char name[ESB_HID_NAME_MAX + 1] = "";
    char layer_name[ESB_HID_LAYER_NAME_MAX + 1] = "";
    size_t trailer_offset = sizeof(struct esb_hid_status_body);

    if (body->proto_version >= 2 && len > sizeof(struct esb_hid_status_body)) {
        size_t wire_name_len = data[sizeof(struct esb_hid_status_body)];
        size_t name_len = wire_name_len;

        if (name_len > len - sizeof(struct esb_hid_status_body) - 1) {
            name_len = len - sizeof(struct esb_hid_status_body) - 1;
        }
        if (name_len > ESB_HID_NAME_MAX) {
            name_len = ESB_HID_NAME_MAX;
        }
        memcpy(name, &data[sizeof(struct esb_hid_status_body) + 1], name_len);
        name[name_len] = '\0';
        trailer_offset += 1 + wire_name_len;
        if (trailer_offset > len) {
            trailer_offset = len;
        }
    }

    if (body->proto_version >= 3 && trailer_offset < len) {
        size_t wire_layer_name_len = data[trailer_offset];
        size_t layer_name_len = wire_layer_name_len;

        if (layer_name_len > len - trailer_offset - 1) {
            layer_name_len = len - trailer_offset - 1;
        }
        if (layer_name_len > ESB_HID_LAYER_NAME_MAX) {
            layer_name_len = ESB_HID_LAYER_NAME_MAX;
        }
        memcpy(layer_name, &data[trailer_offset + 1], layer_name_len);
        layer_name[layer_name_len] = '\0';
        trailer_offset += 1 + wire_layer_name_len;
        if (trailer_offset > len) {
            trailer_offset = len;
        }
    }

    /* Status v4 peer battery trailer: the split peer's state of
     * charge, ESB_HID_BATTERY_UNKNOWN when not split / unknown. */
    uint8_t battery_peer = ESB_HID_BATTERY_UNKNOWN;

    if (body->proto_version >= 4 && trailer_offset < len) {
        battery_peer = data[trailer_offset];
    }

    if (!peer->status.valid || peer->status.battery != body->battery ||
        peer->status.battery_peer != battery_peer ||
        peer->status.layer != body->layer || peer->status.output != output ||
        strcmp(peer->status.name, name) != 0 ||
        strcmp(peer->status.layer_name, layer_name) != 0) {
        if (body->battery == ESB_HID_BATTERY_UNKNOWN) {
            LOG_INF("Peer %d status: battery unknown, layer %d, output %d", slot, body->layer,
                    output);
        } else {
            LOG_INF("Peer %d status: battery %d%%, layer %d, output %d", slot, body->battery,
                    body->layer, output);
        }
    }

    peer->status.battery = body->battery;
    peer->status.battery_peer = battery_peer;
    peer->status.layer = body->layer;
    peer->status.flags = body->flags;
    peer->status.output = output;
    peer->status.proto_version = body->proto_version;
    strncpy(peer->status.name, name, ESB_HID_NAME_MAX);
    peer->status.name[ESB_HID_NAME_MAX] = '\0';
    strncpy(peer->status.layer_name, layer_name, ESB_HID_LAYER_NAME_MAX);
    peer->status.layer_name[ESB_HID_LAYER_NAME_MAX] = '\0';
    peer->status.last_update_ms = k_uptime_get();
    peer->status.valid = true;

    /* Edge-detect the keyboard's BOOT flag: it is set for the first
     * few status frames after a keyboard reboot and clear afterwards,
     * so a set frame while armed means "new boot". The Studio session
     * state held for the previous boot is stale then and would poison
     * the fresh session, so flush it — but only when the booting
     * device is the active one: the Studio session belongs to the
     * active device, and an inactive device rebooting must not tear
     * down the active device's session. The duplicate suppression for
     * this slot restarts either way: its seq counters restarted at 0,
     * and a fresh frame whose seq happens to equal the stale last_seq
     * must not be dropped as a duplicate. */
    if (body->flags & ESB_HID_STATUS_FLAG_BOOT) {
        if (peer->boot_armed) {
            peer->boot_armed = false;
            for (int i = 0; i < ESB_HID_TYPE_COUNT; i++) {
                seq_valid[slot][i] = false;
            }
            if (slot == registry.active_slot) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
                esb_hid_studio_fwd_flush_session();
#endif
                LOG_INF("Peer %d reboot detected, receiver session state flushed", slot);
            } else {
                LOG_INF("Peer %d reboot detected (inactive slot, no flush)", slot);
            }
        }
    } else {
        peer->boot_armed = true;
    }
}

/* Link indicator: blink the board's green user LED (devicetree alias
 * led1) for as long as the link is alive. Any accepted frame refreshes
 * the liveness timestamp; the TX status heartbeat (every 5 s) keeps
 * the link marked alive while a keyboard is on, so this blinks
 * continuously during normal use and stops within the timeout after
 * the TX goes away. Boards without the alias build without it. */
#if DT_NODE_EXISTS(DT_ALIAS(led1)) && DT_NODE_EXISTS(DT_ALIAS(led0))
#define ESB_HID_LINK_LED 1

#define ESB_HID_LINK_ALIVE_MS 45000
#define ESB_HID_LINK_BLINK_MS 250

static const struct gpio_dt_spec link_led = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec link_led_red = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static int64_t link_last_frame_ms;
static bool link_led_on;
static int rx_init_err;

static void link_blink(struct k_work *work) {
    if (rx_init_err != 0) {
        /* Link start failed: blink |err| in red as two digit groups
         * (tens, gap, ones), repeating — same code as the TX diag. */
        int code = rx_init_err < 0 ? -rx_init_err : rx_init_err;
        int tens = code / 10;
        int ones = code % 10;
        int cycle = 2 * tens + 3 + 2 * ones + 8;
        static int slot;

        int s = slot++ % cycle;
        bool on = false;

        if (s < 2 * tens) {
            on = (s % 2) == 0;
        } else if (s >= 2 * tens + 3 && s < 2 * tens + 3 + 2 * ones) {
            on = ((s - (2 * tens + 3)) % 2) == 0;
        }
        gpio_pin_set_dt(&link_led_red, on ? 1 : 0);
        gpio_pin_set_dt(&link_led, 0);
        k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(200));
        return;
    }

    bool alive = link_last_frame_ms > 0 &&
                 (k_uptime_get() - link_last_frame_ms) < ESB_HID_LINK_ALIVE_MS;

    link_led_on = alive && !link_led_on;
    gpio_pin_set_dt(&link_led, link_led_on ? 1 : 0);

    k_work_reschedule(k_work_delayable_from_work(work), K_MSEC(ESB_HID_LINK_BLINK_MS));
}

static K_WORK_DELAYABLE_DEFINE(link_blink_work, link_blink);

static void link_led_note_frame(void) { link_last_frame_ms = k_uptime_get(); }
#else
#define ESB_HID_LINK_LED 0
static void link_led_note_frame(void) {}
#endif

static void esb_hid_rx_event_handler(const struct esb_evt *evt) {
    if (evt->evt_id == ESB_EVENT_RX_RECEIVED) {
        k_sem_give(&rx_sem);
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
        /* Receiving a packet consumed one queued ACK payload; the
         * Studio forwarder may be able to load the next fragment. */
        esb_hid_studio_fwd_note_link_event();
#endif
    }
}

/* Queue one SLOT_IDENTITY ACK payload on a slot pipe. Callers hold
 * esb_hid_rx_mutex (handle_payload context). A failed write (ACK
 * FIFO full) leaves the bookkeeping untouched so the next received
 * frame retries; Studio fragments keep priority by volume alone —
 * identity payloads are one small frame per pipe per period. */
static void identity_load_payload(uint8_t slot) {
    struct esb_payload payload = {
        .length = ESB_HID_FRAME_HDR_LEN + 1 + ESB_HID_DEVICE_ID_LEN,
        .pipe = slot,
        .noack = 0,
    };
    payload.data[0] = ESB_HID_TYPE_SLOT_IDENTITY;
    payload.data[1] = identity_seq++;
    payload.data[ESB_HID_FRAME_HDR_LEN] = registry.slots[slot].used
                                              ? ESB_HID_SLOT_IDENTITY_REGISTERED
                                              : 0;
    memcpy(&payload.data[ESB_HID_FRAME_HDR_LEN + 1], registry.slots[slot].dev_id,
           ESB_HID_DEVICE_ID_LEN);

    if (esb_write_payload(&payload) == 0) {
        identity_last_sent_ms[slot] = k_uptime_get();
        identity_dirty[slot] = false;
    }
}

static void identity_maybe_load(uint8_t slot) {
    if (identity_dirty[slot] || identity_last_sent_ms[slot] == 0 ||
        k_uptime_get() - identity_last_sent_ms[slot] >= ESB_HID_IDENTITY_PERIOD_MS) {
        identity_load_payload(slot);
    }
}

static void handle_payload(const struct esb_payload *payload) {
    if (payload->length < ESB_HID_FRAME_HDR_LEN) {
        return;
    }

    uint8_t type = payload->data[0];
    uint8_t seq = payload->data[1];
    uint8_t pipe = payload->pipe;

    if (type >= ESB_HID_TYPE_COUNT || pipe >= 8) {
        return;
    }

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);

    /* Keyboard/consumer/status are state: drop exact duplicates
     * (e.g. a re-delivered frame after a lost ACK). Mouse is relative;
     * every frame carries new motion, so forward all of them. Dedup
     * is per pipe, so one device's seq never suppresses another's. */
    if (type != ESB_HID_TYPE_MOUSE) {
        if (seq_valid[pipe][type] && seq == last_seq[pipe][type]) {
            k_mutex_unlock(&esb_hid_rx_mutex);
            return;
        }
        seq_valid[pipe][type] = true;
        last_seq[pipe][type] = seq;
    }

    const uint8_t *body = &payload->data[ESB_HID_FRAME_HDR_LEN];
    size_t body_len = payload->length - ESB_HID_FRAME_HDR_LEN;

    if (pipe == ESB_HID_PAIRING_PIPE) {
        /* Pairing pipe: only PAIR_REQUEST is meaningful here. */
        if (type == ESB_HID_TYPE_PAIR_REQUEST) {
            pairing_on_request(body, body_len);
        }
        k_mutex_unlock(&esb_hid_rx_mutex);
        return;
    }

    if (pipe >= ESB_HID_SLOT_COUNT) {
        /* Pipes 4..6 are not enabled; ignore defensively. */
        k_mutex_unlock(&esb_hid_rx_mutex);
        return;
    }

    uint8_t slot = pipe;
    struct esb_hid_peer *peer = &peers[slot];

    peer->last_seen_ms = k_uptime_get();
    link_led_note_frame();

    bool forward = (slot == registry.active_slot) && registry.slots[slot].used;

    switch (type) {
    case ESB_HID_TYPE_STUDIO:
        /* Studio uplink likewise only from the active slot. */
        if (forward) {
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_STUDIO_RX)
            esb_hid_studio_fwd_to_pc(body, body_len);
#endif
        }
        break;
    case ESB_HID_TYPE_STUDIO_HINT:
        /* Hints only ever travel as ACK payloads (RX->TX). One
         * arriving as a regular uplink frame is a protocol error;
         * never forward it to USB HID. */
        break;
    case ESB_HID_TYPE_POLL:
        /* Polls exist only to pull ACK payloads; nothing to forward.
         * (The ACK refill itself is triggered from the RX event
         * handler, which fires for polls as well.) */
        break;
    case ESB_HID_TYPE_STATUS:
        handle_status(slot, body, body_len);
        break;
    case ESB_HID_TYPE_KEYBOARD:
    case ESB_HID_TYPE_CONSUMER:
    case ESB_HID_TYPE_MOUSE: {
        /* Cache the latest body per (slot, type) for slot switching,
         * whether or not this slot is currently forwarded. */
        size_t cache_len = body_len;

        if (cache_len > ESB_HID_CACHE_BODY_MAX) {
            cache_len = ESB_HID_CACHE_BODY_MAX;
        }
        memcpy(peer->hid_cache[type].body, body, cache_len);
        peer->hid_cache[type].len = (uint8_t)cache_len;
        peer->hid_cache[type].valid = true;

        if (forward) {
            int err = rx_send_hid(type, body, body_len);

            if (err) {
                LOG_WRN("USB HID forward failed (type %d): %d", type, err);
            }
        }
        break;
    }
    default:
        /* PAIR_REQUEST / PAIR_ASSIGN on a slot pipe: protocol error,
         * ignore. */
        break;
    }

    identity_maybe_load(slot);

    k_mutex_unlock(&esb_hid_rx_mutex);
}

static void rx_thread(void) {
    struct esb_payload payload;

    while (1) {
        k_sem_take(&rx_sem, K_FOREVER);
        while (esb_read_rx_payload(&payload) == 0) {
            handle_payload(&payload);
        }
    }
}

K_THREAD_DEFINE(esb_hid_rx_thread, 1024, rx_thread, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

/* ------------------------------------------------------------------ */
/* Active-slot failover                                                */

/* When the active keyboard has been silent past the link-lost
 * timeout (ESB_HID_LINK_LOST_TIMEOUT_MS — the same 15 s the display
 * greys it out at, three missed 5 s status frames), hand active to
 * the freshest other registered keyboard that is still on the air,
 * so typing continues on the surviving keyboard instead of dying
 * with the one that was switched off or walked away. There is no
 * automatic fail-back: when the original keyboard returns it does
 * not steal active back (the user can switch manually), which also
 * keeps two keyboards at the edge of range from flapping active
 * between them. The switch itself is esb_hid_rx_set_active_slot(),
 * with its usual release / Studio flush / re-inject contract. */
#define ESB_HID_ACTIVE_WATCHDOG_MS 1000

static void active_watchdog(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(active_watchdog_work, active_watchdog);

static void active_watchdog(struct k_work *work) {
    k_work_reschedule(&active_watchdog_work, K_MSEC(ESB_HID_ACTIVE_WATCHDOG_MS));

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_AUTO_ACTIVE)
    uint8_t target = ESB_HID_SLOT_COUNT;

    k_mutex_lock(&esb_hid_rx_mutex, K_FOREVER);
    uint8_t active = registry.active_slot;
    bool pairing_busy = (pairing.state != ESB_HID_PAIRING_IDLE);

#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING)
    pairing_busy = pairing_busy || boot_pairing_armed;
#endif

    if (active < ESB_HID_SLOT_COUNT && registry.slots[active].used && !pairing_busy &&
        peers[active].last_seen_ms != 0) {
        int64_t now = k_uptime_get();

        if (now - peers[active].last_seen_ms > ESB_HID_LINK_LOST_TIMEOUT_MS) {
            int64_t freshest = 0;

            for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
                if (i == active || !registry.slots[i].used || peers[i].last_seen_ms == 0) {
                    continue;
                }
                if (now - peers[i].last_seen_ms <= ESB_HID_LINK_LOST_TIMEOUT_MS &&
                    peers[i].last_seen_ms > freshest) {
                    freshest = peers[i].last_seen_ms;
                    target = (uint8_t)i;
                }
            }
        }
    }
    k_mutex_unlock(&esb_hid_rx_mutex);

    if (target < ESB_HID_SLOT_COUNT) {
        LOG_INF("Active slot %d silent past %d ms; failing over to slot %d", active,
                ESB_HID_LINK_LOST_TIMEOUT_MS, target);
        esb_hid_rx_set_active_slot(target);
    }
#endif /* IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_AUTO_ACTIVE) */
}

static void rx_link_retry(struct k_work *work);

/* Same retry contract as the TX side: while link start fails, retry
 * every 5 s so the failing stage keeps logging its error. */
static K_WORK_DELAYABLE_DEFINE(rx_link_retry_work, rx_link_retry);

static void rx_link_retry(struct k_work *work) {
    LOG_INF("esb_hid: RX link start attempt (ESBHID-3)");
    registry_load_once();
    int err = esb_hid_link_start(ESB_MODE_PRX, esb_hid_rx_event_handler);

#if ESB_HID_LINK_LED
    rx_init_err = err;
#endif
    if (err) {
        LOG_ERR("ESB RX link start failed: %d (retry in 5s)", err);
        k_work_reschedule(&rx_link_retry_work, K_SECONDS(5));
    }
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_BOOT_PAIRING)
    if (!err) {
        /* Defer the boot window decision a little: the keyboard this
         * receiver was last used with gets a chance to show up first,
         * and if it does, no window opens at all. */
        k_work_reschedule(&boot_pairing_work, K_MSEC(ESB_HID_BOOT_PAIRING_DELAY_MS));
    }
#endif
}

static int esb_hid_rx_init(void) {
#if ESB_HID_LINK_LED
    if (gpio_is_ready_dt(&link_led)) {
        gpio_pin_configure_dt(&link_led, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&link_led_red, GPIO_OUTPUT_INACTIVE);
        k_work_schedule(&link_blink_work, K_MSEC(ESB_HID_LINK_BLINK_MS));
    }
#endif
    for (int i = 0; i < ESB_HID_SLOT_COUNT; i++) {
        peers[i].boot_armed = true;
    }
    /* First attempt deferred 6 s, same reason as TX: don't lose the
     * original error to the not-yet-attached USB console. */
    LOG_INF("esb_hid: RX init, first link start in 6s (ESBHID-3)");
    k_work_schedule(&rx_link_retry_work, K_SECONDS(6));
#if IS_ENABLED(CONFIG_ZMK_ESB_HID_RX_AUTO_ACTIVE)
    k_work_schedule(&active_watchdog_work, K_SECONDS(7));
#endif
    return 0;
}

/* After the USB HID init (CONFIG_ZMK_USB_HID_INIT_PRIORITY=95) so early
 * frames are not forwarded to a not-yet-registered HID device. */
SYS_INIT(esb_hid_rx_init, APPLICATION, 96);
