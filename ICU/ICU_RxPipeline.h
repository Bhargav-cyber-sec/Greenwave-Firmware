#ifndef ICU_RX_PIPELINE_H
#define ICU_RX_PIPELINE_H

/***********************************************************************
 * ICU_RxPipeline.h  --  v7.1 receive/validation concurrency substrate
 *
 * ---------------------------------------------------------------------
 * NEW IN v7.1. There was no such file in v7.
 *
 * It is a new tab-adjacent header in the ICU sketch folder, so it costs
 * nothing on the RDU side and does not participate in the six-folder
 * identity rule (that rule covers the RDU support files; see README §3).
 *
 * verify_rdu_tree.py's check_rx_pipeline_wired() asserts that ICU.ino
 * actually INCLUDES this and actually CALLS its entry points. That check
 * is not defending against a past incident with this file -- it is
 * applying README §8's standing lesson to a new piece of machinery
 * before it has a chance to go wrong:
 *
 *   "Constant defined, documented, cross-checked, NEVER READ. Every
 *    other check passed because they all verified the value and none
 *    verified that anything used it."
 *
 * A substrate that is present, correct and unwired would fail in exactly
 * that shape, and would be worse than absent, because it would make the
 * tree look solved.
 *
 * ---------------------------------------------------------------------
 * WHAT THE ACTUAL BOTTLENECK IS
 *
 * Until v7 the ICU did this, in one thread, inside loop():
 *
 *     parsePacket -> drain FIFO into ONE global rxFrame
 *                 -> header checks
 *                 -> replay check
 *                 -> HMAC verify (mbedTLS)
 *                 -> mutate NodeState security fields
 *                 -> Serial.printf the [RX2] line
 *                 -> processHeartbeat/LoRaEvent/AcousticEvent
 *                    (each of which Serial.printf's again)
 *                 -> geometry, tracks, EVU tracks
 *
 * and then returned to loop(), which went on to do ercService(),
 * gwUpdateNodeHealth(), gwUpdateTracks(), gwUpdateEvuTracks(),
 * gwUpdateDecision(), ercBenchConsole() and printRejectSummary() before
 * it could look at the radio again.
 *
 * The radio was unattended for all of that. Measured cost, from the
 * actual v7 source (see README §7.4 for the arithmetic):
 *
 *   [RX2] + [LORA RX] for one accepted event frame ... ~31 ms of
 *       BLOCKING UART, inside the receive path, radio unattended
 *   printRejectSummary() every 30 s .................. >=240 ms
 *
 * The SX127x in continuous RX does not queue. A frame that completes
 * while its FIFO still holds an unread one is DESTROYED, not delayed.
 * At 86-129 ms of airtime per frame, a 240 ms printout costs about two
 * frames outright, and the node's next re-assert is ACOUSTIC_REASSERT_MS
 * (8 s) away. That is the "adding RDUs adds latency" symptom -- but the
 * unit is SECONDS OF LOST RE-ASSERT, not microseconds of queued HMAC.
 *
 * IT IS NOT THE HMAC. HMAC-SHA256 over a <=56-byte frame is four SHA256
 * compressions; on an ESP32-S3 with the SHA accelerator that is tens of
 * microseconds against 129 ms of airtime for the frame it authenticates
 * -- about 0.03%. Six workers all doing HMAC in parallel would save
 * nothing measurable, because six RDUs SHARE ONE 434.5 MHz CHANNEL and
 * cannot deliver two frames to this receiver at the same instant
 * anyway. Two overlapping transmissions collide and BOTH are lost.
 *
 * So the fix is not "parallelise the cryptography". It is "stop making
 * radio attendance wait for the decision-and-logging tail", plus enough
 * concurrency that the one genuinely expensive operation in the
 * validation path -- X25519 ECDH inside keyFor() on an epoch change,
 * milliseconds, not microseconds -- cannot head-of-line block a
 * DIFFERENT node's frame. See "WHY TWO WORKERS" in ICU.ino.
 *
 * ---------------------------------------------------------------------
 * WHAT THIS FILE PROVIDES
 *
 *   1. GwRxRing   -- one lock-free SPSC ring PER NODE. Single producer
 *                    is the RX task (there is one radio, so there is one
 *                    reader of it, by construction rather than by
 *                    convention). Single consumer is whichever
 *                    validation worker currently HOLDS that node.
 *
 *   2. node claims -- gwClaimNode()/gwReleaseNode(). A worker takes
 *                    exclusive ownership of ONE node's security state
 *                    (lastCounter, counterStarted, that node's session
 *                    Slot, its reject counters) for as long as it is
 *                    draining that node's ring. Different nodes are
 *                    claimed independently and progress concurrently.
 *                    This is serialisation BY IDENTITY, not by resource:
 *                    it is what makes "same RDU cannot race itself" and
 *                    "different RDUs do not wait for each other" both
 *                    true with no global mutex anywhere.
 *
 *   3. GwValRing  -- the validated-event handoff to the decision layer.
 *                    Many producers (workers), one consumer (loop()).
 *                    Guarded by a spinlock held ONLY for the duration of
 *                    a bounded memcpy. That is handoff serialisation, in
 *                    the same category as "one radio has one owner". No
 *                    cryptography ever runs inside it.
 *
 *   4. GwLogRing  -- deferred Serial, for the reasons in GW_SESS_LOG's
 *                    comment in GreenwaveCrypto.h. Neither the RX task
 *                    nor a worker may touch the UART.
 *
 * ---------------------------------------------------------------------
 * OWNERSHIP, STATED ONCE, HERE
 *
 *   LoRa radio + SPI ........ gwRxTask, exclusively. Nothing else in the
 *                             build may touch LoRa.* after setup().
 *   Per-node security state . the worker currently holding that node's
 *                             claim. Exactly one at a time, by CAS.
 *                             Enumerated field by field in NodeState.
 *   Decision/track/signal/
 *   ERC/geometry state ...... loop(), exclusively. Unchanged from v7.
 *   Serial .................. loop(), exclusively.
 *
 * ---------------------------------------------------------------------
 * HOST BUILD
 *
 * Everything here compiles on a host with -DGW_HOST_TEST so
 * tests/test_icu_rx_pipeline.cpp can exercise the ring, the claim
 * protocol and the duplicate-suppression ordering with real threads.
 ***********************************************************************/

#include <atomic>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>

#ifndef GW_MAX_FRAME_LEN
#error "include GreenwaveTypes.h before ICU_RxPipeline.h"
#endif

// ---------------------------------------------------------------------
// TUNABLES
// ---------------------------------------------------------------------

// Slots per node. Depth 4 across 6 nodes is 24 in-flight frames.
//
// Sizing argument, from the air rather than from taste: the six RDUs
// share ONE SF7/125k/CR4-6 channel, so the fastest the ICU can be handed
// frames is one per 86 ms (the shortest frame, AcousticEventFrame at 30
// bytes). Filling one node's 4 slots therefore needs >=344 ms during
// which that node's worker made no progress at all. A worker drains a
// node in tens of microseconds in the common case and a few milliseconds
// in the ECDH case, so 4 is deep enough to absorb any burst the air can
// physically produce, and shallow enough that a genuinely stuck worker
// shows up as counted drops rather than as unbounded memory.
//
// MUST be a power of two: the ring uses a mask, not a modulo.
#define GW_RX_RING_DEPTH        4

// Validated events waiting for loop(). One decision pass drains all of
// them, so this only has to cover one loop() period's worth of arrivals.
#define GW_VAL_RING_DEPTH       8

// Deferred log lines. Overflow is counted and reported, never silent.
#define GW_LOG_RING_DEPTH      16
#define GW_LOG_LINE_LEN       192

#define GW_RX_NODES             6

static_assert((GW_RX_RING_DEPTH  & (GW_RX_RING_DEPTH  - 1)) == 0,
              "GW_RX_RING_DEPTH must be a power of two");
static_assert((GW_VAL_RING_DEPTH & (GW_VAL_RING_DEPTH - 1)) == 0,
              "GW_VAL_RING_DEPTH must be a power of two");
static_assert((GW_LOG_RING_DEPTH & (GW_LOG_RING_DEPTH - 1)) == 0,
              "GW_LOG_RING_DEPTH must be a power of two");

// ---------------------------------------------------------------------
// SPINLOCK  --  ESP32 portMUX on target, std::mutex on host.
//
// Used for the two MANY-PRODUCER rings only (validated events, log).
// Never wrapped around key derivation, HMAC, or any per-node security
// transition. The critical section is a bounded memcpy of one fixed-size
// slot and nothing else.
// ---------------------------------------------------------------------
#ifdef GW_HOST_TEST
  #include <mutex>
  struct GwSpin {
      std::mutex m;
      void lock()   { m.lock();   }
      void unlock() { m.unlock(); }
  };
#else
  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  struct GwSpin {
      portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
      void lock()   { portENTER_CRITICAL(&mux); }
      void unlock() { portEXIT_CRITICAL(&mux);  }
  };
#endif

// ---------------------------------------------------------------------
// RX SLOT
//
// Brief §12: every queued request owns an IMMUTABLE COPY of the packet
// for the whole of its processing. There is no pointer into a shared
// rxFrame anywhere in this design; the old global is gone. The RX task
// copies the radio FIFO straight into a slot it has exclusively
// reserved, and nothing writes that slot again until the consumer has
// popped it.
//
// Because of that last property, a worker validates IN PLACE out of the
// slot and pops only when it is finished. The producer cannot reach
// slot[tail] while tail has not advanced -- reserve() refuses at
// (head - tail) == DEPTH -- so this is safe and saves a second 128-byte
// copy per frame. Do not "optimise" the pop to before the validation.
//
// Fixed size, no heap: 6 nodes x 4 slots x sizeof(GwRxSlot).
// ---------------------------------------------------------------------
struct GwRxSlot {
    uint8_t  bytes[GW_MAX_FRAME_LEN];
    uint16_t len;
    int16_t  rssi;        // IF-2 packet RSSI, dBm
    int16_t  snr_x10;     // IF-2 packet SNR, dB x10 (float kept out of the ring)
    uint16_t _pad;
    uint32_t t_rx_us;     // micros() at end of FIFO drain
    uint32_t t_q_us;      // micros() at commit -- queue insertion cost
};

// ---------------------------------------------------------------------
// PER-NODE SPSC RING  --  lock free, no CAS, no critical section
//
// Producer: gwRxTask, and only gwRxTask.
// Consumer: whichever worker holds this node's claim. The claim protocol
// guarantees at most one at any instant, which is what makes the SPSC
// assumption sound with more than one worker configured.
//
// Free-running 32-bit counters, index = counter & mask. Wrap is
// well-defined on unsigned arithmetic and (head - tail) stays correct
// across it.
// ---------------------------------------------------------------------
struct GwRxRing {
    GwRxSlot slot[GW_RX_RING_DEPTH];
    std::atomic<uint32_t> head{0};    // written by producer only
    std::atomic<uint32_t> tail{0};    // written by consumer only
    std::atomic<uint32_t> dropped{0}; // producer only; diagnostic
    std::atomic<uint32_t> maxDepth{0};// producer only; diagnostic

    // ---- producer side ----
    // Returns a slot to fill, or nullptr if the ring is full. Does not
    // publish; the producer must call commit() after writing.
    GwRxSlot *reserve() {
        uint32_t h = head.load(std::memory_order_relaxed);
        uint32_t t = tail.load(std::memory_order_acquire);
        uint32_t used = h - t;
        if (used >= GW_RX_RING_DEPTH) {
            dropped.fetch_add(1, std::memory_order_relaxed);
            return nullptr;                       // §17: bounded, explicit
        }
        if (used + 1 > maxDepth.load(std::memory_order_relaxed))
            maxDepth.store(used + 1, std::memory_order_relaxed);
        return &slot[h & (GW_RX_RING_DEPTH - 1)];
    }

    // Release: everything written into the slot above happens-before the
    // consumer's acquire-load of head in peek().
    void commit() { head.fetch_add(1, std::memory_order_release); }

    // ---- consumer side ----
    const GwRxSlot *peek() const {
        uint32_t t = tail.load(std::memory_order_relaxed);
        uint32_t h = head.load(std::memory_order_acquire);
        if (h == t) return nullptr;
        return &slot[t & (GW_RX_RING_DEPTH - 1)];
    }

    void pop() { tail.fetch_add(1, std::memory_order_release); }

    uint32_t depth() const {
        return head.load(std::memory_order_acquire) -
               tail.load(std::memory_order_relaxed);
    }
};

// ---------------------------------------------------------------------
// VALIDATED EVENT  --  worker -> loop()
//
// Carries the AUTHENTICATED BYTES, not a decoded struct, so the decision
// layer keeps doing its own memcpy into HeartbeatFrame/LoRaEventFrame/
// AcousticEventFrame exactly as v7 did. The wire format is untouched and
// the handlers are called with identical arguments to before.
//
// t_* are micros() stamps for the [PERF] line. They are diagnostics; no
// decision reads them.
// ---------------------------------------------------------------------
struct GwValEvent {
    uint8_t  bytes[GW_MAX_FRAME_LEN];
    uint16_t len;
    uint8_t  nodeIdx;      // 0..5, gwKeyIndex order
    uint8_t  packetType;
    int16_t  rssi;
    int16_t  snr_x10;
    uint32_t counter;
    uint32_t sessionEpoch;
    uint32_t t_rx_us;      // radio FIFO drained
    uint32_t t_q_us;       // committed to the node ring
    uint32_t t_val0_us;    // worker picked it up
    uint32_t t_key_us;     // session key resolved (post keyFor)
    uint32_t t_val1_us;    // HMAC verified, security state updated
};

// Many producers (workers), one consumer (loop). Spinlock, memcpy only.
struct GwValRing {
    GwValEvent slot[GW_VAL_RING_DEPTH];
    uint32_t   head = 0;
    uint32_t   tail = 0;
    std::atomic<uint32_t> dropped{0};
    GwSpin     lk;

    // Returns false when full. Never blocks, never allocates.
    //
    // NOTE what a false return means for security: the frame HAS already
    // been authenticated and this node's replay counter HAS already been
    // advanced. Dropping here loses a detection, it does not admit an
    // unauthenticated one, and it can never cause the same counter to be
    // accepted twice. It is counted and reported on the [PIPE] line.
    bool push(const GwValEvent &e) {
        bool ok;
        lk.lock();
        if ((uint32_t)(head - tail) >= GW_VAL_RING_DEPTH) {
            ok = false;
        } else {
            slot[head & (GW_VAL_RING_DEPTH - 1)] = e;
            head++;
            ok = true;
        }
        lk.unlock();
        if (!ok) dropped.fetch_add(1, std::memory_order_relaxed);
        return ok;
    }

    bool pop(GwValEvent &out) {
        bool ok;
        lk.lock();
        if (head == tail) {
            ok = false;
        } else {
            out = slot[tail & (GW_VAL_RING_DEPTH - 1)];
            tail++;
            ok = true;
        }
        lk.unlock();
        return ok;
    }
};

// ---------------------------------------------------------------------
// DEFERRED LOG RING
//
// vsnprintf runs in the CALLER (worker or RX task); only the memcpy of
// the finished line happens under the lock. Formatting is the expensive
// part and it stays outside the critical section.
// ---------------------------------------------------------------------
struct GwLogRing {
    char     line[GW_LOG_RING_DEPTH][GW_LOG_LINE_LEN];
    uint32_t head = 0;
    uint32_t tail = 0;
    std::atomic<uint32_t> dropped{0};
    GwSpin   lk;

    void pushv(const char *fmt, va_list ap) {
        char tmp[GW_LOG_LINE_LEN];
        vsnprintf(tmp, sizeof(tmp), fmt, ap);
        bool ok;
        lk.lock();
        if ((uint32_t)(head - tail) >= GW_LOG_RING_DEPTH) {
            ok = false;
        } else {
            memcpy(line[head & (GW_LOG_RING_DEPTH - 1)], tmp, sizeof(tmp));
            head++;
            ok = true;
        }
        lk.unlock();
        if (!ok) dropped.fetch_add(1, std::memory_order_relaxed);
    }

    bool pop(char *out, size_t n) {
        bool ok;
        lk.lock();
        if (head == tail) {
            ok = false;
        } else {
            strncpy(out, line[tail & (GW_LOG_RING_DEPTH - 1)], n - 1);
            out[n - 1] = '\0';
            tail++;
            ok = true;
        }
        lk.unlock();
        return ok;
    }
};

// ---------------------------------------------------------------------
// NODE OWNERSHIP CLAIM
//
// THE CORE OF THE WHOLE DESIGN. Read this before changing anything.
//
// gwNodeOwner[k] == 0        -> node k's security state is unowned
// gwNodeOwner[k] == w + 1    -> worker w owns it, exclusively
//
// A worker CASes 0 -> w+1 to take node k, drains k's ring to completion,
// then stores 0. Nothing else in the system reads or writes k's
// lastCounter, counterStarted, framesAccepted, framesLostIf2,
// rejectedMac, rejectedReplay, or its GwIcuSessionCache::Slot.
//
// Consequences, which are exactly the guarantees the brief asks for:
//
//  * Two copies of the same counter from node k cannot both be accepted
//    (brief §3, §11, §20 case 3). They land in k's ring in arrival order;
//    ONE owner pops them in that order; the first sets lastCounter, the
//    second compares equal and is suppressed. The check-then-update
//    sequence is never interleaved, because there is never a second
//    reader of that state to interleave WITH. No lock is held across the
//    HMAC to achieve this. The race in brief §11 is not prevented, it is
//    made unrepresentable.
//
//  * Node j proceeds while node k is being validated (§9, §20 cases 4-5):
//    different index, different CAS, no contention. Nothing a node-k
//    worker does -- including a multi-millisecond X25519 inside keyFor()
//    -- can delay node j.
//
//  * If a second worker finds k already claimed, it does NOT wait. It
//    moves on to the next node. Head-of-line blocking is impossible, and
//    there is no lock to hold, so there is nothing to deadlock.
//
// acquire on success / release on store pairs the security state itself:
// everything the previous owner wrote is visible to the next owner.
// ---------------------------------------------------------------------
extern std::atomic<uint8_t> gwNodeOwner[GW_RX_NODES];

static inline bool gwClaimNode(int k, uint8_t workerId) {
    uint8_t expected = 0;
    return gwNodeOwner[k].compare_exchange_strong(
               expected, (uint8_t)(workerId + 1),
               std::memory_order_acquire, std::memory_order_relaxed);
}

static inline void gwReleaseNode(int k) {
    gwNodeOwner[k].store(0, std::memory_order_release);
}

// ---------------------------------------------------------------------
// GLOBALS  (defined in ICU.ino)
// ---------------------------------------------------------------------
extern GwRxRing  gwRxRing[GW_RX_NODES];
extern GwValRing gwValRing;
extern GwLogRing gwLogRing;

// Deferred printf. Safe from any task. This is what GW_SESS_LOG is
// pointed at, so GreenwaveCrypto.h's [SESS] lines no longer touch the
// UART from inside a worker. Adds no newline: the drainer uses println.
void gwPipeLogf(const char *fmt, ...);

// Relaxed atomic increment for the global reject counters. They are
// written by the RX task and by workers, read by loop()'s [HEALTH]
// printout. Relaxed is right: they are counters, nothing branches on
// them, and no other state is ordered against them.
#define GW_CNT_INC(x)  __atomic_fetch_add(&(x), 1u, __ATOMIC_RELAXED)

#endif // ICU_RX_PIPELINE_H
