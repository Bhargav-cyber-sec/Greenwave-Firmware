#ifndef GREENWAVE_TYPES_H
#define GREENWAVE_TYPES_H

/***********************************************************************
 * GREENWAVE TYPES  --  WIRE PROTOCOL v4
 *
 * CANONICAL DEFINITION. Copy this file byte-identical into BOTH the
 * Lane Node sketch folder and the ICU sketch folder.
 *
 * v3 CHANGES vs v2 (read this before flashing anything):
 *
 *  1. Every IF-2 frame now starts with proto_version. The ICU dispatches
 *     on packet_type and CHECKS the length, instead of guessing the type
 *     FROM the length. In v2, if these two files ever drifted by one
 *     field, every packet silently became "[UNKNOWN PACKET]" with no
 *     indication of why. That failure mode is now impossible: a version
 *     mismatch prints a specific error.
 *
 *  2. Frames shrank. EventPacket was 136 bytes = 861 ms of airtime at
 *     SF9/CR4-6. LoRaEventFrame is 52 bytes = 394 ms. The lane node has
 *     ONE radio shared between 433.0 (listening to the ambulance) and
 *     434.5 (talking to the ICU), so every millisecond spent
 *     transmitting is a millisecond deaf to the ambulance. See
 *     sendToICU() in LaneNode.ino.
 *
 *  3. Every frame carries a monotonic counter and an 8-byte tag. Without
 *     these, anyone with a cheap LoRa module could transmit a struct on
 *     434.5 MHz and trigger preemption without ever touching the
 *     ambulance link or its Ed25519 signatures.
 *
 * v4 CHANGES vs v3:
 *
 *  5. FrameHeader gained uint32 session_epoch (10 -> 14 bytes), so every
 *     frame grew by 4: LoRaEvent 52->56, Acoustic 20->24, Heartbeat
 *     28->32. Airtime at SF9/CR4-6 rose ~25 ms per frame; EVENT_AIRTIME_MS
 *     and HEARTBEAT_AIRTIME_MS in LaneNode.ino were updated to match and
 *     MUST be kept in step with any future size change.
 *
 *  6. The tag is now truncated HMAC-SHA256 under a per-session key derived
 *     by X25519 ECDH + HKDF-SHA256, NOT the v3 static AES-CMAC. There is
 *     no longer any table of other devices' secrets in any image.
 *
 *  4. char[] fields that were mostly padding became enums:
 *       motion_state[16]     -> uint8_t  (MotionState)
 *       signature_status[16] -> uint8_t  (SigStatus)
 *       intersection_id[8]   -> uint16_t
 *       vehicle_id[32]       -> char[8]   (the EVU only ever sends 6)
 *     Dropped entirely: altitude_m (nothing read it), tx_uptime (always
 *     0), uptime_ms (nothing read it), distance_m / eta_s (the ICU knows
 *     its own surveyed geometry and the vehicle position -- it should
 *     compute these itself rather than trust six nodes to each compute
 *     them separately).
 *
 * v5 CHANGES vs v4  --  SPEC v2.0 ALIGNMENT
 *
 *  7. HeartbeatFrame now carries the node's SURVEYED GEOMETRY:
 *       approach_id, distance_to_stopline_m, config_hash
 *     This is the single most important change in v5.
 *
 *     Under v4 the ICU had no way to know which of a lane's two nodes was
 *     physically closer to the stop line. It used node_id, and node_id is
 *     an install-order label. Spec v2.0 Scenario 1 defect S1-01 is exactly
 *     this: the v1.0 logic document had the geometry INVERTED, so every
 *     approaching vehicle would have been classified as departing and
 *     every departing vehicle would have triggered a preemption. The
 *     failure is silent and symmetric -- nothing in the logs looks wrong.
 *
 *     Distance is DYNAMIC per junction. It is never a constant in code.
 *     Each RDU is provisioned with its own surveyed distance, transmits it
 *     in every heartbeat, and the ICU sorts a lane's nodes by that value
 *     at runtime. Largest distance = RDU_FAR. Swap two enclosures, or
 *     re-flash one with the wrong config, and the ICU sees it rather than
 *     silently inverting its direction logic.
 *
 *  8. HeartbeatFrame gained mic_noise_floor and loop_liveness.
 *     Spec defect S1-04: v1.0 required the ICU to distinguish "healthy but
 *     hearing nothing" from "failed", and gave it no mechanism. A dead or
 *     obstructed microphone reports a noise floor at or near zero, which
 *     is a SELF-DETECTED fault. Without this field such a node presents
 *     forever as a healthy node that happens never to hear anything.
 *
 *  9. AcousticEventFrame gained event_id and age_ms.
 *     Direction of travel is inferred from the time delta between the FAR
 *     observation and the NEAR observation. Measuring that delta from
 *     frame ARRIVAL time at the ICU folds in LoRa queueing and airtime
 *     jitter, which on a shared 434.5 channel can exceed the delta being
 *     measured. age_ms lets the ICU recover the true detection instant.
 *     event_id lets two nodes' reports be tied to one physical siren.
 *
 * 10. LoRaEventFrame gained heading_deg.
 *     It was absent in v4 and nothing could work without it. Spec S2-14:
 *     approach association from position alone puts two EVUs travelling in
 *     OPPOSITE directions on the same road into the same approach, which
 *     needs opposite phases. Divergence detection (S2-09) also requires
 *     heading. The EVU already sends heading in IF-1; v4 simply discarded
 *     it at the RDU instead of relaying it.
 *
 * 11. GW_NUM_APPROACHES / ApproachConfig. Three-way today, four-way by
 *     changing one number. Nothing below this line assumes 3.
 *
 *  ---- AIRTIME IMPACT OF v5, READ BEFORE FLASHING ----
 *     LoRaEventFrame     56 -> 58 bytes
 *     AcousticEventFrame 24 -> 30 bytes
 *     HeartbeatFrame     32 -> 42 bytes
 *     EVENT_AIRTIME_MS and HEARTBEAT_AIRTIME_MS in RDU.ino are sized
 *     against the OLD lengths and MUST be recomputed. The heartbeat grew
 *     by 31%, and the RDU shares one radio between listening to the
 *     ambulance on 433.0 and talking to the ICU on 434.5 -- every extra
 *     millisecond transmitting is a millisecond deaf to the ambulance.
 *
 * ANY change to a struct below requires: (a) bump GW_PROTO_VERSION,
 * (b) update the static_assert, (c) copy this file to the other sketch,
 * (d) reflash every node AND the ICU. There is no partial upgrade.
 ***********************************************************************/

#include <Arduino.h>
#include <stddef.h>

#define GW_PROTO_VERSION 5

/***********************************************************************
 * INTERSECTION SHAPE  (v5)
 *
 * Change GW_NUM_APPROACHES to 4 and add the fourth row to the table in
 * ICU.ino. Nothing in this header, in RDU.ino, or in the ICU decision
 * logic is written against the number 3.
 *
 * NODES_PER_APPROACH is 2 and is NOT a free parameter -- the whole
 * direction-inference method is "which of two points did the siren reach
 * first". A third node would be a different algorithm, not a bigger loop.
 ***********************************************************************/
#define GW_NUM_APPROACHES     3
#define GW_NODES_PER_APPROACH 2
#define GW_MAX_NODES          (GW_NUM_APPROACHES * GW_NODES_PER_APPROACH)

/*
 * Geometric role, derived by the ICU at runtime from the surveyed
 * distances in the heartbeats. NEVER stored on a node, never sent on the
 * wire, never derived from node_id.
 *
 * Spec Scenario 1 v2.0 section 3: "The ordinal names RDU 1 and RDU 2 are
 * withdrawn from the logic layer. They encoded position implicitly, and
 * in v1.0 the implication was inverted."
 */
enum NodeRole : uint8_t {
    ROLE_UNKNOWN = 0,   // not enough heartbeats yet, or distances invalid
    ROLE_FAR     = 1,   // outer node  -- first one an approaching vehicle passes
    ROLE_NEAR    = 2    // inner node  -- last one before the stop line
};

/*
 * Sentinel for "this node has not been given a surveyed distance".
 * Distinct from 0, because 0 metres is a plausible-looking value that
 * would sort as the NEAR node and silently win the ordering.
 */
#define GW_DISTANCE_UNSET  0xFFFF

/*
 * Plausibility bounds for a surveyed distance. A node reporting outside
 * this range is treated as MISCONFIGURED rather than believed.
 * These bound a PROVISIONING MISTAKE, not the road -- they are wide on
 * purpose. Real values for this deployment are 150 m and 250 m.
 */
#define GW_DISTANCE_MIN_M  5
#define GW_DISTANCE_MAX_M  2000

/***********************************************************************
 * IF-1 : EVU -> LANE NODE PAYLOAD
 * Duplicated verbatim in EVU2.ino (which does not include this header).
 * Any change here MUST be mirrored there byte-for-byte.
 ***********************************************************************/
struct __attribute__((packed)) TelemetryPayload {
  char vehicle_id[6];      // Fixed length matching "AMB_02", not NUL-terminated
  uint8_t flags;           // b0 emergency, b1 gps_valid, b2 siren_active, b3 altitude_valid
  uint8_t priority_class;
  uint32_t seq;
  uint32_t gps_epoch;      // Always UTC
  uint16_t gps_ms;
  int32_t latitude;        // Scaled by 10,000,000
  int32_t longitude;       // Scaled by 10,000,000
  int16_t altitude_m;      // Metres above MSL
  uint16_t speed_kmph;     // Scaled by 100
  uint16_t heading_deg;    // Scaled by 100
  uint8_t test_leg;        // Range-test tag. MUST be removed before pilot.
};

/***********************************************************************
 * ENUMS  (replace the old fixed-length strings)
 ***********************************************************************/
enum PacketType : uint8_t {
    HEARTBEAT_PACKET      = 0,
    LORA_EVENT_PACKET     = 1,
    ACOUSTIC_EVENT_PACKET = 2,
    SESSION_ESTABLISH     = 3    // v4, SOP 5.1 -- only when GW_CERT_ON_AIR
};

enum SigStatus : uint8_t {
    SIG_UNVERIFIED    = 0,
    SIG_VERIFIED      = 1,
    SIG_CERT_REJECTED = 2,
    SIG_WAITING_CERT  = 3,
    SIG_FAILED        = 4
};

enum MotionState : uint8_t {
    MOTION_UNKNOWN     = 0,
    MOTION_APPROACHING = 1,
    MOTION_RECEDING    = 2,
    MOTION_STATIONARY  = 3
};

// Event frame flag bits
#define EVF_LORA_DETECTED   (1 << 0)
#define EVF_EMERGENCY       (1 << 1)
#define EVF_GPS_VALID       (1 << 2)
#define EVF_GEOFENCE_PASS   (1 << 3)
#define EVF_GEOFENCE_BYPASS (1 << 4)   // node is running with the SOP bypass on

// Acoustic frame flag bits
#define ACF_MIC_OK          (1 << 0)
#define ACF_SUSTAINED       (1 << 1)   // score reached CONFIRM_SCORE, not a single spike

// Heartbeat flag bits
#define HBF_LORA_OK         (1 << 0)
#define HBF_MIC_OK          (1 << 1)
#define HBF_BENCH_BUILD     (1 << 2)   // node built with test/bypass options enabled
#define HBF_BATT_VALID      (1 << 3)   // battery ADC actually read something

// v5 additions.
#define HBF_DISTANCE_VALID  (1 << 4)   // node was provisioned with a surveyed
                                       // distance. Clear = never provisioned.
                                       // An explicit flag, not just the
                                       // sentinel value, so a corrupted field
                                       // that happens to land in the plausible
                                       // range still fails the check.

#define HBF_MIC_SELFTEST_OK (1 << 5)   // noise floor inside plausible bounds
                                       // AND the inference loop is advancing.
                                       // Distinct from HBF_MIC_OK, which only
                                       // says the driver initialised: a mic
                                       // can initialise fine and be physically
                                       // taped over.

#define HBF_TIME_VALID      (1 << 6)   // node has a disciplined clock. Its
                                       // age_ms values are only meaningful
                                       // when this is set.

/***********************************************************************
 * NODE HEALTH STATES  (spec Scenario 1 v2.0 section 5.2)
 *
 * ICU-side only -- never transmitted. Listed here so RDU and ICU share
 * one vocabulary in logs and on the ERC screen.
 *
 * The distinction that earns this enum its existence is SUSPECT vs
 * FAILED. Under v4 there were two states, online and offline, and one
 * missed heartbeat flipped a node to offline. Offline unlocks
 * degraded mode, and degraded mode LOWERS the evidence needed to act.
 *
 * So under v4 a single dropped packet quietly relaxed the system's own
 * safety bar -- and IF-2 heartbeat delivery was measured at 96.3%, so
 * this was routine, not hypothetical. SUSPECT exists to absorb that:
 * the node keeps participating, and its partner is NOT promoted.
 *
 * This also closes a cheap attack (spec S1-06). Jamming one node is far
 * easier than defeating authentication, and under v4 jamming was
 * REWARDED with a lower evidence threshold at the surviving node.
 ***********************************************************************/
enum NodeHealth : uint8_t {
    NH_UNKNOWN      = 0,   // no heartbeat since ICU boot
    NH_HEALTHY      = 1,   // current and self-tests passing. Full
                           // participation, and this node's SILENCE is
                           // meaningful evidence that nothing is there.
    NH_SUSPECT      = 2,   // a few heartbeats missed, or a marginal
                           // self-test. Still participates. Does NOT
                           // unlock degraded authority for its partner.
    NH_FAILED       = 3,   // sustained silence, hard self-test fault, or
                           // config mismatch. Observations discarded.
                           // Partner may act under the RESTRICTED
                           // degraded policy -- which is asymmetric:
                           // FAR-alone and NEAR-alone are opposite risks
                           // and get opposite policies (spec S1-05).
    NH_RECOVERING   = 4,   // heartbeats returned after FAILED. Logged but
                           // excluded from decisions until it proves
                           // stable, so an intermittent node cannot
                           // oscillate the lane between normal and
                           // degraded authority.
    NH_MISCONFIGURED= 5,   // missing / duplicate / implausible distance,
                           // or config hash mismatch. AUTOMATIC ACTION ON
                           // THIS APPROACH IS DISABLED. The ICU never
                           // infers geometry it was not given.
    NH_LINK_SUSPECT = 6    // heartbeats lost while the ICU's own receiver
                           // noise floor is elevated. A jammed radio and
                           // a dead node look identical, but one is a
                           // maintenance ticket and the other is an
                           // attack in progress. LINK_SUSPECT on one node
                           // while the other reports a siren is a defined
                           // attack signature and suppresses degraded
                           // action entirely.
};

/***********************************************************************
 * COMMON IF-2 FRAME HEADER -- 14 octets
 *
 * counter is (boot_epoch << 16) | seq. It is monotonic ACROSS power
 * cycles, because a counter that restarts at zero on reboot is not
 * replay protection -- an attacker just waits for a power blip. Only the
 * epoch is written to NVS, once per boot, so flash wear is one write per
 * power cycle rather than one per packet.
 ***********************************************************************/
struct __attribute__((packed)) FrameHeader {
    uint8_t  proto_version;   // GW_PROTO_VERSION
    uint8_t  packet_type;     // PacketType
    uint16_t intersection_id; // numeric; MUST be checked by the ICU
    uint8_t  lane_id;         // 1..3
    uint8_t  node_id;         // 1..2
    uint32_t counter;         // (epoch << 16) | seq, never resets

    // v4 (SOP 5.2). Selects which derived session key authenticates this
    // frame. APPENDED AFTER counter on purpose: GW_COUNTER_OFFSET stays
    // 6, so radiateToICU()'s transmit-time counter stamping (the v3.2
    // fix) does not move.
    //
    // uint32, not uint16: at the 60 s re-key cadence a uint16 wraps in 45
    // days, after which the HKDF salt repeats and a session key captured
    // 45 days earlier becomes valid again -- which would defeat the
    // entire point of SOP 3.4 rotation. uint32 gives ~8000 years.
    uint32_t session_epoch;
};

/***********************************************************************
 * LORA EVENT FRAME -- 56 octets
 * "I received an authenticated beacon from an ambulance."
 ***********************************************************************/
struct __attribute__((packed)) LoRaEventFrame {
    FrameHeader hdr;

    uint8_t  flags;         // EVF_*
    uint8_t  sig_status;    // SigStatus -- DIAGNOSTIC ONLY, never authorisation
    uint8_t  priority;      // from the signed region of the IF-1 frame
    uint8_t  motion_state;  // MotionState

    char     vehicle_id[8]; // NUL-padded; EVU sends 6 chars
    int32_t  latitude;      // x10^7, passed through from IF-1 unchanged
    int32_t  longitude;     // x10^7
    uint16_t speed_kmph;    // x100

    // v5: NEW. Relayed unchanged from IF-1, where the EVU has always sent
    // it -- v4 read it at the RDU and threw it away.
    //
    // Two things are impossible without it:
    //   (a) Approach association. Corridor membership alone puts two EVUs
    //       driving in opposite directions on one road into the SAME
    //       approach, and they need opposite phases (spec S2-14).
    //   (b) Divergence / U-turn. A vehicle that turns away otherwise holds
    //       its demand until an undefined timeout (spec S2-09).
    //
    // Degrees x100, 0..35999. 0xFFFF means "no valid heading", which is
    // NOT the same as 0 -- 0 is due north and is a perfectly good heading.
    uint16_t heading_deg;

    uint32_t gps_epoch;     // from IF-1; lets the ICU measure true end-to-end latency
    uint32_t tx_seq;        // the EVU's own seq, for TX<->RX log correlation
    int16_t  rssi_if1;      // dBm on the ambulance link
    int16_t  snr_if1_x10;   // dB x10 on the ambulance link

    uint8_t  tag[8];        // truncated HMAC-SHA256 over every preceding byte
};

/***********************************************************************
 * ACOUSTIC EVENT FRAME -- 24 octets
 * v2 sent a 136-byte struct, zero in all but four fields, to carry one
 * confidence value. That was 861 ms of airtime for 1 byte of meaning.
 ***********************************************************************/
struct __attribute__((packed)) AcousticEventFrame {
    FrameHeader hdr;

    uint8_t confidence;     // 0..255 maps to 0.0..1.0
    uint8_t flags;          // ACF_*

    // v5: NEW. Node-local identifier for one continuous siren episode.
    // Every report about the same unbroken detection carries the same
    // event_id; silence long enough to end the episode starts a new one.
    //
    // Without it, the periodic re-assert stream from the two nodes is just
    // a sequence of undifferentiated "siren now" pulses, and the ICU
    // cannot tell one vehicle's sustained siren from a second vehicle
    // arriving. Spec Scenario 1 section 9 forbids collapsing concurrent
    // sirens into one event; this field is what makes that possible.
    //
    // Node-local, not global: (lane_id, node_id, event_id) is the key.
    uint32_t event_id;

    // v5: NEW. Milliseconds between the FIRST detection of this episode
    // and the moment this frame was built. Lets the ICU reconstruct when
    // detection actually happened.
    //
    // This exists because direction of travel is a TIME DELTA between the
    // two nodes, and measuring it from frame arrival at the ICU measures
    // the radio, not the road. Both nodes share one 434.5 channel with
    // heartbeats and relays; a frame can sit waiting for a clear window
    // for seconds. At 60 km/h across a 100 m separation the true delta is
    // about 6 s, so transport jitter is the same order as the signal --
    // it would not perturb the measurement, it would DOMINATE it.
    //
    // Saturates at 65535 ms rather than wrapping. A wrap here would read
    // as a near-zero age and could fabricate an AMBIGUOUS classification.
    uint16_t age_ms;

    uint8_t tag[8];
};

/***********************************************************************
 * HEARTBEAT FRAME -- 32 octets
 ***********************************************************************/
struct __attribute__((packed)) HeartbeatFrame {
    FrameHeader hdr;

    uint32_t uptime_s;      // seconds, not ms -- ms overflows in 49 days
    uint16_t free_heap_kb;  // KiB
    uint16_t battery_mv;    // millivolts (a float on the wire buys nothing
                            // when the ADC resolution is about 1 mV)
    uint8_t  battery_level; // 1..5
    uint8_t  flags;         // HBF_*

    // ---------------- v5: SURVEYED GEOMETRY ----------------
    //
    // THE MOST IMPORTANT ADDITION IN v5. See the v5 changelog at the top.
    //
    // The node does not know whether it is FAR or NEAR and is never told.
    // It knows only its own surveyed distance. The ICU sorts each lane's
    // nodes by that distance on every heartbeat: largest = RDU_FAR.
    //
    // Carried in the HEARTBEAT rather than sent once at boot, on purpose.
    // A one-time announcement is lost forever if the ICU reboots or the
    // frame is dropped, and it cannot detect an enclosure that was swapped
    // or re-flashed while the ICU was running. Repeating it makes the
    // geometry continuously verified rather than assumed.
    //
    // The heartbeat is authenticated like every other frame, so this
    // cannot be injected by an attacker to invert a lane's direction
    // logic -- which would be an unusually elegant attack, since it makes
    // the system preempt for departing traffic and ignore arrivals.

    uint8_t  approach_id;            // 1..GW_NUM_APPROACHES. Which approach
                                     // this node watches. Cross-checked
                                     // against hdr.lane_id; a mismatch is
                                     // MISCONFIGURED, not a guess.

    uint16_t distance_to_stopline_m; // Surveyed metres. GW_DISTANCE_UNSET
                                     // if never provisioned.
                                     //
                                     // DYNAMIC PER JUNCTION. This is the
                                     // reason it is on the wire at all --
                                     // intersection layouts differ, so the
                                     // number belongs in per-node config,
                                     // not in firmware. Whole metres are
                                     // enough: the quantity it feeds is a
                                     // transit-time window several seconds
                                     // wide, and survey error exceeds 1 m.

    uint32_t config_hash;            // Hash of this node's entire config
                                     // set. The ICU compares it against
                                     // what it expects for this node.
                                     //
                                     // Catches the failure where a node is
                                     // flashed with a neighbouring lane's
                                     // config: every field is individually
                                     // plausible, so no range check fires,
                                     // and the node reports confidently
                                     // about the wrong piece of road.

    // ---------------- v5: SELF-TEST ----------------

    uint8_t  mic_noise_floor;        // Ambient RMS, 0..255, log-scaled.
                                     //
                                     // A dead, unplugged or obstructed
                                     // microphone reports at or near ZERO.
                                     // Real roadside ambient never does.
                                     //
                                     // This is the specific fault v4 could
                                     // not represent (spec S1-04): such a
                                     // node passes every other health
                                     // check and simply never detects
                                     // anything, indefinitely. It is also
                                     // the fault a two-node design is
                                     // least able to notice on its own,
                                     // because the partner node keeps
                                     // working and the lane looks merely
                                     // quiet.
                                     //
                                     // An implausibly HIGH floor matters
                                     // too: it means the classifier is
                                     // working against a noise wall and
                                     // its confidence numbers no longer
                                     // mean what they meant in the lab.

    uint16_t loop_liveness;          // Free-running counter, incremented by
                                     // the inference loop, wraps freely.
                                     //
                                     // Proves the DETECTION path is alive,
                                     // not just the radio path. A hung or
                                     // crashed inference task with a
                                     // healthy comms task still heartbeats
                                     // perfectly -- that node is deaf and
                                     // looks fine. If this value is
                                     // unchanged across consecutive
                                     // heartbeats, the node is not
                                     // listening.

    uint8_t  tag[8];
};

/***********************************************************************
 * COMPILE-TIME SIZE LOCK
 *
 * This is the whole point of the file. If the two copies ever drift, you
 * now get a BUILD failure with a clear message instead of a system that
 * boots fine, links fine, and silently never preempts.
 ***********************************************************************/
// v5 sizes. FrameHeader is DELIBERATELY unchanged at 14: GW_COUNTER_OFFSET
// and GW_EPOCH_OFFSET below are byte positions the signing code writes
// into the raw buffer, and moving them breaks every signature silently.
static_assert(sizeof(FrameHeader)        == 14, "FrameHeader size drift - the two copies of GreenwaveTypes.h are out of sync");
static_assert(sizeof(LoRaEventFrame)     == 58, "LoRaEventFrame size drift - v5 is 58 (56 + heading_deg). Copy this file to the other sketch folder");
static_assert(sizeof(AcousticEventFrame) == 30, "AcousticEventFrame size drift - v5 is 30 (24 + event_id + age_ms). Copy this file to the other sketch folder");
static_assert(sizeof(HeartbeatFrame)     == 42, "HeartbeatFrame size drift - v5 is 42 (32 + geometry + self-test). Copy this file to the other sketch folder");
static_assert(sizeof(TelemetryPayload)   == 33, "TelemetryPayload size drift vs the copy inside EVU2.ino");

// Largest IF-2 frame, used to size receive buffers.
// 128 covers the 118-byte SessionEstablishFrame when GW_CERT_ON_AIR is
// enabled; steady-state frames are at most 56.
#define GW_MAX_FRAME_LEN 128

/***********************************************************************
 * COUNTER STAMPING OFFSET  (v3.2)
 *
 * radiateToICU() writes the counter directly into the outgoing byte
 * buffer at this offset, because the counter must reflect TRANSMIT order
 * rather than build order (see the comment in radiateToICU). That means
 * one place in the code depends on the byte position of a struct field,
 * which is exactly the kind of thing that breaks silently when somebody
 * reorders FrameHeader. The static_assert below makes it break loudly.
 ***********************************************************************/
#define GW_COUNTER_OFFSET 6
static_assert(offsetof(FrameHeader, counter) == GW_COUNTER_OFFSET,
              "FrameHeader.counter moved - update GW_COUNTER_OFFSET and radiateToICU()");

// v4: gwSignFrame() stamps session_epoch into the raw buffer at this
// offset for the same reason -- the tag covers the epoch, so the epoch
// must be written before the tag is computed.
#define GW_EPOCH_OFFSET 10
static_assert(offsetof(FrameHeader, session_epoch) == GW_EPOCH_OFFSET,
              "FrameHeader.session_epoch moved - update GW_EPOCH_OFFSET and gwSignFrame()");

/***********************************************************************
 * VEHICLE TRUST DATABASE ENTRY (lane node only)
 ***********************************************************************/
#define MAX_TRACKED_VEHICLES 8

struct VehicleTrust
{
    char vehicleID[8];
    bool used;
    uint8_t publicKey[32];
    bool keyValid;
    uint32_t lastSequence;
    bool sequenceStarted;
    unsigned long lastSeen;
};

/***********************************************************************
 * RETRO VERIFICATION BUFFER ENTRY (lane node only)
 ***********************************************************************/
#define RETRO_BUFFER_SIZE 6
#define RETRO_TIMEOUT_MS 15000

struct RetroPacket
{
    bool used;
    char vehicleID[8];
    uint8_t buffer[256];
    size_t len;
    int rssi;
    float snr;
    unsigned long time;
};

/***********************************************************************
 * SESSION ESTABLISHMENT FRAME -- 118 octets  (SOP 5.1)
 *
 * ONLY transmitted when GW_CERT_ON_AIR is 1 in GreenwaveCrypto.h. It is
 * 0 by default: with RDU public keys pre-provisioned at the ICU, the
 * session_epoch in every steady-state header is sufficient for the ICU
 * to derive the right key, and this frame never needs to exist.
 *
 * At 118 bytes / 763 ms it is by far the largest frame in the system --
 * 1.8x a normal event -- on a link where transmit time is directly
 * ambulance-deafness. Enable only for field-swappable RDUs.
 ***********************************************************************/
struct __attribute__((packed)) SessionEstablishFrame {
    FrameHeader hdr;

    uint8_t rdu_public_key[32];   // this node's X25519 public key
    uint8_t ca_signature[64];     // Root CA Ed25519 sig over {RDU_ID || pubkey}

    uint8_t tag[8];               // proof of possession under the new session key
};

static_assert(sizeof(SessionEstablishFrame) == 118,
              "SessionEstablishFrame size drift - copy this file to the other sketch folder");

/***********************************************************************
 * ROAD MODEL
 ***********************************************************************/
struct GPSPoint
{
    double lat;
    double lon;
};

#endif // GREENWAVE_TYPES_H
