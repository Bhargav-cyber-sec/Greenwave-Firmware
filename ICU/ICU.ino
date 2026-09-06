/***********************************************************************
 * GREENWAVE ICU (CENTRAL NODE) -- MINIMAL COMMS-ONLY BUILD, PROTOCOL v3
 *
 * Purpose: verify the EVU -> Lane Node -> ICU chain actually works,
 * visible on this board's Serial Monitor, with nothing else in the way.
 *
 * STILL REMOVED vs. the full build (add back later, one at a time):
 *   - Relays / traffic light GPIO
 *   - Battery ADC reading
 *   - WiFi / MQTT
 *   - FreeRTOS multi-task split -- everything runs in plain loop()
 *
 * ===================================================================
 * WHAT CHANGED IN v3 AND WHY  (read this before flashing)
 * ===================================================================
 *
 * 1. INTERSECTION ID IS NOW CHECKED.
 *    The old code received intersection_id in every packet and never
 *    looked at it -- not once, in any of the three handlers. At SF9 the
 *    radio horizon is kilometres. That means the moment you deploy a
 *    SECOND intersection, its lane nodes write into THIS intersection's
 *    node database and trigger THIS intersection's preemption. The
 *    system worked only because there was exactly one of everything.
 *
 * 2. PACKETS ARE AUTHENTICATED.
 *    The old link had no cryptography at all. Anyone could transmit a
 *    struct on 434.5 MHz with sync word 0xA5 and own the intersection --
 *    without ever touching the ambulance link or its Ed25519
 *    signatures. All of that certificate work was bypassable by
 *    attacking the cheaper link. Every frame now carries an AES-CMAC
 *    tag under a per-node key.
 *
 * 3. REPLAY IS BLOCKED.
 *    Even with a MAC, a recording of a genuine frame is still a genuine
 *    frame. Every frame carries a counter that never resets (not even
 *    across a power cut), and the ICU rejects anything not strictly
 *    newer than what it has already seen from that node.
 *
 * 4. PACKET TYPE IS READ, NOT GUESSED.
 *    The old pollLoRa() decided what a packet WAS by comparing its
 *    LENGTH to sizeof() of each struct. If the two copies of
 *    GreenwaveTypes.h drifted by one field, every packet became
 *    "[UNKNOWN PACKET]" and the link died silently with no clue as to
 *    why. Now: version byte, then type byte, then length is VALIDATED
 *    against that type.
 *
 * 5. sig_status IS TREATED AS DIAGNOSTIC, NOT AUTHORISATION.
 *    It is a claim a lane node makes about work it says it did. Trust
 *    on this link comes from the CMAC, not from a string in a payload.
 *
 * PIN NOTE: LORA_SCK/MISO/MOSI/SS/RST/DIO0 are carried over unchanged.
 * I don't have a pin diagram for this specific ESP32-S3 module -- these
 * are only as good as the wiring assumption that produced them.
 ***********************************************************************/

#include <SPI.h>
#include <LoRa.h>

// CHANGED v3.6: declare this build's role BEFORE including GreenwaveCrypto.h.
//
// RESOLVED IN v4 (SOP 3.2). GW_NODE_KEYS is gone. An ICU image contains the
// ICU's own private key plus the six RDU PUBLIC keys -- dumping it yields
// nothing that forges a node. A lane-node image contains only its own
// private key. The all-six-symmetric-keys-in-every-image problem this
// comment described no longer exists.
//
// The header fix is to wrap the table so a lane node compiles only its own
// slot and the ICU compiles all six. Defining the role here is harmless
// against the current header and makes that change a drop-in, with no edit
// needed in either sketch. The ICU legitimately needs all six keys -- so
// note that this relocates the concentration risk to one cabinet per
// intersection rather than eliminating it. Say that in the spec.
#define GW_ROLE_ICU

#include "GreenwaveTypes.h"
#include "GreenwaveCrypto.h"

// EDIT 1 of 4  (PHASE 2)
// The ICU <-> ERC text protocol. Shared byte-identical with the ERC
// sketch folder, the same rule as GreenwaveTypes.h. Transport for it
// lives in ICU_Link.ino, which is a second tab in THIS sketch folder --
// the Arduino IDE compiles every .ino in a folder into one program.
#include "GreenwaveLink.h"

/***********************************************************************
 * MUST BE INCLUDED FROM THE MAIN SKETCH, NOT FROM ICU_EVU.ino.
 *
 * The Arduino builder concatenates every .ino in the folder with this
 * file FIRST, then inserts auto-generated prototypes for ALL tabs near
 * the top of the combined file -- above the point where ICU_EVU.ino's
 * own #include would run.
 *
 * So a header included only by the tab that needs it arrives too late:
 * the prototypes naming EvuTrack and VehicleRegistryEntry are emitted
 * before those types exist, and the build fails with "does not name a
 * type" even though the header is correct and included.
 *
 * Including it HERE puts the definitions above the insertion point,
 * because the generator places prototypes after the last #include in
 * the main sketch.
 *
 * The tab includes it too; the guard makes that harmless, and it keeps
 * the tab readable on its own.
 ***********************************************************************/
#include "ICU_EvuTypes.h"

/***********************************************************************
 * CROSS-TAB DECLARATIONS
 *
 * ICU.ino, ICU_Geometry.ino, ICU_Link.ino and ICU_Tracks.ino are four
 * tabs of ONE sketch -- the IDE concatenates every .ino in the folder
 * into a single translation unit.
 *
 * The IDE also auto-generates function prototypes, but it does so
 * unreliably across tabs: it inserts them near the top of the main
 * sketch, and functions whose definitions live in a later tab are not
 * always picked up. That produced "'gwPrintGeometry' was not declared in
 * this scope" even though the function exists and the sketch is
 * otherwise correct.
 *
 * Declaring them explicitly removes the dependency on that behaviour.
 * It is also better documentation: this list is exactly the surface
 * ICU.ino uses from the other three tabs, and anything not on it is
 * internal to its own file.
 ***********************************************************************/

// ---- ICU_Link.ino : console transport ----
void ercInit();
void ercService();
void ercBenchConsole();
void ercPrintStats();
void ercAlert(int lane, uint8_t stage, uint8_t priority,
              uint8_t evidence, int16_t etaSeconds);
void ercHold(int lane);
void ercRelease(int lane, uint8_t reason);
void ercHealth(int lane, int node, uint8_t role, uint8_t health, uint16_t distM);
void ercClearAll(uint8_t reason);
bool ercIsLinkUp();
bool ercIsAcked();

// ---- ICU_Geometry.ino : 5A, surveyed geometry and node health ----
struct HeartbeatFrame;                      // defined in GreenwaveTypes.h
void     gwGeometryOnHeartbeat(const HeartbeatFrame &f);
void     gwUpdateNodeHealth();
void     gwPrintGeometry();
uint8_t  gwFarNode(int lane);
uint8_t  gwNearNode(int lane);
uint16_t gwSeparationM(int lane);
bool     gwApproachUsable(int lane);
uint8_t  gwNodeHealth(int lane, int node);
uint8_t  gwNodeRole(int lane, int node);

// ---- ICU_Tracks.ino : 5B, acoustic tracks and direction ----
struct AcousticEventFrame;                  // defined in GreenwaveTypes.h
void    gwTrackOnAcoustic(const AcousticEventFrame &f);
void    gwUpdateTracks();
void    gwPrintTracks();
void    gwPrintWindow(int lane);
uint8_t gwAcousticEvidence(int lane);
uint8_t gwAcousticState(int lane);
float   gwAcousticConfidence(int lane);
void    gwMarkTrackServed(int lane);
void    gwSimApproach(int lane, long deltaMs, float conf, bool sustained);
void    gwSimSingle(int lane, bool useFar, float conf, bool sustained);
void    gwSimReset(int lane);
void    gwSimReassert(int lane);
long    gwAcousticDeltaMs(int lane);
uint16_t gwNearDistanceM(int lane);
uint16_t gwNodeDistanceM(int lane, int node);

// ---- ICU_Decision.ino : 5C/5D/5E, demand, ETA, staging ----
void gwUpdateDecision();
void gwPrintDemand();

// ---- ICU_EVU.ino : Scenario 2, authenticated vehicle tracks ----
struct LoRaEventFrame;                      // defined in GreenwaveTypes.h
void gwEvuOnRelay(const LoRaEventFrame &f, bool directToIcu);
void gwUpdateEvuTracks();
void gwPrintEvuTracks();
void gwSimEvuApproach(int lane, float distM, float speedKmph);
void gwSimEvuStep(int lane, float seconds);
void gwSimEvuDiverge(int lane);
void gwSimEvuTeleport(int lane, float jumpM);
void gwSimEvuEmergency(int lane, bool on);
void gwSimEvuClear(int lane);
void gwSimEvuRun(int lane, int count, float secondsPerStep);
bool    gwEvuDemand(int lane, uint8_t *priority, uint8_t *evidence, int16_t *etaS);
bool    gwEvuIsReceding(int lane);
uint8_t gwEvuReleaseReason(int lane);
uint8_t gwEvuEndState(int lane);
void    gwEvuDropTrack(const char *id);
float   gwEvuDistanceM(int lane);

// ---- ICU_Safety.ino : 5E abuse controls, 5F watchdog ----
void  gwWatchdogInit();
void  gwWatchdogFeed();
bool  gwCommitAllowed(int lane);
void  gwCommitRecord(int lane);
void  gwNoteRfNoise(int rssi);
bool  gwRfBandNoisy();
float gwRfNoiseFloor();
void  gwNoteLinkSuspect(int lane, int node);
void  gwPrintSafety();

// ---- ICU_Site.ino : per-junction geometry, held in NVS ----
void   gwSiteLoad();
void   gwSitePrint();
bool   gwSiteCommand(int argc, char **argv);
double gwSiteLat();
double gwSiteLon();
float  gwSiteBearing(int lane);
bool   gwSiteProvisioned();
void  ercQueue(const uint8_t *lanes, const uint8_t *prios, uint8_t count);
void  gwAbuseFill(int lane);
void  gwAbuseReset(int lane);

// =====================================================
// CONFIG
// =====================================================

// CHANGED v3: numeric, and actually enforced. See MY_INTERSECTION_ID
// checks in pollLoRa().
#define MY_INTERSECTION_ID      1

#define TOTAL_LANES             3
#define NODE1_ID                1
#define NODE2_ID                2

// CHANGED v3: was 60000 against a 20000ms heartbeat -- only 3 heartbeats
// of margin on a link with no acknowledgement and no retry. Lane nodes
// now heartbeat at 5000ms +/- jitter, so 20000ms gives 4 clean misses
// before a node is called offline.
// CHANGED v3.2: 20000 -> 25000, sized against a measured 5.8-8.1s spacing.
//
// CHANGED v3.6: 25000 -> 40000. The v3.2 measurement went stale.
//
// TC-LN-001 (v3.5 run, 2026-07-29) measured heartbeat spacing at the ICU of
// min 5.42s / mean 6.96s / MAX 13.65s. Against 25000 that is 1.8 clean
// misses of margin, not the 3 the comment above claimed -- and that run had
// ZERO acoustic traffic, so it is a floor. Periodic acoustic re-assert adds
// a third frame class competing for the same 560ms window.
//
// Raising the timeout alone would have been the wrong fix: it treats the
// symptom and the number goes stale again the moment traffic changes. The
// real fix is in LaneNode.ino -- HEARTBEAT_MAX_DEFER_MS bounds how long a
// heartbeat may be held waiting for a clear window, making worst-case
// spacing a DESIGNED quantity this timeout can be derived from.
//
// Worst spacing is the SUM of the lane node's three terms plus airtime:
//   HEARTBEAT_BASE_MS 5000 + HEARTBEAT_JITTER_MS 1500
//   + HEARTBEAT_MAX_DEFER_MS 6000 + airtime 271  =  ~12.8 s
//   40000 / 12800 = 3.1 clean misses.
//
// Note the first draft of this pairing used a 12000ms defer on the mistaken
// belief that the defer term WAS the worst spacing rather than one addend
// of it. Simulation measured 18050ms -- 2.2 misses, worse than the 1.8 the
// change existed to fix. Recompute the SUM when you touch any term.
//
// CORRECTED v3.7 -- THE DERIVATION ABOVE WAS INCOMPLETE, NOT WRONG.
//
// It bounded TRANSMIT-SIDE spacing and silently assumed zero IF-2 loss.
// TC-LN-001 (v3.6 run, 09:32-09:35) measured 14.43s spacing at this end
// while the lane node reported hbMaxGap=8430ms and hbForced=0 -- i.e.
// scheduling behaved perfectly and the extra 6s was ONE HEARTBEAT LOST IN
// THE AIR on a link measured at 96.3% delivery (26 of 27).
//
// Loss-inclusive worst case, allowing one consecutive loss:
//   (5000 + 1500 + 6000 + 271) x 2  =  ~25.6 s
//   40000 / 25600 = 1.6 clean misses, NOT the 3.1 computed above.
//
// 40000 IS RETAINED DELIBERATELY. Raising it to 60000 would buy 2.3 misses
// against the loss-inclusive case, but it would do so by making a node that
// is genuinely dead take a full minute to be reported. That trades
// detection speed for a problem whose real fix is on the radio side, not in
// a timeout constant. Observed margin is 40000/14430 = 2.8 misses, which is
// adequate; the number to watch is IF-2 delivery, not this constant.
//
// If IF-2 heartbeat delivery drops below ~90%, revisit -- but fix the link
// first. Stating the assumption is the point: the previous comment claimed
// a margin the system did not have.
// Raised 40000 -> 60000 alongside NODE_FAILED_MS in ICU_Geometry.ino.
//
// The RDU heartbeat interval doubled to 10 s as part of the duty-cycle
// fix, so worst-case spacing from a healthy node is now 17.5 s. Leaving
// this at 40 s would have marked nodes offline here while the geometry
// layer still considered them merely SUSPECT -- two subsystems
// disagreeing about whether the same node is alive.
#define HEARTBEAT_TIMEOUT_MS    60000

// CHANGED v3.7: the banner used to print these two numbers as hardcoded
// string literals. When HEARTBEAT_MAX_DEFER_MS was corrected 12000 -> 6000
// in LaneNode.ino, the #define and both derivation comments were updated
// and the literal in the Serial.println was not -- so the v3.6 banner
// announced 12000 against a lane node built with 6000.
//
// That is the exact drift this banner exists to catch, in the one line
// whose whole job is catching it. A banner that lies is worse than no
// banner, because it converts "unknown" into "confidently wrong".
//
// Now they are constants, printed with %lu. Still not compiler-checkable
// across two sketch folders -- nothing can be -- but there is now exactly
// ONE place per value to change, and it sits next to the timeout it is
// paired with.
#define EXPECTED_LANE_NODE_DEFER_MS      6000UL
#define EXPECTED_ACOUSTIC_REASSERT_MS    8000UL

// CHANGED v3.7: radio parameters were literals in the setter calls AND
// retyped again in the "Central Ready" banner. Same class as the defer
// constant: the banner cannot catch a mismatch it is retyped from. A sync
// word mismatch in particular is silent -- the radio just never reports a
// packet -- so it is the worst constant to duplicate.
// MUST MATCH LORA_SYNC_WORD_ICU AND LORA_SF IN LaneNode.ino.
/***********************************************************************
 * MUST MATCH LORA_SF_ICU IN RDU.ino -- currently 7.
 *
 * Lowered from 9 as part of the duty-cycle fix. The node-to-ICU link is
 * a fixed ~250 m path between two boxes that never move, so it does not
 * need SF9's range and was paying 4x the airtime for it. Two nodes
 * sharing this channel measured 41% duty during a live event against a
 * 10% ceiling.
 *
 * A node at SF7 and an ICU at SF9 ARE COMPLETELY DEAF TO EACH OTHER.
 * There is no partial failure, no error, no warning -- they simply
 * never hear one another, and the symptom is an ICU that boots
 * perfectly and receives nothing at all.
 *
 * If you change one, change both, and reflash everything together.
 ***********************************************************************/
#define ICU_LORA_SF        7
#define ICU_LORA_SYNC_WORD 0xA5

// CHANGED v3.5: 35000 -> 20000.
//
// This is the one v3.4 timer that had to move in the OPPOSITE direction
// from the rest. 35000 was compensating for a bug, not sizing a margin:
// the lane node emitted acoustic detection on the rising edge only, so
// acousticDetected was a latch that NOTHING ever refreshed, and widening
// its expiry just meant the ICU could preempt on evidence half a minute
// stale. Longer is not safer when the input is never renewed.
//
// LaneNode.ino v3.5 now re-asserts every ACOUSTIC_REASSERT_MS (8000), so
// this is a real timeout again: 20000 = 2.5 clean misses, the same margin
// rule as HEARTBEAT_TIMEOUT_MS above.
//
// These two constants are ONE decision expressed in two files. If you
// change ACOUSTIC_REASSERT_MS, change this with it.
#define EVENT_TIMEOUT_MS        20000

#define EMERGENCY_QUEUE_SIZE    10
#define EMERGENCY_HOLD          40000

// CHANGED v3.5: NEW, and read the reasoning before deleting it.
//
// Periodic re-assert (LaneNode v3.5) introduced a behaviour that did not
// exist before: a lane whose siren is still audible RE-VALIDATES after its
// hold completes and re-enters the queue. For a single ambulance that is
// exactly right -- it converts a blind fixed 40 s timer into a green that
// renews on evidence, which is what OPEN-007 has been asking for.
//
// But the renewal is bounded only by acoustic detection fading. It is NOT
// bounded by the vehicle passing: loraDetected will keep refreshing long
// after the ambulance has cleared, because the geofence is bypassed and
// the EVU stays in radio range for kilometres. The AND-gate's acoustic
// half is the only thing that ends the green, and its range has never
// been measured (OPEN-009).
//
// So: after a lane's hold completes, that lane cannot start another one
// until cross traffic has had this long. It bounds preemption duty on any
// single approach at EMERGENCY_HOLD/(EMERGENCY_HOLD+LANE_COOLDOWN_MS) =
// 80%, and guarantees conflicting movements are served. It does NOT
// interrupt a hold in progress and cannot shorten a green under a vehicle.
//
// This is a floor, not a policy. Real arbitration -- priority ordering, a
// starvation bound, preemption-of-preemption -- is still undefined
// (OPEN-011), and `priority` is still carried in every frame and never read.
#define LANE_COOLDOWN_MS        10000

// CHANGED v3.6: INTERIM ARBITRATION POLICY. Read this before relying on it.
//
// The queue was strict FIFO by order of validation, and `priority` was
// carried in every LoRaEventFrame, stored in NodeState, printed in the log,
// and read by NO decision anywhere in the system. A priority-1 vehicle
// arriving second waited behind a lower-priority one, which is the single
// thing a priority field exists to prevent.
//
// Policy now implemented:
//   1. Any lane that has waited >= QUEUE_STARVATION_MS is served first
//      (oldest such lane wins). This is the starvation bound.
//   2. Otherwise the most urgent priority class wins (LOWER numeric value
//      = more urgent; priority_class 1 is the highest in EVU2.ino).
//   3. Ties broken by longest wait.
//
// What this deliberately does NOT do: preempt a hold already in progress.
// Interrupting a green under a moving vehicle to serve a more urgent one is
// a signal-safety decision that needs a traffic engineer, not a firmware
// author, and it interacts with the yellow/all-red/pedestrian-clearance
// handling this build does not have.
//
// This is a defensible interim policy, not a specified one. OPEN-011 stays
// open until someone with authority over the deployment signs off on the
// ordering rule, the starvation bound, and the preemption-of-preemption
// question. Written down here so the next reader knows which it is.
#define QUEUE_STARVATION_MS     120000

// priority_class 0 means "not stated". Treat it as least urgent rather than
// most urgent, which is what a raw numeric compare would do.
#define PRIORITY_UNKNOWN_RANK   255

// CHANGED v3: runtime, not compile-time.
//
// This was `#define REQUIRE_DUAL_NODE_CONFIRMATION false` -- i.e. every
// board shipped in single-node bench mode, and there was NO WAY to tell
// a bench build from a production build by looking at a running unit. A
// bench build reaching a live intersection silently removes the two-node
// corroboration that the entire false-positive resistance depends on.
// Now it is a variable, reported at boot and (once MQTT is back) in
// central/health.
bool requireDualNodeConfirmation = false;   // set true for production

#define ACOUSTIC_WINDOW_MS      30000

#define LORA_SCK   12
#define LORA_MISO  13
#define LORA_MOSI  11
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  8

// Must match LaneNode.ino's CENTRAL_FREQ and SF9/125kHz/CR4-6/Preamble12
// exactly, or packets won't demodulate at all.
#define CENTRAL_FREQ 434.5E6

// =====================================================
// RECEIVE BUFFER
// =====================================================
static uint8_t rxFrame[GW_MAX_FRAME_LEN];

// =====================================================
// NODE STATE
// =====================================================
struct NodeState
{
    bool online = false;
    bool loraDetected = false;
    bool acousticDetected = false;

    float confidence = 0.0;

    // CHANGED v3: replay state. Every frame from this node must carry a
    // counter strictly greater than this one.
    uint32_t lastCounter = 0;
    bool     counterStarted = false;

    // CHANGED v3.7: live IF-2 loss measurement.
    //
    // The per-node counter increments on EVERY frame that node transmits,
    // of every type. So a gap between consecutive ACCEPTED counters is
    // exactly the number of frames that did not survive the air -- no
    // inference, no cross-log correlation, no dependence on serial capture
    // fidelity at either end.
    //
    // This matters because IF-2 loss is now a load-bearing number: the
    // HEARTBEAT_TIMEOUT_MS derivation depends on it (see above), and the
    // v3.6 run had to be measured after the fact by diffing three logs,
    // which produced a 20-point error on the first attempt.
    uint32_t framesAccepted = 0;
    uint32_t framesLostIf2  = 0;

    int lastRSSI = 0;            // IF-1 (ambulance -> node) link quality
    float lastSNR = 0;
    int lastIf2RSSI = 0;         // CHANGED v3: IF-2 (node -> ICU) link quality
    float lastIf2SNR = 0;

    // CHANGED v3: split apart. The old code set lastHeartbeat = millis()
    // inside the EVENT handlers too, which meant a node whose heartbeat
    // path had completely failed still looked "online" as long as it was
    // sending events -- masking the exact fault the heartbeat exists to
    // detect.
    unsigned long lastHeartbeat = 0;
    unsigned long lastSeen = 0;

    unsigned long lastLoRa = 0;
    unsigned long lastAcoustic = 0;

    char vehicleID[8] = {0};
    int32_t latitude = 0;        // x10^7, as received
    int32_t longitude = 0;
    float speed = 0;
    uint8_t motionState = MOTION_UNKNOWN;
    uint8_t sigStatus = SIG_UNVERIFIED;
    uint8_t priority = 0;
    uint32_t gpsEpoch = 0;
    uint32_t txSequence = 0;
    bool benchBuild = false;

    // Health counters
    uint32_t rejectedMac = 0;
    uint32_t rejectedReplay = 0;
};

NodeState lane1node1, lane1node2;
NodeState lane2node1, lane2node2;
NodeState lane3node1, lane3node2;

// Global reject counters -- the only externally visible sign of an
// active attack on IF-2.
uint32_t rejWrongVersion = 0;
uint32_t rejWrongIntersection = 0;
uint32_t rejBadLength = 0;
uint32_t rejUnknownNode = 0;
uint32_t rejReplay = 0;      // counter went BACKWARDS - investigate
uint32_t dupSuppressed = 0;  // exact repeat of a frame we accepted - benign
uint32_t rejMac = 0;

// v4 (SOP 3.5 / 5.3). Extends the existing counter pattern -- every new
// rejection path gets its own counter and its own log line. Averaging a
// revocation into a generic "bad MAC" total is how a real event
// disappears into noise.
uint32_t rejRevoked      = 0;   // SOP 3.5: node revoked at session setup
uint32_t rejNoSession    = 0;   // key derivation failed
uint32_t rejEpoch        = 0;   // epoch older than the overlap window
uint32_t rejUnknownNodeK = 0;   // no provisioned public key for this node

// v4: session cache with epoch-overlap window (SOP 5.3).
GwIcuSessionCache gwIcuSessions;

// =====================================================
// EMERGENCY QUEUE
//
// CHANGED v3.6: was a ring buffer of bare lane numbers. It now carries the
// priority class and the enqueue time, because selection needs both. A
// compacting array replaces the ring: at a maximum depth of 10 the shift
// costs nothing and it makes "remove the best entry, wherever it is"
// expressible without ring-index arithmetic.
// =====================================================
struct QueueEntry {
    uint8_t lane;
    uint8_t priority;        // as received; 0 = not stated
    unsigned long queuedAt;
};

QueueEntry emergencyQueue[EMERGENCY_QUEUE_SIZE];
int queueCount = 0;

bool emergencyRunning = false;
int activeEmergencyLane = 0;
unsigned long emergencyStartTime = 0;

// CHANGED v3.5: when each lane's last hold ENDED. Indexed by lane (1..3);
// index 0 unused so the lane number can be used directly without an
// off-by-one. 0 = this lane has never held. See LANE_COOLDOWN_MS.
unsigned long lastHoldEnd[TOTAL_LANES + 1] = {0};

// =====================================================
// NODE LOOKUP
// =====================================================
NodeState* getNodeState(uint8_t lane, uint8_t node) {
    if (lane == 1) return (node == 1) ? &lane1node1 : (node == 2 ? &lane1node2 : nullptr);
    if (lane == 2) return (node == 1) ? &lane2node1 : (node == 2 ? &lane2node2 : nullptr);
    if (lane == 3) return (node == 1) ? &lane3node1 : (node == 2 ? &lane3node2 : nullptr);
    return nullptr;
}

// =====================================================
// QUEUE HELPERS
// =====================================================
bool queueIsEmpty() { return queueCount == 0; }
bool queueIsFull()  { return queueCount >= EMERGENCY_QUEUE_SIZE; }

// Rank a priority class for comparison. Lower rank = served earlier.
// priority_class 0 ("not stated") ranks last, not first.
static inline uint8_t priorityRank(uint8_t p) {
    return (p == 0) ? PRIORITY_UNKNOWN_RANK : p;
}

bool laneAlreadyQueued(int lane) {
    if (activeEmergencyLane == lane) return true;
    for (int i = 0; i < queueCount; i++) {
        if (emergencyQueue[i].lane == lane) return true;
    }
    return false;
}

bool enqueueEmergency(int lane, uint8_t priority) {
    if (queueIsFull()) {
        Serial.println("[QUEUE] FULL");
        return false;
    }
    emergencyQueue[queueCount].lane     = (uint8_t)lane;
    emergencyQueue[queueCount].priority = priority;
    emergencyQueue[queueCount].queuedAt = millis();
    queueCount++;
    Serial.printf("[QUEUE] Added Lane %d PRI=%u depth=%d\n",
                  lane, (unsigned)priority, queueCount);
    return true;
}

// CHANGED v3.6: selection, not just removal. See QUEUE_STARVATION_MS.
//
// Returns 0 when empty. Writes the chosen entry's priority and wait time
// out through the pointers so the caller can log WHY this lane was picked
// -- an arbitration decision nobody can see is an arbitration decision
// nobody can debug at 2am.
int dequeueEmergency(uint8_t *outPriority, unsigned long *outWaitedMs) {
    if (queueIsEmpty()) return 0;

    unsigned long now = millis();
    int best = -1;

    // Pass 1: starvation. Any lane past the bound outranks everything,
    // regardless of priority class. Oldest such lane wins.
    for (int i = 0; i < queueCount; i++) {
        if ((now - emergencyQueue[i].queuedAt) >= QUEUE_STARVATION_MS) {
            if (best < 0 ||
                emergencyQueue[i].queuedAt < emergencyQueue[best].queuedAt) {
                best = i;
            }
        }
    }

    // Pass 2: most urgent priority class, ties broken by longest wait.
    if (best < 0) {
        for (int i = 0; i < queueCount; i++) {
            if (best < 0) { best = i; continue; }
            uint8_t ri = priorityRank(emergencyQueue[i].priority);
            uint8_t rb = priorityRank(emergencyQueue[best].priority);
            if (ri < rb ||
                (ri == rb && emergencyQueue[i].queuedAt < emergencyQueue[best].queuedAt)) {
                best = i;
            }
        }
    }

    if (best < 0) return 0;   // unreachable, but do not return junk

    int lane = emergencyQueue[best].lane;
    if (outPriority)  *outPriority  = emergencyQueue[best].priority;
    if (outWaitedMs)  *outWaitedMs  = now - emergencyQueue[best].queuedAt;

    // Compact. Depth <= 10, so this is cheaper than maintaining ring indices.
    for (int i = best; i < queueCount - 1; i++) emergencyQueue[i] = emergencyQueue[i + 1];
    queueCount--;

    return lane;
}

// =====================================================
// HEARTBEAT / EVENT TIMEOUT CHECKS
// =====================================================
// CHANGED v3.6: offline no longer wipes detection state.
//
// This change and the one in processLoRaEvent()/processAcousticEvent()
// below MUST land together. Separately, either one is a bug:
//
//   - Event handlers used to set online = true, so checkHeartbeats() would
//     mark a node offline and the next event marked it online again. The
//     node oscillated instead of latching, defeating the whole point of
//     making heartbeats the only proof of life.
//
//   - But if events stop setting online while THIS function still clears
//     loraDetected/acousticDetected, then a node whose heartbeats are being
//     lost -- measured at 13.65s spacing in TC-LN-001 v3.5 -- gets marked
//     offline and has its LIVE DETECTIONS DISCARDED. That converts a
//     cosmetic logging bug into missed preemptions.
//
// So: "offline" is a health signal about the heartbeat path. Detection
// freshness is a separate question already answered by EVENT_TIMEOUT_MS in
// checkEventTimeouts(). Two different facts, two different timers, no
// coupling between them. A node that is still delivering authenticated
// detections keeps them until they age out on their own.
void markNodeOffline(int lane, int node, NodeState* state) {
    if (state->online) {
        state->online = false;
        Serial.printf("[NODE OFFLINE] L%dN%d  (heartbeat path silent; any live "
                      "detections retained until they age out)\n", lane, node);
    }
}

void checkHeartbeats() {
    unsigned long now = millis();
    struct { int lane; int node; NodeState* ptr; } nodes[] = {
        {1,1,&lane1node1}, {1,2,&lane1node2},
        {2,1,&lane2node1}, {2,2,&lane2node2},
        {3,1,&lane3node1}, {3,2,&lane3node2}
    };
    for (auto &n : nodes) {
        // CHANGED v3: keyed on lastHeartbeat only. Event traffic no
        // longer counts as proof of life -- a node whose heartbeat has
        // stopped is now reported offline even if events still arrive,
        // because a half-dead node is exactly what you want to know about.
        if (n.ptr->online && (now - n.ptr->lastHeartbeat) > HEARTBEAT_TIMEOUT_MS) {
            markNodeOffline(n.lane, n.node, n.ptr);
        }
    }
}

void checkEventTimeouts() {
    unsigned long now = millis();
    NodeState* nodes[] = { &lane1node1, &lane1node2, &lane2node1, &lane2node2, &lane3node1, &lane3node2 };
    for (int i = 0; i < 6; i++) {
        NodeState* n = nodes[i];
        if (n->loraDetected && (now - n->lastLoRa) > EVENT_TIMEOUT_MS)      n->loraDetected = false;
        if (n->acousticDetected && (now - n->lastAcoustic) > EVENT_TIMEOUT_MS) n->acousticDetected = false;
    }
}

// =====================================================
// PACKET HANDLERS
// =====================================================
void processHeartbeat(const HeartbeatFrame& f, NodeState* n) {
    // PHASE 5A: hand the surveyed geometry and self-test to the geometry
    // layer BEFORE anything else uses this frame. It derives FAR/NEAR by
    // sorting the two nodes' reported distances -- never from node_id,
    // which is an install-order label and was the source of the inverted
    // direction logic in v1.0 (spec defect S1-01).
    gwGeometryOnHeartbeat(f);

    bool wasOffline = !n->online;
    n->online = true;
    n->lastHeartbeat = millis();
    n->lastSeen = millis();
    n->benchBuild = (f.flags & HBF_BENCH_BUILD) != 0;

    if (wasOffline) {
        Serial.printf("[NODE ONLINE] L%dN%d%s\n", f.hdr.lane_id, f.hdr.node_id,
                      n->benchBuild ? "  *** BENCH BUILD ***" : "");
    }

    // CHANGED v3.1: battery reported as n/a when the ADC is unwired,
    // rather than as a critical reading.
    char battStr[16], lvlStr[8];
    if (f.flags & HBF_BATT_VALID) {
        snprintf(battStr, sizeof(battStr), "%umV", (unsigned)f.battery_mv);
        snprintf(lvlStr,  sizeof(lvlStr),  "%u",   (unsigned)f.battery_level);
    } else {
        snprintf(battStr, sizeof(battStr), "n/a");
        snprintf(lvlStr,  sizeof(lvlStr),  "-");
    }

    Serial.printf(
        "[HEARTBEAT] L%dN%d CTR=%lu UP=%lus HEAP=%uKB LORA=%d MIC=%d BATT=%s LVL=%s%s\n",
        f.hdr.lane_id, f.hdr.node_id, (unsigned long)f.hdr.counter,
        (unsigned long)f.uptime_s, (unsigned)f.free_heap_kb,
        (f.flags & HBF_LORA_OK) ? 1 : 0, (f.flags & HBF_MIC_OK) ? 1 : 0,
        battStr, lvlStr,
        n->benchBuild ? "  [BENCH]" : ""
    );
}

void processLoRaEvent(const LoRaEventFrame& f, NodeState* n, int if2rssi, float if2snr) {
    // CHANGED v3.6: does NOT set n->online. Only a heartbeat proves the
    // heartbeat path is alive; an event proves only that events arrive. A
    // half-dead node is exactly the thing worth knowing about, and letting
    // events mask it defeated the v3 split. See markNodeOffline().
    n->lastSeen = millis();
    n->loraDetected = true;
    n->lastLoRa = millis();

    n->lastRSSI = f.rssi_if1;
    n->lastSNR  = f.snr_if1_x10 / 10.0f;

    // CHANGED v3: the old handler took rssi/snr as arguments and then
    // threw them away, storing the IF-1 values instead. IF-2 link margin
    // was therefore unobservable anywhere in the entire system -- you
    // could not tell a healthy node link from one about to drop out.
    n->lastIf2RSSI = if2rssi;
    n->lastIf2SNR  = if2snr;

    memcpy(n->vehicleID, f.vehicle_id, 8);
    n->vehicleID[7] = '\0';
    n->latitude    = f.latitude;
    n->longitude   = f.longitude;
    n->speed       = f.speed_kmph / 100.0f;
    n->motionState = f.motion_state;
    n->sigStatus   = f.sig_status;
    // SCENARIO 2 / S2-01: this field is the vehicle's OWN CLAIM about its
    // importance, kept only for logging and for detecting a unit that
    // asserts an importance it was not granted.
    //
    // It MUST NOT reach arbitration. EVU2.ino compiles in
    // `#define PRIORITY_CLASS 1` and transmits it, so anyone who can
    // reflash a unit could otherwise outrank every genuine emergency in
    // the network -- and the packet would authenticate perfectly,
    // because it really is signed. It is simply lying.
    //
    // The authoritative priority comes from the registry in ICU_EVU.ino.
    n->priority    = f.priority;   // CLAIMED, advisory only
    n->gpsEpoch    = f.gps_epoch;
    n->txSequence  = f.tx_seq;

    Serial.printf(
        "[LORA RX] L%dN%d VEH=%s PRI=%d EMERG=%d GPSVALID=%d GEOFENCE=%s SIG=%d "
        "SPEED=%.1f TXSEQ=%lu GPSEPOCH=%lu | IF1 RSSI=%d SNR=%.1f | IF2 RSSI=%d SNR=%.1f\n",
        f.hdr.lane_id, f.hdr.node_id, n->vehicleID, f.priority,
        (f.flags & EVF_EMERGENCY) ? 1 : 0,
        (f.flags & EVF_GPS_VALID) ? 1 : 0,
        (f.flags & EVF_GEOFENCE_BYPASS) ? "BYPASSED" :
            ((f.flags & EVF_GEOFENCE_PASS) ? "PASS" : "FAIL"),
        f.sig_status, n->speed,
        (unsigned long)f.tx_seq, (unsigned long)f.gps_epoch,
        (int)f.rssi_if1, f.snr_if1_x10 / 10.0f, if2rssi, if2snr
    );

    // SCENARIO 2: hand the observation to the EVU track layer, which
    // applies the registry, plausibility, approach association and
    // divergence rules.
    //
    // directToIcu is false: this frame reached us relayed by an RDU, so
    // the ICU did not observe the transmission itself. That distinction
    // is carried through to the evidence class as EVU_INDIRECT.
    gwEvuOnRelay(f, false);
}

void processAcousticEvent(const AcousticEventFrame& f, NodeState* n) {
    // CHANGED v3.6: does NOT set n->online. See processLoRaEvent().
    n->lastSeen = millis();
    // ==================================================================
    // OPEN DESIGN DECISION -- NOT A BUG, DELIBERATELY NOT CHANGED
    //
    // acousticDetected is set for ANY accepted acoustic frame. Neither
    // ACF_SUSTAINED nor confidence influences validateLane().
    //
    // Observed in TC 21:39:50 -- a frame with sustained=0 (flags=0x01,
    // SCORE=15) triggered [EMERGENCY VALID]; the sustained=1 frame
    // (flags=0x03, SCORE=39) arrived 10 s later. So today, preemption
    // fires on the FIRST detection and the sustain flag is decoration.
    //
    // That may be exactly right: reacting on first detection minimises
    // latency, and a false positive costs one green phase while a false
    // negative costs an ambulance. But it is currently the DEFAULT rather
    // than a decision, and both ACF_SUSTAINED and confidence are collected
    // and then discarded -- the same pattern flagged for confidence in the
    // very first design review.
    //
    // Pick one and write it down:
    //   (a) keep as-is -> delete ACF_SUSTAINED, stop paying to transmit it
    //   (b) require sustained for preemption -> gate here, accept +8 s
    //       latency (ACOUSTIC_REASSERT_MS)
    //   (c) require sustained ONLY when no LoRa corroboration exists
    // Do not resolve this by leaving it undecided.
    // ==================================================================
    n->acousticDetected = true;
    n->lastAcoustic = millis();
    n->confidence = f.confidence / 255.0f;

    // PHASE 5B: hand the observation to the track layer, which infers
    // direction from the FAR/NEAR ordering established in 5A.
    //
    // The open question in the comment above -- whether ACF_SUSTAINED
    // should gate action -- is now answerable rather than defaulted.
    // Sustain and confidence are carried INTO the track and become part
    // of the evidence tuple, instead of being collected and discarded.
    gwTrackOnAcoustic(f);

    Serial.printf("[ACOUSTIC RX] L%dN%d CONF=%.2f MIC=%d SUSTAINED=%d\n",
                  f.hdr.lane_id, f.hdr.node_id, n->confidence,
                  (f.flags & ACF_MIC_OK) ? 1 : 0,
                  (f.flags & ACF_SUSTAINED) ? 1 : 0);
}

// =====================================================
// VALIDATION
// =====================================================
void validateLane(uint8_t lane) {
    NodeState* node1 = getNodeState(lane, 1);
    NodeState* node2 = getNodeState(lane, 2);
    if (node1 == nullptr) return;

    bool sirenValid;
    bool loraValid;

    if (requireDualNodeConfirmation) {
        if (node2 == nullptr) return;

        // CHANGED v3: unsigned-safe time difference. The old code did
        //   abs((long)(node1->lastAcoustic - node2->lastAcoustic))
        // Subtracting two unsigned longs and casting to signed is
        // implementation-defined once the values straddle a millis()
        // wrap at 49.7 days, and abs() on the result can then return a
        // huge positive number. Comparing the larger to the smaller is
        // wrap-safe for any interval shorter than the wrap period.
        unsigned long a = node1->lastAcoustic;
        unsigned long b = node2->lastAcoustic;
        unsigned long delta = (a > b) ? (a - b) : (b - a);

        sirenValid = node1->acousticDetected && node2->acousticDetected &&
                     delta < ACOUSTIC_WINDOW_MS;
        loraValid  = node1->loraDetected || node2->loraDetected;
    } else {
        // SINGLE-NODE BENCH MODE: this one node's own acoustic + LoRa
        // detections must agree with each other.
        sirenValid = node1->acousticDetected;
        loraValid  = node1->loraDetected;
    }

    if (sirenValid && loraValid) {
        // CHANGED v3.5: mandatory cross-traffic recovery between consecutive
        // holds on the same approach. See LANE_COOLDOWN_MS.
        //
        // NOTE THE EARLY RETURN IS BEFORE THE FLAG CLEAR, DELIBERATELY.
        // Returning here leaves acousticDetected/loraDetected set, so the
        // very next processValidation() pass re-tests and the lane queues
        // the instant the cooldown expires. Clearing them here instead
        // would DISCARD a live detection to enforce a fairness rule, which
        // is the wrong trade in a system whose worst error is a missed
        // ambulance.
        if (lastHoldEnd[lane] != 0 &&
            (millis() - lastHoldEnd[lane]) < LANE_COOLDOWN_MS) {
            return;
        }

        if (!laneAlreadyQueued(lane)) {
            // CHANGED v3.6: carry the priority class into the queue.
            //
            // Take the MOST URGENT class reported by either node on this
            // approach. They are reporting the same vehicle, so they should
            // agree; if they don't, one of them missed a certificate or is
            // reading a stale frame, and erring toward the more urgent
            // reading is the correct bias for an emergency system.
            uint8_t lanePriority = node1->priority;
            if (requireDualNodeConfirmation && node2 != nullptr) {
                if (priorityRank(node2->priority) < priorityRank(lanePriority))
                    lanePriority = node2->priority;
            }

            enqueueEmergency(lane, lanePriority);
            Serial.printf("[EMERGENCY VALID] Lane %d PRI=%u (mode=%s)\n",
                          lane, (unsigned)lanePriority,
                          requireDualNodeConfirmation ? "DUAL-NODE" : "SINGLE-NODE-BENCH");
        }
        // CHANGED v3: removed validationConsumed. It was written in three
        // places and read in none -- dead state that looked meaningful.
        node1->acousticDetected = false;
        node1->loraDetected = false;
        if (requireDualNodeConfirmation && node2 != nullptr) {
            node2->acousticDetected = false;
            node2->loraDetected = false;
        }
    }
}

void processValidation() {
    validateLane(1);
    validateLane(2);
    validateLane(3);
}

// =====================================================
// EMERGENCY QUEUE PROCESSOR (no relays -- prints what WOULD happen)
// =====================================================
void processEmergencyQueue() {
    if (emergencyRunning || queueIsEmpty()) return;

    uint8_t pri = 0;
    unsigned long waited = 0;
    int lane = dequeueEmergency(&pri, &waited);
    if (lane == 0) return;

    activeEmergencyLane = lane;
    emergencyRunning = true;
    emergencyStartTime = millis();

    // Log the arbitration decision, not just its outcome. `starved` marks a
    // lane promoted by the starvation bound rather than won on priority --
    // if that fires often, the priority mix or the hold length is wrong.
    bool starved = (waited >= QUEUE_STARVATION_MS);
    Serial.printf("[QUEUE] START LANE %d PRI=%u waited=%lums%s depth=%d "
                  "(relay hardware disabled)\n",
                  activeEmergencyLane, (unsigned)pri, waited,
                  starved ? " *STARVATION-PROMOTED*" : "", queueCount);

    // EDIT 4a of 4  (PHASE 2)  -- tell the console.
    //
    // LS_COMMIT and LE_ACOUSTIC_CONFIRMED are hardcoded, and that is
    // honest rather than lazy: this build has no staging and no evidence
    // classification. validateLane() produces ONE binary outcome, so
    // there is nothing truthful to vary these with yet. Phase 5 supplies
    // the real stage and evidence class and this call site does not
    // change shape.
    //
    // ETA is -1, meaning "not computable". There is no ETA engine in
    // this build. Sending a plausible-looking number instead would be
    // strictly worse than sending none, because the console renders it
    // to an officer who has no way to know it was invented.
    // PHASE 5E: SUPERSEDED. The console is now driven exclusively by
    // gwUpdateDecision(), which has the stage, the evidence class and
    // the ETA. This call had all three hardcoded because none of them
    // existed when it was written.
    //
    // Removed rather than left in place: two independent writers to the
    // same console would race, and the loser would be whichever ran
    // last in the loop. That produces an alert whose stage flickers
    // between the real decision and a hardcoded COMMIT -- which reads
    // on screen as the system changing its mind, and is the worst
    // possible thing for an operator to be shown.
    //
    // The legacy queue still runs and still logs; it simply no longer
    // reaches the console.
    // ercAlert(...) -- see ICU_Decision.ino
}

void processEmergencyFSM() {
    if (!emergencyRunning) return;
    if (millis() - emergencyStartTime > EMERGENCY_HOLD) {
        Serial.printf("[QUEUE] COMPLETE LANE %d (cooldown %lums before it may hold again)\n",
                      activeEmergencyLane, (unsigned long)LANE_COOLDOWN_MS);

        // EDIT 4b of 4  (PHASE 2)  -- end the alert on the console.
        //
        // MUST be before activeEmergencyLane is cleared below, or the
        // lane number is already 0 and the release names an approach
        // that does not exist -- the same ordering bug the v3.5 comment
        // on lastHoldEnd[] describes two lines down.
        //
        // LR_MAX_DURATION is the truthful reason TODAY. This build ends
        // an event on a fixed EMERGENCY_HOLD timer and on nothing else:
        // it cannot observe the ambulance passing, the siren stopping,
        // or the vehicle turning away. Phase 5 adds those paths, each
        // with its own reason code, so that "the ambulance arrived" and
        // "we gave up waiting" stop being indistinguishable on the
        // console and in the logs.
        // PHASE 5E: SUPERSEDED, same reason as the matching ercAlert.
        // Release is owned by gwUpdateDecision(), which knows WHY the
        // event ended -- siren lost, expired, diverged -- rather than
        // reporting every ending as a timeout.
        // ercRelease(...) -- see ICU_Decision.ino

        // CHANGED v3.5: stamp before clearing activeEmergencyLane, or the
        // index is 0 and the cooldown is recorded against a lane that
        // doesn't exist.
        if (activeEmergencyLane >= 1 && activeEmergencyLane <= TOTAL_LANES) {
            lastHoldEnd[activeEmergencyLane] = millis();
        }
        emergencyRunning = false;
        activeEmergencyLane = 0;
    }
}

// =====================================================
// LORA INIT
// =====================================================
void initCentralLoRa() {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

    if (!LoRa.begin(CENTRAL_FREQ)) {
        Serial.println("[LORA] Init Failed");
        while (true) {
            Serial.println("[LORA] Init Failed - check wiring/module");
            delay(2000);
        }
    }

    LoRa.setSyncWord(ICU_LORA_SYNC_WORD);
    LoRa.setSpreadingFactor(ICU_LORA_SF);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(6);
    LoRa.setPreambleLength(12);
    LoRa.enableCrc();
    LoRa.receive();

    Serial.printf("[LORA] Central Ready - 434.5MHz SF%d BW125k CR4/6 Pre12 Sync 0x%02X\n",
                  ICU_LORA_SF, ICU_LORA_SYNC_WORD);
}

// =====================================================
// PACKET RECEIVE
//
// This is the function that changed most. Read the ordering: it is
// deliberate, cheapest-check-first, so that a flood of forged frames
// costs us comparisons rather than cryptographic operations.
// =====================================================
void pollLoRa() {
    int packetSize = LoRa.parsePacket();

    if (!packetSize) {
        // 5E: sample the band while NOTHING is being received.
        //
        // This is the only discriminator the ICU has between a
        // node that has failed and a node that is being jammed --
        // both simply stop arriving, but one is a maintenance
        // ticket and the other is an attack, and node loss unlocks
        // degraded authority at the survivor.
        //
        // Rate limited: LoRa.rssi() is an SPI transaction and this
        // function runs every loop pass. Once a second is ample for
        // a slowly-filtered band estimate.
        static unsigned long lastNoiseSample = 0;
        if (millis() - lastNoiseSample >= 1000) {
            lastNoiseSample = millis();
            gwNoteRfNoise(LoRa.rssi());
        }
        return;
    }

    int if2rssi = LoRa.packetRssi();
    float if2snr = LoRa.packetSnr();

    if (packetSize > GW_MAX_FRAME_LEN) {
        Serial.printf("[DROP] oversize frame %d\n", packetSize);
        while (LoRa.available()) LoRa.read();
        return;
    }

    int len = 0;
    while (LoRa.available() && len < GW_MAX_FRAME_LEN) rxFrame[len++] = (uint8_t)LoRa.read();

    // ---- 1. minimum length for a header -----------------------------
    if (len < (int)sizeof(FrameHeader)) { rejBadLength++; return; }

    FrameHeader hdr;
    memcpy(&hdr, rxFrame, sizeof(FrameHeader));

    // ---- 2. protocol version ----------------------------------------
    // A specific, loud error. The v2 failure mode was a generic
    // "[UNKNOWN PACKET] size=119" that told you nothing about the cause.
    if (hdr.proto_version != GW_PROTO_VERSION) {
        rejWrongVersion++;
        Serial.printf("[DROP] proto v%u, expected v%d -- the two copies of "
                      "GreenwaveTypes.h are out of sync, reflash both ends\n",
                      (unsigned)hdr.proto_version, GW_PROTO_VERSION);
        return;
    }

    // ---- 3. is this even our intersection? --------------------------
    // Never checked in v2. This is what stops intersection #2 from
    // driving intersection #1.
    if (hdr.intersection_id != MY_INTERSECTION_ID) {
        rejWrongIntersection++;
        return;   // silent: a neighbour's traffic is normal, not an error
    }

    // ---- 4. length must match the declared type ---------------------
    size_t expected;
    switch (hdr.packet_type) {
        case HEARTBEAT_PACKET:      expected = sizeof(HeartbeatFrame); break;
        case LORA_EVENT_PACKET:     expected = sizeof(LoRaEventFrame); break;
        case ACOUSTIC_EVENT_PACKET: expected = sizeof(AcousticEventFrame); break;
        default:
            rejBadLength++;
            Serial.printf("[DROP] unknown packet_type %u\n", (unsigned)hdr.packet_type);
            return;
    }
    if ((size_t)len != expected) {
        rejBadLength++;
        Serial.printf("[DROP] type %u should be %u bytes, got %d\n",
                      (unsigned)hdr.packet_type, (unsigned)expected, len);
        return;
    }

    // ---- 5. is this a node we know? ---------------------------------
    NodeState* n = getNodeState(hdr.lane_id, hdr.node_id);
    if (n == nullptr) {
        rejUnknownNode++;
        Serial.printf("[DROP] unknown node L%uN%u\n",
                      (unsigned)hdr.lane_id, (unsigned)hdr.node_id);
        return;
    }

    // ---- 6. replay check BEFORE the MAC -----------------------------
    // Ordering matters. The counter check is one integer comparison; the
    // CMAC is a cryptographic operation. Under a flood of replayed
    // frames we want to be spending comparisons, not AES rounds.
    // CHANGED v3.1: separated benign duplicates from actual replays.
    //
    // A frame with the SAME counter is one of the node's own deliberate
    // repeats. A frame with a LOWER counter is something else entirely --
    // either an attacker replaying a recording, or a node whose epoch
    // counter failed to persist across a reboot. Averaging those together
    // meant a real attack would vanish into thousands of routine
    // duplicates: TC-LN-001 logged replay=2520, all of it benign.
    if (n->counterStarted) {
        if (hdr.counter == n->lastCounter) {
            dupSuppressed++;
            return;                       // expected, silent
        }
        if (hdr.counter < n->lastCounter) {
            n->rejectedReplay++; rejReplay++;
            // v3.2: this should now be genuinely rare. Before v3.2 the
            // lane node assigned counters at frame-build time while
            // transmitting in a different order, so out-of-order arrival
            // was routine and REAL DETECTIONS were discarded here. The
            // counter is now stamped at transmit time. If this fires
            // again, it is either an actual replay or a node whose epoch
            // failed to persist across a reboot -- both worth chasing.
            Serial.printf("[SECURITY] REPLAY from L%uN%u type=%u ctr=%lu < last=%lu (gap=%ld)\n",
                          (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                          (unsigned)hdr.packet_type,
                          (unsigned long)hdr.counter,
                          (unsigned long)n->lastCounter,
                          (long)(n->lastCounter - hdr.counter));
            return;
        }
    }

    // ---- 7. authenticate --------------------------------------------
    //
    // CHANGED v4 (SOP 3.3/3.5/5.3). Was a static-key CMAC check. Now:
    // derive (or look up) the session key for this node and epoch, then
    // verify a truncated HMAC-SHA256 tag against it.
    //
    // gwVerifyFrameEx returns a SPECIFIC failure reason so each rejection
    // path is counted separately. A revoked node and a forged tag are very
    // different events and must not share a counter.
    //
    // Ordering note: this still runs AFTER the counter check above, so a
    // replay flood costs one integer comparison rather than an HMAC.
    GwVerifyResult vr = gwVerifyFrameEx(hdr.lane_id, hdr.node_id,
                                        hdr.session_epoch, rxFrame, len);
    if (vr != GW_OK) {
        switch (vr) {
            case GW_ERR_REVOKED:
                rejRevoked++;
                Serial.printf("[SECURITY] REVOKED node L%uN%u ctr=%lu epoch=%lu "
                              "-- session refused at establishment (SOP 3.5)\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                              (unsigned long)hdr.counter,
                              (unsigned long)hdr.session_epoch);
                break;
            case GW_ERR_EPOCH:
                rejEpoch++;
                Serial.printf("[SECURITY] STALE EPOCH from L%uN%u epoch=%lu "
                              "-- older than the overlap window\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                              (unsigned long)hdr.session_epoch);
                break;
            case GW_ERR_NO_SESSION:
                rejNoSession++;
                Serial.printf("[SECURITY] NO SESSION for L%uN%u epoch=%lu "
                              "-- ECDH/HKDF derivation failed\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                              (unsigned long)hdr.session_epoch);
                break;
            case GW_ERR_UNKNOWN_NODE:
                rejUnknownNodeK++;
                Serial.printf("[SECURITY] NO PUBLIC KEY provisioned for L%uN%u\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id);
                break;
            case GW_ERR_LENGTH:
                rejBadLength++;
                Serial.printf("[SECURITY] LENGTH REJECT from L%uN%u len=%d\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id, len);
                break;
            default:   // GW_ERR_TAG
                n->rejectedMac++; rejMac++;
                Serial.printf("[SECURITY] BAD TAG from L%uN%u ctr=%lu epoch=%lu "
                              "(total=%lu) -- forged frame or key mismatch\n",
                              (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                              (unsigned long)hdr.counter,
                              (unsigned long)hdr.session_epoch,
                              (unsigned long)n->rejectedMac);
                break;
        }
        return;
    }

    // ---- 8. accepted -------------------------------------------------
    // v3.7: count the hole this frame's counter reveals, BEFORE advancing.
    // Guarded on counterStarted so the first frame after an ICU reboot does
    // not report the node's entire boot history as loss.
    if (n->counterStarted && hdr.counter > n->lastCounter + 1) {
        n->framesLostIf2 += (hdr.counter - n->lastCounter - 1);
    }
    n->framesAccepted++;

    n->lastCounter = hdr.counter;
    n->counterStarted = true;

    // ==================================================================
    // CHANGED v3.7: [RX2] -- one machine-parseable line per accepted frame,
    // mirroring [RX1] on the lane node.
    //
    // WHY: measuring IF-2 delivery by counting [LORA RX] lines against the
    // lane node's [EVENT RADIATED] lines gave 73.2%. Matching by SEQUENCE
    // NUMBER gave 93.9%. The 20-point error was this ICU's own serial
    // capture dropping long log lines -- the same failure that made the
    // IF-1 PRR read 131.8% before [RX1] existed. Line counts are not a
    // measurement; sequence diffs are.
    //
    // `ctr` is the per-node monotonic counter and is the authoritative key
    // for IF-2 loss: it increments on EVERY frame this node sends, of every
    // type, so a gap in ctr across an [RX2] series is IF-2 loss with no
    // inference required. `seq` is the EVU's own sequence, present only on
    // event frames, and is what correlates an ICU record back to [RX1] on
    // the lane node and to the EVU log. Both are needed: one measures this
    // link, the other stitches the three logs together.
    //
    //   IF-2 loss:      grep -o 'ctr=[0-9]*' ICULOG | cut -d= -f2 | sort -n
    //   end-to-end:     compare seq= across RX2 / RX1 / EVU
    //
    // Emitted for EVERY accepted frame including heartbeats, because a
    // heartbeat is exactly the frame whose loss the [HEALTH] line cannot
    // distinguish from a node that stopped sending.
    // ==================================================================
    switch (hdr.packet_type) {
        case HEARTBEAT_PACKET: {
            HeartbeatFrame f; memcpy(&f, rxFrame, sizeof(f));
            // v4.5: sepoch added. Without it the entire SOP 3.4 re-key
            // mechanism and the SOP 5.3 overlap window are invisible -- they
            // could rotate, fail, or never run and no log would differ.
            Serial.printf("[RX2] type=hb  lane=%u node=%u ctr=%lu sepoch=%lu seq=- "
                          "rssi=%d snr=%.2f up=%lu heap=%u\n",
                          (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                          (unsigned long)hdr.counter,
                          (unsigned long)hdr.session_epoch, if2rssi, if2snr,
                          (unsigned long)f.uptime_s, (unsigned)f.free_heap_kb);
            processHeartbeat(f, n);
            break;
        }
        case LORA_EVENT_PACKET: {
            LoRaEventFrame f; memcpy(&f, rxFrame, sizeof(f));
            Serial.printf("[RX2] type=evt lane=%u node=%u ctr=%lu sepoch=%lu seq=%lu "
                          "rssi=%d snr=%.2f pri=%u sig=%u flags=0x%02X "
                          "if1rssi=%d if1snr=%.1f epoch=%lu\n",
                          (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                          (unsigned long)hdr.counter,
                          (unsigned long)hdr.session_epoch, (unsigned long)f.tx_seq,
                          if2rssi, if2snr, (unsigned)f.priority,
                          (unsigned)f.sig_status, (unsigned)f.flags,
                          f.rssi_if1, f.snr_if1_x10 / 10.0f,
                          (unsigned long)f.gps_epoch);
            processLoRaEvent(f, n, if2rssi, if2snr);
            break;
        }
        case ACOUSTIC_EVENT_PACKET: {
            AcousticEventFrame f; memcpy(&f, rxFrame, sizeof(f));
            Serial.printf("[RX2] type=aco lane=%u node=%u ctr=%lu sepoch=%lu seq=- "
                          "rssi=%d snr=%.2f conf=%u flags=0x%02X sustained=%d\n",
                          (unsigned)hdr.lane_id, (unsigned)hdr.node_id,
                          (unsigned long)hdr.counter,
                          (unsigned long)hdr.session_epoch, if2rssi, if2snr,
                          (unsigned)f.confidence, (unsigned)f.flags,
                          (f.flags & ACF_SUSTAINED) ? 1 : 0);
            processAcousticEvent(f, n);
            break;
        }
    }
}

// =====================================================
// PERIODIC HEALTH SUMMARY
// Rejection counters are the only externally visible sign that someone
// is attacking IF-2. Printing them periodically means an attack shows up
// in the serial log instead of being invisible.
// =====================================================
void printRejectSummary() {
    static unsigned long last = 0;
    if (millis() - last < 30000) return;
    last = millis();
    Serial.printf("[HEALTH] rejects: ver=%lu xsect=%lu len=%lu node=%lu replay=%lu mac=%lu | dup=%lu | mode=%s\n",
                  (unsigned long)rejWrongVersion, (unsigned long)rejWrongIntersection,
                  (unsigned long)rejBadLength, (unsigned long)rejUnknownNode,
                  (unsigned long)rejReplay, (unsigned long)rejMac,
                  (unsigned long)dupSuppressed,
                  requireDualNodeConfirmation ? "DUAL-NODE" : "SINGLE-NODE-BENCH");

    // v4 session-layer rejections, on their own line so the existing
    // machine-parseable [HEALTH] line keeps its exact field set.
    // v4.5: overlap= added. A nonzero value proves the SOP 5.3 cutover
    // window is being exercised; epoch= staying 0 alongside it proves no
    // frame was lost to a rotation. Both were previously unobservable.
    Serial.printf("[HEALTH] session: revoked=%lu nosession=%lu epoch=%lu nokey=%lu "
                  "overlap=%lu | keys=%s | revmask=0x%02X\n",
                  (unsigned long)rejRevoked, (unsigned long)rejNoSession,
                  (unsigned long)rejEpoch, (unsigned long)rejUnknownNodeK,
                  (unsigned long)gwOverlapHits, GW_KEY_SET_ID,
                  (unsigned)gwRevokedMask);

    // PHASE 2 (optional edit): console link health.
    //
    // On its own line for the same reason as the session line above --
    // the [HEALTH] rejects line is machine-parseable and its field set
    // must not drift.
    //
    // Watch the RETURN percentage specifically. The ICU knows its own
    // transmissions left the pin; only PONG proves the reverse path
    // works, and the reverse path is the one carrying the operator's
    // ACK. A link that transmits perfectly and receives nothing looks
    // entirely healthy from the outbound side while silently discarding
    // the only evidence that a human ever saw an alert.
    ercPrintStats();

    // PHASE 5A: full geometry and health picture.
    gwPrintGeometry();
    gwPrintTracks();
    gwPrintEvuTracks();
    gwPrintDemand();
    gwPrintSafety();

    // CHANGED v3.6: liveness and detection state are now independent (see
    // markNodeOffline), so print both. A node showing det=L-- with hb=0 is
    // the half-dead case the v3 heartbeat split exists to expose, and it is
    // no longer inferable from any other line in the log.
    struct { int lane; int node; NodeState* ptr; } nodes[] = {
        {1,1,&lane1node1}, {1,2,&lane1node2},
        {2,1,&lane2node1}, {2,2,&lane2node2},
        {3,1,&lane3node1}, {3,2,&lane3node2}
    };
    Serial.printf("[HEALTH] queue depth=%d active=%d |", queueCount, activeEmergencyLane);
    for (auto &e : nodes) {
        if (e.ptr->lastSeen == 0) continue;      // never heard from: skip

        // v3.7: IF-2 delivery per node. Denominator is accepted + lost,
        // i.e. frames the node demonstrably sent, so this is link delivery
        // and not a function of how long the ICU has been listening.
        uint32_t sent = e.ptr->framesAccepted + e.ptr->framesLostIf2;
        float pdr = sent ? (100.0f * e.ptr->framesAccepted / (float)sent) : 0.0f;

        Serial.printf(" L%dN%d:hb=%d,det=%c%c,if2=%.1f%%(%lu/%lu)",
                      e.lane, e.node, e.ptr->online ? 1 : 0,
                      e.ptr->loraDetected     ? 'L' : '-',
                      e.ptr->acousticDetected ? 'A' : '-',
                      pdr, (unsigned long)e.ptr->framesAccepted,
                      (unsigned long)sent);
    }
    Serial.println();
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("=================================");
    Serial.println("GREENWAVE CENTRAL NODE - MINIMAL COMMS BUILD");
    Serial.printf ("protocol v%d  intersection=%d\n", GW_PROTO_VERSION, MY_INTERSECTION_ID);
    Serial.println("(no relays, no battery ADC, no WiFi, no MQTT)");
    if (!requireDualNodeConfirmation)
        Serial.println("*** SINGLE-NODE BENCH MODE - NOT FOR DEPLOYMENT ***");
    // CHANGED v3.5: print the decision timers. EVENT_TIMEOUT_MS is paired
    // with ACOUSTIC_REASSERT_MS in LaneNode.ino and there is no way for the
    // compiler to check that pairing across two sketches. Printing it means
    // a mismatched flash is visible in the first ten lines of the log
    // instead of showing up as intermittent missed preemptions.
    Serial.printf("Timers: acousticWin=%dms eventTimeout=%dms hold=%dms "
                  "cooldown=%dms hbTimeout=%dms starvation=%dms\n",
                  ACOUSTIC_WINDOW_MS, EVENT_TIMEOUT_MS, EMERGENCY_HOLD,
                  LANE_COOLDOWN_MS, HEARTBEAT_TIMEOUT_MS, QUEUE_STARVATION_MS);
    Serial.printf("  expects LaneNode: ACOUSTIC_REASSERT_MS=%lu HEARTBEAT_MAX_DEFER_MS=%lu\n",
                  EXPECTED_ACOUSTIC_REASSERT_MS, EXPECTED_LANE_NODE_DEFER_MS);
    Serial.println("  arbitration: priority-first, starvation-bounded, no preempt-of-preempt");
    Serial.printf("Frames: HB=%uB ACO=%uB EVT=%uB\n",
                  (unsigned)sizeof(HeartbeatFrame),
                  (unsigned)sizeof(AcousticEventFrame),
                  (unsigned)sizeof(LoRaEventFrame));
    Serial.println("=================================");

    // EDIT 2 of 4  (PHASE 2)
    //
    // DELIBERATELY BEFORE initCentralLoRa(). That function contains a
    // `while (true)` spin if the radio fails to start, so a board with a
    // dead or unwired LoRa module never reaches any line placed after
    // it. Bringing the console up first means that failure appears on
    // the ERC screen instead of presenting as a board that boots to
    // nothing at all -- which is indistinguishable from a dead board.
    //
    // ercInit() also sends CLEAR. Spec section 11.1: on reboot the ICU
    // starts in NORMAL with all state cleared, and preemption state is
    // NEVER restored from storage. The console may be sitting in a stale
    // alert from before the reset and has no way to know we restarted,
    // so telling it is the first thing we do.
    // Load the junction's surveyed geometry FIRST. Everything downstream
    // -- approach association, distance, ETA -- measures from it, so it
    // must be settled before the first frame can arrive.
    gwSiteLoad();
    gwSitePrint();

    ercInit();

    // 5F: armed AFTER ercInit() but the feed happens in loop(), so
    // anything that blocks during the rest of setup() is unaffected.
    // Arming earlier would make a slow LoRa init look like a hang.
    gwWatchdogInit();

    // v4: must precede initCentralLoRa() so no frame can arrive before the
    // session cache exists.
    gwIcuSessions.setIntersection(MY_INTERSECTION_ID);
    gwIcuSessions.begin();

    initCentralLoRa();
    Serial.println("READY");
}

void loop() {
    // 5F: fed FIRST, so a reboot means loop() genuinely stopped
    // iterating -- not that one slow call inside it overran.
    gwWatchdogFeed();

    pollLoRa();

    // EDIT 3 of 4  (PHASE 2)
    // Sends PING, reads replies, tracks link health. Rate-limits itself
    // internally, so calling it every pass is correct and cheap.
    ercService();

    // PHASE 5A: node health FSM + FAR/NEAR derivation. Rate-limits
    // itself; safe to call every pass.
    gwUpdateNodeHealth();

    // PHASE 5B: track expiry, partner-wait timeout, siren-loss release.
    gwUpdateTracks();

    // SCENARIO 2: EVU track grace, expiry and divergence bookkeeping.
    // Must run BEFORE gwUpdateDecision(), which reads the result.
    gwUpdateEvuTracks();

    // PHASE 5C/5D/5E: reduce tracks to a demand tuple, arbitrate across
    // approaches, and drive the console. MUST run after gwUpdateTracks()
    // and gwUpdateEvuTracks() -- it reads the state those calls settled.
    gwUpdateDecision();

    // PHASE 3: bench console on the USB serial port. Type `help` in the
    // serial monitor. Lets the ERC screens be driven and tested with no
    // RDUs present, which is the only way to exercise them before Phase 4
    // exists -- and the only practical way to produce failure states like
    // AMBIGUOUS or MISCONFIGURED on demand without breaking hardware.
    ercBenchConsole();

    checkHeartbeats();
    checkEventTimeouts();
    processValidation();
    processEmergencyQueue();
    processEmergencyFSM();
    printRejectSummary();
}
