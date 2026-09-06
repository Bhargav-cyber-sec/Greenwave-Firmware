/***********************************************************************
 * ICU_Geometry.ino  --  PHASE 5A : GEOMETRY AND NODE HEALTH
 *
 * A new tab in the ICU sketch folder. The Arduino IDE compiles every
 * .ino in a folder into one program.
 *
 * This file answers two questions the ICU has never been able to answer:
 *
 *   1. Which of an approach's two nodes is closer to the stop line?
 *   2. Is a node that has gone quiet broken, or just briefly unlucky?
 *
 * Both are prerequisites for every decision in 5B onwards. Direction of
 * travel is "which node heard it first", which is meaningless without
 * (1). And degraded-mode authority hinges entirely on (2).
 *
 * NOTHING HERE DECIDES ANYTHING ABOUT TRAFFIC. It establishes what the
 * ICU knows about its own sensors. Tracks, evidence and staging are 5B.
 ***********************************************************************/

#include "GreenwaveLink.h"

// Defined in ICU_Link.ino. Declared explicitly because the Arduino IDE
// concatenates .ino tabs alphabetically -- ICU_Geometry comes before
// ICU_Link, so the auto-generated prototype is not guaranteed to be
// visible here.
void ercHealth(int lane, int node, uint8_t role, uint8_t health, uint16_t distM);

// From ICU_Safety.ino.
bool gwRfBandNoisy();
void gwNoteLinkSuspect(int lane, int node);

// Defined in ICU.ino, above this tab in the concatenation order.
extern NodeState lane1node1, lane1node2;
extern NodeState lane2node1, lane2node2;
extern NodeState lane3node1, lane3node2;

// =====================================================
// HEALTH TIMING
// =====================================================
//
// Derived from the RDU's actual transmit schedule, not chosen:
//
//   HEARTBEAT_BASE_MS      5000
//   HEARTBEAT_JITTER_MS    0..1500
//   HEARTBEAT_MAX_DEFER_MS 6000   (channel busy -> heartbeat waits)
//
// Worst-case spacing between two consecutive heartbeats is therefore
// about 12.9 s even when the node is perfectly healthy. Any threshold
// below that would fire on a working node.

// Two missed heartbeats at worst-case spacing, plus margin.
//
// SUSPECT is the state that exists purely to STOP the system reacting
// to ordinary packet loss. IF-2 delivery measures ~96%, so a missed
// heartbeat is routine, not exceptional.
//
// Under the old two-state model (online/offline) a single missed
// heartbeat flipped a node to offline -- and offline unlocks degraded
// mode, and degraded mode LOWERS the evidence needed to act. So one
// dropped packet quietly relaxed the system's own safety bar, several
// times an hour, silently.
//
// A SUSPECT node keeps participating fully. What it does NOT do is
// promote its partner to degraded authority.
// WIDENED 26s -> 40s when the heartbeat interval doubled.
//
// This is derived from the RDU's actual schedule, not chosen:
//
//     HEARTBEAT_BASE_MS      10000   (was 5000)
//     HEARTBEAT_JITTER_MS     1500
//     HEARTBEAT_MAX_DEFER_MS  6000   (channel busy -> heartbeat waits)
//
// Worst-case spacing between two heartbeats from a perfectly healthy
// node is therefore 17.5 s. At the old 26 s this was only 1.5 heartbeat
// intervals, so ONE lost heartbeat would have demoted a healthy node to
// SUSPECT.
//
// That would have quietly undone the whole reason SUSPECT exists. It is
// there to stop ordinary packet loss disturbing the system, and at 1.5
// intervals it would have become the thing causing the disturbance --
// with IF-2 delivery around 96%, several times an hour.
//
// 40 s is 2.3 worst-case intervals. A healthy node survives one lost
// heartbeat; a node that has genuinely stopped is still caught within
// two cycles.
//
// THE RELATIONSHIP IS WHAT MATTERS, NOT THE NUMBER. If the heartbeat
// interval changes again, recompute: this must stay above 2x
// (BASE + JITTER + MAX_DEFER).
#define NODE_SUSPECT_MS   40000UL

// Sustained silence. Matches the existing HEARTBEAT_TIMEOUT_MS so the
// two notions of "gone" cannot drift apart.
// Raised with NODE_SUSPECT_MS above, to keep a clear gap between them.
//
// If the two were equal a node would jump straight from HEALTHY to
// FAILED, skipping SUSPECT entirely -- and FAILED is what unlocks
// degraded authority at the partner. The intermediate state existing
// but being unreachable would be worse than not having it.
#define NODE_FAILED_MS    60000UL

// Consecutive good heartbeats required to leave RECOVERING.
//
// A node that flaps between working and failed would otherwise
// oscillate its partner between normal and degraded authority, and
// degraded authority is exactly when the evidence bar is lowest. Making
// recovery deliberately slow means an intermittent node cannot repeatedly
// hand its partner extra power.
#define NODE_RECOVER_COUNT  3

// =====================================================
// PER-NODE GEOMETRY AND HEALTH
// =====================================================

struct NodeGeometry {
    // ---- reported by the node ----
    bool     reported          = false;
    uint8_t  approachId        = 0;
    uint16_t distanceM         = GW_DISTANCE_UNSET;
    uint32_t configHash        = 0;
    uint8_t  micNoiseFloor     = 0;
    uint16_t loopLiveness      = 0;
    uint8_t  flags             = 0;

    // ---- derived by the ICU ----
    uint8_t  role              = ROLE_UNKNOWN;   // NEVER from node_id
    uint8_t  health            = NH_UNKNOWN;
    uint8_t  lastPushedHealth  = 0xFF;           // for change-driven ERC push
    uint8_t  lastPushedRole    = 0xFF;

    // ---- liveness bookkeeping ----
    uint16_t prevLiveness      = 0;
    bool     livenessSeen      = false;
    unsigned long lastLivenessChange = 0;
    uint8_t  recoverStreak     = 0;

    // Set when the node's own report is internally inconsistent. Sticky
    // for the life of the session: a node that has once contradicted
    // itself is not trusted again until it is power-cycled and
    // re-examined by a human.
    bool     configFault       = false;
    const char *faultReason    = "";
};

// [lane][node], both 1-based. Index 0 unused, for readability at the
// call sites -- an off-by-one in this table would silently attribute one
// node's geometry to another.
static NodeGeometry nodeGeo[GW_NUM_APPROACHES + 1][GW_NODES_PER_APPROACH + 1];

// Per-approach summary.
struct ApproachGeometry {
    bool    usable      = false;   // geometry good enough to reason with
    uint8_t farNode     = 0;       // node_id of the FAR node, 0 = unknown
    uint8_t nearNode    = 0;
    uint16_t separationM = 0;      // derived, never configured
    const char *reason  = "no data";
};

static ApproachGeometry approachGeo[GW_NUM_APPROACHES + 1];

// =====================================================
// GEOMETRY: DERIVE FAR / NEAR AT RUNTIME
// =====================================================

// Recompute roles for one approach from the two nodes' reported
// distances.
//
// THIS IS THE FIX FOR SPEC DEFECT S1-01.
//
// v1.0 assigned position from the ordinal names "RDU 1" and "RDU 2",
// and it assigned them INVERTED. The consequence is silent and
// symmetric: the junction preempts for traffic LEAVING and ignores
// traffic ARRIVING. No error is raised, nothing in any log looks wrong,
// and the system simply behaves backwards forever.
//
// Sorting by a surveyed distance that each node reports about itself
// makes that failure impossible to express. Swap two enclosures and the
// ICU follows the swap, because the distance travels with the box.
static void gwRecomputeApproach(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    NodeGeometry &a = nodeGeo[lane][1];
    NodeGeometry &b = nodeGeo[lane][2];
    ApproachGeometry &ap = approachGeo[lane];

    ap.usable      = false;
    ap.farNode     = 0;
    ap.nearNode    = 0;
    ap.separationM = 0;

    if (!a.reported || !b.reported) {
        ap.reason = "waiting for both nodes";
        // Roles are cleared, not guessed. A single reporting node cannot
        // establish which end of the approach it is on -- it has nothing
        // to be compared against.
        a.role = ROLE_UNKNOWN;
        b.role = ROLE_UNKNOWN;
        return;
    }

    bool aValid = (a.flags & HBF_DISTANCE_VALID) &&
                  a.distanceM != GW_DISTANCE_UNSET;
    bool bValid = (b.flags & HBF_DISTANCE_VALID) &&
                  b.distanceM != GW_DISTANCE_UNSET;

    if (!aValid || !bValid) {
        ap.reason = "distance not surveyed";
        a.role = ROLE_UNKNOWN;
        b.role = ROLE_UNKNOWN;
        return;
    }

    // EQUAL DISTANCES ARE A HARD FAULT, NOT A TIE TO BREAK.
    //
    // There is no correct answer here and no safe default. Falling back
    // to node_id would reintroduce exactly the defect this function
    // exists to eliminate, and it would do so invisibly -- the system
    // would look like it was sorting by distance while actually sorting
    // by install order.
    //
    // Two nodes reporting the same distance means one was provisioned
    // wrong. That is a maintenance ticket, and the approach does nothing
    // until it is resolved.
    if (a.distanceM == b.distanceM) {
        ap.reason = "IDENTICAL DISTANCES - cannot order nodes";
        a.role = ROLE_UNKNOWN;
        b.role = ROLE_UNKNOWN;
        a.configFault = true;  a.faultReason = "duplicate distance";
        b.configFault = true;  b.faultReason = "duplicate distance";
        return;
    }

    // The sort itself. Everything above is refusing to do it on bad data.
    if (a.distanceM > b.distanceM) {
        a.role = ROLE_FAR;   b.role = ROLE_NEAR;
        ap.farNode = 1;      ap.nearNode = 2;
        ap.separationM = a.distanceM - b.distanceM;
    } else {
        b.role = ROLE_FAR;   a.role = ROLE_NEAR;
        ap.farNode = 2;      ap.nearNode = 1;
        ap.separationM = b.distanceM - a.distanceM;
    }

    // Separation bounds the correlation window in 5B: the time a vehicle
    // takes to travel between the two nodes is what direction inference
    // is measured against. A separation of a few metres cannot produce a
    // measurable delta over a shared radio channel, and a huge one means
    // the two nodes are not observing the same approach event.
    //
    // [FIELD] These bounds want confirming against real site layouts.
    if (ap.separationM < 20) {
        ap.reason = "nodes too close to infer direction";
        return;
    }
    if (ap.separationM > 1000) {
        ap.reason = "nodes too far apart to be one approach";
        return;
    }

    ap.usable = true;
    ap.reason = "ok";
}

// =====================================================
// INGEST A HEARTBEAT
// =====================================================

void gwGeometryOnHeartbeat(const HeartbeatFrame &f) {
    int lane = f.hdr.lane_id;
    int node = f.hdr.node_id;

    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    if (node < 1 || node > GW_NODES_PER_APPROACH) return;

    NodeGeometry &g = nodeGeo[lane][node];

    g.reported      = true;
    g.approachId    = f.approach_id;
    g.distanceM     = f.distance_to_stopline_m;
    g.configHash    = f.config_hash;
    g.micNoiseFloor = f.mic_noise_floor;
    g.flags         = f.flags;

    // ---- inference-path liveness ----
    //
    // Compared for INEQUALITY only, never magnitude, so the counter
    // wrapping at 65535 needs no special handling.
    //
    // A node whose transport task is healthy and whose inference task
    // has hung heartbeats perfectly while being completely deaf. This
    // counter is the only thing that distinguishes it from a node on a
    // quiet road.
    if (g.livenessSeen && f.loop_liveness != g.prevLiveness) {
        g.lastLivenessChange = millis();
    } else if (!g.livenessSeen) {
        g.lastLivenessChange = millis();
    }
    g.prevLiveness  = f.loop_liveness;
    g.loopLiveness  = f.loop_liveness;
    g.livenessSeen  = true;

    // ---- config consistency ----
    //
    // A node that says it watches a different approach than the one its
    // frame header claims is internally inconsistent. One of the two is
    // wrong and there is no way to tell which, so neither is believed.
    //
    // This is the check that catches a node flashed with a neighbouring
    // node's configuration -- the case where every individual field is
    // plausible, no range check fires, and the node reports confidently
    // and accurately about the wrong piece of road.
    if (f.approach_id != 0 && f.approach_id != lane) {
        g.configFault = true;
        g.faultReason = "approach_id != lane_id";
    }

    if ((f.flags & HBF_DISTANCE_VALID) &&
        (f.distance_to_stopline_m < GW_DISTANCE_MIN_M ||
         f.distance_to_stopline_m > GW_DISTANCE_MAX_M)) {
        g.configFault = true;
        g.faultReason = "distance out of plausible range";
    }

    gwRecomputeApproach(lane);
}

// =====================================================
// HEALTH STATE MACHINE
// =====================================================
//
// Spec Scenario 1 v2.0 section 5.2. The state that earns this machine
// its existence is SUSPECT -- see NODE_SUSPECT_MS above for why a
// two-state model was actively unsafe.

static uint8_t gwEvaluateHealth(int lane, int node, unsigned long now,
                                unsigned long lastHb, bool everSeen) {
    NodeGeometry &g = nodeGeo[lane][node];

    if (!everSeen) return NH_UNKNOWN;

    // ---- MISCONFIGURED outranks everything ----
    //
    // Deliberately checked before liveness and before silence. A node
    // that is transmitting perfectly and is provisioned wrong is MORE
    // dangerous than one that is silent: the silent one is obviously
    // broken, and this one is confidently wrong.
    //
    // The ICU never infers geometry it was not given.
    if (g.configFault) return NH_MISCONFIGURED;

    if (g.reported && !(g.flags & HBF_DISTANCE_VALID)) return NH_MISCONFIGURED;

    unsigned long silence = now - lastHb;

    // ---- FAILED / LINK_SUSPECT ----
    if (silence > NODE_FAILED_MS) {
        g.recoverStreak = 0;

        // A node that goes silent while the BAND IS NOISY may be
        // jammed rather than broken. The two look identical from
        // here, and they call for opposite responses.
        //
        // It matters because FAILED unlocks degraded authority at
        // the partner, which LOWERS the evidence bar. Jamming one
        // node of a pair is far cheaper than defeating
        // authentication, so treating a jammed node as merely
        // failed would make the radio the soft target (spec S1-06).
        //
        // LINK_SUSPECT is used ONLY to withhold authority, never to
        // grant it. A false positive costs a missed degraded-mode
        // alert; a false negative hands an attacker exactly what
        // they were trying to obtain.
        if (gwRfBandNoisy()) {
            if (g.health != NH_LINK_SUSPECT) {
                gwNoteLinkSuspect(lane, node);
            }
            return NH_LINK_SUSPECT;
        }

        return NH_FAILED;
    }

    // ---- self-test faults ----
    //
    // Treated as SUSPECT rather than FAILED, on purpose. The node is
    // still delivering frames and its radio is fine; what is in doubt is
    // whether its microphone is hearing anything. SUSPECT keeps it
    // participating while denying its partner degraded authority --
    // which is the correct response to "this node may be deaf", because
    // promoting the partner on the strength of a possibly-deaf node's
    // silence is exactly backwards.
    if (g.reported && !(g.flags & HBF_MIC_SELFTEST_OK)) {
        return NH_SUSPECT;
    }

    // Inference loop stalled: heartbeats arriving, detector not running.
    if (g.livenessSeen &&
        (now - g.lastLivenessChange) > NODE_FAILED_MS) {
        return NH_SUSPECT;
    }

    // ---- RECOVERING ----
    if (g.health == NH_FAILED || g.health == NH_RECOVERING) {
        if (silence < NODE_SUSPECT_MS) {
            if (g.recoverStreak < 255) g.recoverStreak++;
            if (g.recoverStreak >= NODE_RECOVER_COUNT) return NH_HEALTHY;
            return NH_RECOVERING;
        }
        return NH_RECOVERING;
    }

    // ---- SUSPECT ----
    if (silence > NODE_SUSPECT_MS) return NH_SUSPECT;

    g.recoverStreak = 0;
    return NH_HEALTHY;
}

// Called periodically from loop().
static uint32_t gwHealthPushTotal = 0;

void gwUpdateNodeHealth() {
    unsigned long now = millis();
    uint8_t pushed = 0;

    struct { int lane; int node; NodeState *ptr; } list[] = {
        {1,1,&lane1node1},{1,2,&lane1node2},
        {2,1,&lane2node1},{2,2,&lane2node2},
        {3,1,&lane3node1},{3,2,&lane3node2}
    };

    for (unsigned i = 0; i < sizeof(list)/sizeof(list[0]); i++) {
        int lane = list[i].lane, node = list[i].node;
        if (lane > GW_NUM_APPROACHES) continue;

        NodeGeometry &g = nodeGeo[lane][node];
        NodeState    *n = list[i].ptr;

        bool everSeen = (n->lastHeartbeat != 0);
        uint8_t next  = gwEvaluateHealth(lane, node, now,
                                         n->lastHeartbeat, everSeen);

        if (next != g.health) {
            Serial.printf("[HEALTH] L%dN%d %s -> %s%s%s\n",
                          lane, node,
                          gwHealthName(g.health), gwHealthName(next),
                          g.configFault ? "  reason=" : "",
                          g.configFault ? g.faultReason : "");
            g.health = next;
            gwRecomputeApproach(lane);
        }

        // Push to the console on change only. Pushing every cycle would
        // flood a 9600 baud link that also has to carry alerts.
        if (g.health != g.lastPushedHealth || g.role != g.lastPushedRole) {
            g.lastPushedHealth = g.health;
            g.lastPushedRole   = g.role;
            ercHealth(lane, node, g.role, g.health,
                      (g.distanceM == GW_DISTANCE_UNSET) ? 0 : g.distanceM);
            pushed++;
            gwHealthPushTotal++;
        }
    }

    // Diagnostic for a specific observed oddity: across several bench
    // runs the boot burst printed five HEALTH lines instead of six, with
    // 2:1 consistently absent -- while [GEOMETRY] showed all six nodes
    // present and correctly tracked.
    //
    // A missing line here would be serious. HEALTH is how the console
    // learns that a node is failed or misconfigured, and a node the
    // operator is never told about is one whose degraded state is
    // invisible at exactly the moment it matters.
    //
    // So this counts what was actually handed to the transport rather
    // than what appeared in a serial capture. If the count says six and
    // only five lines are visible, the loss is in the ICU's USB serial
    // output under burst load -- cosmetic, and not on the path the
    // console depends on. If it says five, the push loop is genuinely
    // skipping a node and that is a real bug.
    if (pushed > 0) {
        Serial.printf("[ERC] pushed %u health line(s) this cycle (total=%lu)\n",
                      (unsigned)pushed, (unsigned long)gwHealthPushTotal);
    }
}

// =====================================================
// QUERIES FOR PHASE 5B
// =====================================================

// The node_id currently acting as FAR / NEAR on an approach, or 0.
// Callers must handle 0 -- it means the geometry is not established, and
// there is no safe fallback.
uint8_t gwFarNode(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;
    return approachGeo[lane].farNode;
}

uint8_t gwNearNode(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;
    return approachGeo[lane].nearNode;
}

uint16_t gwSeparationM(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;
    return approachGeo[lane].separationM;
}

// True only when both nodes are known, ordered, and plausibly spaced.
// 5B must refuse to infer direction when this is false.
bool gwApproachUsable(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return false;
    return approachGeo[lane].usable;
}

uint8_t gwNodeHealth(int lane, int node) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return NH_UNKNOWN;
    if (node < 1 || node > GW_NODES_PER_APPROACH) return NH_UNKNOWN;
    return nodeGeo[lane][node].health;
}

// Surveyed distance for one node, or 0 when not established.
//
// Returns 0 rather than GW_DISTANCE_UNSET so callers cannot accidentally
// arithmetic on the sentinel -- 65535 metres would silently become a
// plausible-looking figure in an ETA calculation.
uint16_t gwNodeDistanceM(int lane, int node) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;
    if (node < 1 || node > GW_NODES_PER_APPROACH) return 0;

    NodeGeometry &g = nodeGeo[lane][node];
    if (!(g.flags & HBF_DISTANCE_VALID)) return 0;
    if (g.distanceM == GW_DISTANCE_UNSET) return 0;

    return g.distanceM;
}

uint8_t gwNodeRole(int lane, int node) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return ROLE_UNKNOWN;
    if (node < 1 || node > GW_NODES_PER_APPROACH) return ROLE_UNKNOWN;
    return nodeGeo[lane][node].role;
}

// =====================================================
// DIAGNOSTICS
// =====================================================

void gwPrintGeometry() {
    Serial.println("[GEOMETRY]");

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        ApproachGeometry &ap = approachGeo[lane];

        Serial.printf("  approach %d (%s): %s  sep=%um  %s\n",
                      lane, gwErcApproachName(lane),
                      ap.usable ? "USABLE" : "NOT USABLE",
                      (unsigned)ap.separationM, ap.reason);

        for (int node = 1; node <= GW_NODES_PER_APPROACH; node++) {
            NodeGeometry &g = nodeGeo[lane][node];

            if (!g.reported) {
                Serial.printf("    N%d  -- no heartbeat --\n", node);
                continue;
            }

            char dist[16];
            if (g.distanceM == GW_DISTANCE_UNSET) strcpy(dist, "UNSET");
            else snprintf(dist, sizeof(dist), "%um", (unsigned)g.distanceM);

            Serial.printf("    N%d  %-7s %-12s dist=%-7s hash=0x%08lX "
                          "floor=%u live=%u %s%s\n",
                          node,
                          gwRoleName(g.role),
                          gwHealthName(g.health),
                          dist,
                          (unsigned long)g.configHash,
                          (unsigned)g.micNoiseFloor,
                          (unsigned)g.loopLiveness,
                          (g.flags & HBF_MIC_SELFTEST_OK) ? "mic:OK" : "mic:FAIL",
                          g.configFault ? "  *** CONFIG FAULT ***" : "");
        }
    }
}
