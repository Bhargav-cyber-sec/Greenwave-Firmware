/***********************************************************************
 * ICU_EVU.ino  --  SCENARIO 2 : AUTHENTICATED VEHICLE TRACKS
 *
 * A new tab in the ICU sketch folder.
 *
 * Scenario 1 reasons about a SOUND. This file reasons about a VEHICLE
 * that has cryptographically proved who it is.
 *
 * ---------------------------------------------------------------------
 * THE CENTRAL DISTINCTION, WHICH v1.0 MISSED ENTIRELY
 *
 *   AUTHENTICATION proves the packet came from a unit holding a valid
 *   key. It says nothing about whether the packet's CONTENTS are true.
 *
 * A stolen ambulance unit, a decommissioned one that was never revoked,
 * or one whose firmware has been read, all produce packets that verify
 * perfectly. Every signature checks out. The unit is simply lying about
 * its own importance, its own position, or both.
 *
 * v1.0 collapsed these two ideas. Having verified the signature it then
 * believed the payload, which means the security model stopped at the
 * enclosure door. Everything below re-establishes the separation.
 *
 * ---------------------------------------------------------------------
 * DEFECTS ADDRESSED
 *
 * S2-01  Self-declared priority. v1.0 took priority_class straight from
 *        the packet. EVU2.ino has `#define PRIORITY_CLASS 1` compiled
 *        in and transmits it, so anyone able to reflash a unit can
 *        outrank every genuine emergency in the network. Priority now
 *        comes from the ICU's registry; the packet's claim is logged as
 *        advisory and never acted upon.
 *
 * S2-05  Unchecked position. A genuinely signed packet can carry a
 *        fabricated or stale position. Consecutive fixes are now checked
 *        against the vehicle's own reported speed.
 *
 * S2-08  Discarded release signal. The emergency flag going from 1 to 0
 *        is an explicit, authenticated "I am done" from the only party
 *        who actually knows -- the driver. v1.0 ignored it and waited
 *        for a timeout instead.
 *
 * S2-09  No divergence detection. A vehicle that turns away holds its
 *        demand until an arbitrary timeout, blocking the junction for a
 *        vehicle that left.
 *
 * S2-14  Approach association from position alone. Two EVUs travelling
 *        in OPPOSITE directions on the same road map to the same
 *        approach -- and they need opposite phases. Heading is now
 *        required, using the heading_deg added in Phase 4 (the RDU was
 *        reading it for its geofence and then discarding it).
 ***********************************************************************/

#include "GreenwaveLink.h"
#include "ICU_EvuTypes.h"

uint16_t gwNodeDistanceM(int lane, int node);
uint8_t  gwNearNode(int lane);

// Defined near the bottom of this file; called from gwUpdateEvuTracks().
void gwSimEvuService();
void gwSimEvuStep(int lane, float seconds);

// =====================================================
// INTERSECTION GEOMETRY
// =====================================================
//
// [FIELD] The stop-line position of THIS intersection. Everything about
// distance, closing rate, divergence and ETA is measured against it.
//
// Placeholder is the RDU's surveyed node position from RDU.ino. It must
// be replaced with the real stop line before any field use -- an error
// here does not fail loudly, it biases every distance in the same
// direction and quietly shifts when preemptions occur.
// SET FOR THIS BENCH so the real EVU (13.02162, 77.66294) falls inside
// the tracking range. Against the previous placeholder it sat 3.9 km
// away, past EVU_MAX_RANGE_M, and could never associate an approach.
//
// This is roughly 500 m north of the EVU's position, so the bearing
// from vehicle to stop line is stable and meaningful even while the
// vehicle is not moving.
//
// [FIELD] REPLACE WITH THE REAL SURVEYED STOP LINE before deployment.
// An error here does not fail loudly -- it biases every distance,
// divergence test and ETA in the same direction, and quietly shifts
// when preemptions happen.
// FALLBACK ONLY. The live values come from NVS via ICU_Site.ino.
//
// These were compile-time constants, which meant testing several
// junctions required an edit-compile-flash cycle each time -- and four
// opportunities to flash the wrong site's numbers without noticing.
//
// That failure is silent. A stop line 200 m out raises no error; it
// biases every distance, ETA and divergence test in the same direction
// while the system goes on looking like it works.
//
// Set the real values over the console:
//     site here <lat> <lon>
//     site bearing <lane> <degrees>
//
// The ICU announces at boot when it is running on these defaults.
const double GW_FALLBACK_LAT = 13.0261;
const double GW_FALLBACK_LON = 77.6630;

// Live accessors, from ICU_Site.ino.
double gwSiteLat();
double gwSiteLon();
float  gwSiteBearing(int lane);
bool   gwSiteProvisioned();

#define ICU_STOPLINE_LAT   gwSiteLat()
#define ICU_STOPLINE_LON   gwSiteLon()

// [FIELD] Bearing FROM the stop line ALONG each approach, degrees true.
// A vehicle approaching on lane N travels on roughly the reciprocal.
//
// This is what makes approach association possible at all. Position says
// which road; only heading says which direction along it.
// FALLBACK ONLY -- see the note at GW_FALLBACK_LAT. Live bearings come
// from NVS via gwSiteBearing().
const float GW_FALLBACK_BEARING[GW_NUM_APPROACHES + 1] = {
    0.0f,      // [0] unused
    270.0f,    // lane 1 LEFT   -- approach runs west of the junction
    180.0f,    // lane 2 BOTTOM -- south
    90.0f      // lane 3 RIGHT  -- east
};

// A MISSING BEARING IS A SILENT WRONG ANSWER, NOT A COMPILE ERROR.
//
// This array is sized GW_NUM_APPROACHES+1 but written with a fixed list
// of initialisers. Add an approach and the extra slot is
// default-initialised to 0.0 -- which is not "unset", it is DUE NORTH,
// a perfectly valid bearing.
//
// The result would be a junction confidently associating vehicles
// against a compass direction nobody chose, with nothing in any log to
// show for it. That is the same failure shape as spec defect S1-01,
// which is the whole reason this project stopped trusting implicit
// geometry.
//
// C++ cannot check "did somebody remember to add a row", but it can
// check the count. If the initialiser list and the array size ever
// disagree, this fails the build instead of the road.
static_assert(sizeof(GW_FALLBACK_BEARING) / sizeof(GW_FALLBACK_BEARING[0])
              == GW_NUM_APPROACHES + 1,
    "APPROACH_BEARING_DEG has the wrong number of entries. Every approach "
    "needs an explicit surveyed bearing -- a default of 0.0 is due north, "
    "not 'unset', and would silently associate vehicles against it.");

// How far a vehicle's heading may differ from the approach bearing and
// still be considered on that approach.
//
// Wide, because roads bend and GPS heading is noisy at low speed. Its
// job is to separate "coming towards us" from "going the other way",
// which are 180 degrees apart -- not to measure lane discipline.
#define APPROACH_HEADING_TOLERANCE_DEG  70.0f

// Corridor width. Beyond this the vehicle is not on this approach.
#define APPROACH_CORRIDOR_M   120.0f

// Maximum range at which an EVU is tracked at all.
// [FIELD] Maximum range at which an EVU is tracked at all.
//
// 2000 m is a reasonable operational figure. It is also larger than the
// LoRa link will usually reach, so in practice the radio bounds this
// before the constant does.
//
// If a bench EVU sits further from ICU_STOPLINE_LAT/LON than this, it
// will be tracked and authenticated but never associated with an
// approach -- which looks like a logic failure and is really a
// configuration one. The log line above says which.
#define EVU_MAX_RANGE_M      2000.0f

// =====================================================
// 5D : ETA INPUTS
// =====================================================

// V_FLOOR REMOVED -- it was dead code.
//
// It existed to stop a stationary vehicle's infinite ETA losing
// arbitration. A bench run showed it never took effect: the ETA branch
// below returns -1 for a stopped vehicle that is still far out, which
// discards the floored value before anything reads it.
//
// More fundamentally, the problem it addressed did not exist.
// gwDemandScore() has never read ETA at all, so an infinite ETA could
// not lose an arbitration it was not an input to.
//
// The REAL gap was that arbitration ignored DISTANCE, so a stopped
// ambulance 100 m out tied with a moving one 800 m away. That is now
// fixed in ICU_Decision.ino with a proximity term -- distance is the
// honest measure here precisely because it does not collapse when a
// vehicle stops.

// Straight-line distance underestimates road distance, and the error is
// never in our favour: every vehicle looks CLOSER and every ETA looks
// SHORTER than the road allows.
//
// True path distance needs a road map the ICU does not have. This is a
// blunt correction -- the ratio of road distance to straight-line
// distance on an urban approach, dominated by the final turn.
//
// [FIELD] MEASURE IT. Drive each approach, compare odometer against
// straight-line, set per site. 1.0 disables it. The 1.15 default is a
// conservative urban figure and NOT a measurement.
//
// Applied to the ETA ONLY -- never to the NEAR_ZONE test or to
// divergence, which compare distances against each other where a
// constant factor cancels and would only add error.
#define PATH_FACTOR    1.15f

// Mirrors NEAR_ZONE_M in ICU_Decision.ino. Inside this range a
// stationary vehicle still gets a displayed ETA, because it is close
// enough that "about to arrive" is the useful reading regardless of
// what its speed says.
#define NEAR_ZONE_HINT_M  80.0f

// =====================================================
// VEHICLE REGISTRY  --  THE FIX FOR S2-01
// =====================================================
//
// Priority is a property the AUTHORITY assigns to a vehicle, not a
// property the vehicle asserts about itself.
//
// The packet carries priority_class and the ICU reads it -- purely to
// log when a unit's claim disagrees with the registry, which is a
// meaningful security signal. It NEVER influences arbitration.
//
// [FIELD] In deployment this belongs in NVS, provisioned and revocable
// without reflashing. Compiled in here so the mechanism exists and is
// testable; the interface below does not change when it moves.
// VehicleRegistryEntry is defined in ICU_EvuTypes.h -- see that file
// for why these types cannot live in a .ino tab.

static const VehicleRegistryEntry vehicleRegistry[] = {
    { "AMB_02", 1, true,  "Ambulance 02" },
    { "AMB_01", 1, true,  "Ambulance 01" },
    { "FIRE_1", 1, true,  "Fire tender 1" },
    { "POL_01", 3, true,  "Police 01"    },
    { "TEST_9", 9, false, "Decommissioned test unit -- REVOKED" },

    // BENCH SIMULATOR. Deliberately a SEPARATE IDENTITY from any real
    // vehicle.
    //
    // The simulator originally reused "AMB_02" and that was a mistake
    // with two distinct consequences, both seen on a bench run:
    //
    //   1. The real EVU and the simulated one shared ONE track. The real
    //      unit sat 3.9 km from the stop line while the simulation
    //      placed a vehicle at 500 m, so every update looked to the
    //      other like a 3.5 km jump and the plausibility check rejected
    //      all of them. Correct behaviour, applied to a situation that
    //      should never have existed.
    //
    //   2. Worse, the simulator's sequence numbers (900000+) advanced
    //      the shared anti-replay watermark, so every genuine packet
    //      from the real EVU was then rejected as stale. A test tool
    //      silently disabled the equipment it was meant to be testing.
    //
    // Priority 2, not 1, on purpose: distinct from AMB_02 so the
    // registry lookup is visibly doing something, and low enough that a
    // real Priority One vehicle still outranks the simulation if both
    // are somehow live at once.
    // BENCH SIMULATORS -- one identity PER LANE.
    //
    // Originally a single SIM_01 shared across all lanes, which made the
    // multi-approach arbitration test impossible: opening a vehicle on
    // lane 2 necessarily destroyed the one on lane 1, because they were
    // the same vehicle and a vehicle cannot be on two approaches.
    //
    // Distinct priorities on purpose. Equal ones would make arbitration
    // fall through to evidence and confidence -- which are identical for
    // two simulated vehicles -- and the test would then be measuring the
    // tie-breaker rather than the priority ordering it is supposed to
    // check.
    //
    // SIM_02 outranks SIM_01, so lane 2 must win against lane 1 even
    // when lane 1 was there first. That is the case worth testing:
    // priority-first arbitration, not first-come.
    { "SIM_01", 2, true,  "Bench simulator lane 1 -- not a real vehicle" },
    { "SIM_02", 1, true,  "Bench simulator lane 2 -- not a real vehicle" },
    { "SIM_03", 3, true,  "Bench simulator lane 3 -- not a real vehicle" },
};

#define VEHICLE_REGISTRY_COUNT \
    (sizeof(vehicleRegistry) / sizeof(vehicleRegistry[0]))

static const VehicleRegistryEntry* gwLookupVehicle(const char *id) {
    for (unsigned i = 0; i < VEHICLE_REGISTRY_COUNT; i++) {
        if (strncmp(id, vehicleRegistry[i].vehicleId, 6) == 0) {
            return &vehicleRegistry[i];
        }
    }
    return nullptr;
}

// Priority for an unregistered but correctly authenticated vehicle.
//
// Admitted at the LOWEST class rather than refused. It holds a valid
// key, so it is probably a legitimate unit that predates this registry
// or was added without the ICU being updated -- a provisioning gap, not
// an attack. Refusing outright would mean a real ambulance being ignored
// because of paperwork.
//
// But it must not outrank a registered vehicle, and the operator must be
// able to see that this one is unusual.
#define UNREGISTERED_PRIORITY  9

// =====================================================
// EVU TRACK
// =====================================================

// EvuTrackState is defined in ICU_EvuTypes.h


static const char* evuStateName(uint8_t s) {
    switch (s) {
        case EVU_ACTIVE:    return "ACTIVE";
        case EVU_DEGRADED:  return "DEGRADED";
        case EVU_DIVERGED:  return "DIVERGED";
        case EVU_COMPLETED: return "COMPLETED";
        case EVU_EXPIRED:   return "EXPIRED";
        case EVU_REJECTED:  return "REJECTED";
        default:            return "IDLE";
    }
}

// One missed packet is not an ambulance vanishing. EVU2 transmits every
// 2 s; this allows several losses before the track weakens.
// Derived from the ACTUAL relay rate, not the EVU's transmit rate.
//
// The EVU transmits every 2 s, but the RDU rate-limits relays to one
// per EVENT_MIN_INTERVAL_MS (4 s), and IF-2 delivery on this bench has
// run as low as 55%. So the interval between relays the ICU actually
// receives can reach 8-12 s with everything working correctly.
//
// A bench run showed AMB_02 EXPIRED between two good relays for exactly
// this reason: the old 20 s expiry was under three effective relay
// intervals, so ordinary loss looked like a vehicle that had vanished.
//
// Erring long is right here. An expired track that should still be live
// means the ICU forgets a vehicle that is still coming -- and the next
// packet opens a NEW track, losing the position history that divergence
// and stop-line detection depend on.
#define EVU_GRACE_MS     12000UL
#define EVU_EXPIRE_MS    45000UL

// Consecutive fixes with growing distance needed to call divergence.
//
// Three, not one. A single fix showing increased distance is ordinary
// GPS jitter, especially at low speed where an ambulance in traffic
// spends most of its time. Acting on one would abandon vehicles that
// had merely stopped moving.
#define DIVERGE_CONFIRM_COUNT  3

// EvuTrack is defined in ICU_EvuTypes.h


#define MAX_EVU_TRACKS 4
static EvuTrack evuTracks[MAX_EVU_TRACKS];

// =====================================================
// GEO HELPERS
// =====================================================

static double gwHaversineM(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double p1 = lat1 * DEG_TO_RAD;
    double p2 = lat2 * DEG_TO_RAD;
    double dp = (lat2 - lat1) * DEG_TO_RAD;
    double dl = (lon2 - lon1) * DEG_TO_RAD;

    double a = sin(dp / 2) * sin(dp / 2) +
               cos(p1) * cos(p2) * sin(dl / 2) * sin(dl / 2);
    return 2.0 * R * atan2(sqrt(a), sqrt(1.0 - a));
}

static double gwBearingDeg(double lat1, double lon1, double lat2, double lon2) {
    double p1 = lat1 * DEG_TO_RAD, p2 = lat2 * DEG_TO_RAD;
    double dl = (lon2 - lon1) * DEG_TO_RAD;
    double y = sin(dl) * cos(p2);
    double x = cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl);
    double b = atan2(y, x) * RAD_TO_DEG;
    return (b < 0) ? b + 360.0 : b;
}

static float gwAngleDiff(float a, float b) {
    float d = fabsf(a - b);
    while (d > 360.0f) d -= 360.0f;
    return (d > 180.0f) ? (360.0f - d) : d;
}

// =====================================================
// APPROACH ASSOCIATION  --  THE FIX FOR S2-14
// =====================================================
//
// Position alone is not enough, and the failure is not subtle: two
// emergency vehicles on the SAME road travelling in OPPOSITE directions
// are at nearly identical coordinates and map to the same approach --
// yet they need opposite signal phases. One of them would be served
// while the other was told nothing.
//
// The vehicle's bearing TO the stop line must also roughly match the
// approach's bearing. That is the difference between "on this road" and
// "on this road, coming this way".
static int gwAssociateApproach(const EvuTrack &t) {
    if (t.headingDeg < 0.0f) return 0;      // no heading, no association

    double bearingToStop = gwBearingDeg(t.lat, t.lon,
                                        ICU_STOPLINE_LAT, ICU_STOPLINE_LON);

    int   best     = 0;
    float bestDiff = 999.0f;

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        // Reciprocal: a vehicle approaching travels opposite to the
        // outbound bearing of its approach.
        float inbound = gwSiteBearing(lane) + 180.0f;
        if (inbound >= 360.0f) inbound -= 360.0f;

        float dHeading = gwAngleDiff(t.headingDeg, inbound);
        float dBearing = gwAngleDiff((float)bearingToStop, inbound);

        if (dHeading > APPROACH_HEADING_TOLERANCE_DEG) continue;
        if (dBearing > APPROACH_HEADING_TOLERANCE_DEG) continue;

        float score = dHeading + dBearing;
        if (score < bestDiff) { bestDiff = score; best = lane; }
    }

    return best;
}

// =====================================================
// PLAUSIBILITY  --  THE FIX FOR S2-05
// =====================================================
//
// A valid signature proves the sender holds a key. It does not make the
// payload true.
//
// Checks the position delta against the vehicle's own reported speed. A
// unit reporting 40 km/h that jumps 800 m between consecutive fixes is
// internally inconsistent -- whatever the cause, its position cannot be
// used to decide when to stop cross traffic.
static bool gwPositionPlausible(EvuTrack &t, double newLat, double newLon,
                                float speedKmph, unsigned long dtMs) {
    if (t.lat == 0.0 && t.lon == 0.0) return true;   // first fix
    if (dtMs == 0) return true;

    double moved = gwHaversineM(t.lat, t.lon, newLat, newLon);

    // Generous ceiling: reported speed plus a wide allowance, plus a
    // floor for GPS scatter while stationary. The aim is to catch
    // fabrication and gross error, not to police driving.
    float maxSpeedMs = (speedKmph / 3.6f) * 2.0f + 15.0f;
    double maxMoved  = maxSpeedMs * (dtMs / 1000.0) + 50.0;

    if (moved > maxMoved) {
        Serial.printf("[EVU] %s IMPLAUSIBLE: moved %.0fm in %lums at reported "
                      "%.0f km/h (max %.0fm) -- position rejected\n",
                      t.vehicleId, moved, dtMs, speedKmph, maxMoved);
        return false;
    }
    return true;
}

// =====================================================
// TRACK LOOKUP
// =====================================================

static EvuTrack* gwFindEvuTrack(const char *id) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        if (evuTracks[i].inUse &&
            strncmp(evuTracks[i].vehicleId, id, 7) == 0) {
            return &evuTracks[i];
        }
    }
    return nullptr;
}

static EvuTrack* gwAllocEvuTrack(const char *id) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        if (!evuTracks[i].inUse) {
            evuTracks[i] = EvuTrack();
            evuTracks[i].inUse = true;
            strncpy(evuTracks[i].vehicleId, id, 7);
            evuTracks[i].vehicleId[7] = '\0';
            return &evuTracks[i];
        }
    }

    // Full. Evict the LOWEST priority track -- never the highest, and
    // never simply the oldest: a Priority One ambulance held up in
    // traffic is exactly the track that has been open longest.
    int worst = -1;
    uint8_t worstPri = 0;
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        if (evuTracks[i].priority >= worstPri) {
            worstPri = evuTracks[i].priority;
            worst = i;
        }
    }
    if (worst < 0) return nullptr;

    Serial.printf("[EVU] track table full -- evicting %s (pri %u)\n",
                  evuTracks[worst].vehicleId, (unsigned)worstPri);

    evuTracks[worst] = EvuTrack();
    evuTracks[worst].inUse = true;
    strncpy(evuTracks[worst].vehicleId, id, 7);
    evuTracks[worst].vehicleId[7] = '\0';
    return &evuTracks[worst];
}

// =====================================================
// INGEST AN AUTHENTICATED EVU RELAY
// =====================================================

void gwEvuOnRelay(const LoRaEventFrame &f, bool directToIcu) {
    char id[8];
    memcpy(id, f.vehicle_id, 7);
    id[7] = '\0';

    unsigned long now = millis();

    EvuTrack *t = gwFindEvuTrack(id);
    bool isNew = false;

    if (t == nullptr) {
        t = gwAllocEvuTrack(id);
        if (t == nullptr) return;
        t->firstSeenMs = now;
        isNew = true;
    }

    // ---- REPLAY / ORDERING ----
    //
    // Rejected only when strictly older. Equal sequence is a duplicate
    // relayed by the second RDU, which is normal and expected once both
    // nodes on an approach can hear the vehicle.
    if (t->seqStarted && f.tx_seq < t->lastSeq) {
        Serial.printf("[EVU] %s stale seq %lu < %lu -- ignored\n",
                      id, (unsigned long)f.tx_seq, (unsigned long)t->lastSeq);
        return;
    }
    bool freshSeq = (!t->seqStarted || f.tx_seq > t->lastSeq);
    t->lastSeq    = f.tx_seq;
    t->seqStarted = true;

    // ---- PRIORITY : REGISTRY, NOT PACKET  (S2-01) ----
    const VehicleRegistryEntry *reg = gwLookupVehicle(id);

    t->claimedPriority = f.priority;

    if (reg != nullptr) {
        t->registered = true;
        t->priority   = reg->priority;

        if (!reg->enabled) {
            if (t->state != EVU_REJECTED) {
                Serial.printf("[EVU] %s IS REVOKED (%s) -- tracked, never acted on\n",
                              id, reg->description);
            }
            t->state    = EVU_REJECTED;
            t->evidence = LE_NONE;
            t->lastSeenMs = now;
            return;
        }

        // A disagreement between the claim and the registry is a
        // meaningful security signal, not a formatting quirk: it means a
        // unit is asserting an importance it was not granted.
        if (f.priority != 0 && f.priority != reg->priority) {
            Serial.printf("[EVU] *** %s CLAIMS priority %u, REGISTRY SAYS %u -- "
                          "registry wins ***\n",
                          id, (unsigned)f.priority, (unsigned)reg->priority);
        }
    } else {
        t->registered = false;
        t->priority   = UNREGISTERED_PRIORITY;
        if (isNew) {
            Serial.printf("[EVU] %s authenticated but NOT IN REGISTRY -- "
                          "admitted at lowest priority %u\n",
                          id, UNREGISTERED_PRIORITY);
        }
    }

    // ---- POSITION ----
    double newLat = f.latitude  / 10000000.0;
    double newLon = f.longitude / 10000000.0;
    float  spd    = f.speed_kmph / 100.0f;

    bool gpsOk = (f.flags & EVF_GPS_VALID) != 0;

    if (gpsOk && freshSeq) {
        unsigned long dt = (t->lastSeenMs == 0) ? 0 : (now - t->lastSeenMs);

        if (!gwPositionPlausible(*t, newLat, newLon, spd, dt)) {
            // Position discarded, track kept. The vehicle is still
            // authenticated and still probably real; it is its POSITION
            // that cannot be trusted. Dropping the whole track would let
            // one corrupt fix erase a genuine emergency.
            t->lastSeenMs = now;
            return;
        }

        t->lat = newLat;
        t->lon = newLon;

        t->prevDistanceM = t->distanceM;
        t->distanceM = (float)gwHaversineM(newLat, newLon,
                                           ICU_STOPLINE_LAT, ICU_STOPLINE_LON);
    }

    // ---- 5D: rolling mean speed ----
    //
    // Updated only on a FRESH sequence. A duplicate relayed by the second
    // RDU carries the same speed sample, and counting it twice would
    // weight that one instant double in the mean.
    if (freshSeq) {
        t->speedHistory[t->speedHistoryHead] = spd;
        t->speedHistoryHead =
            (uint8_t)((t->speedHistoryHead + 1) % EvuTrack::SPEED_HISTORY);
        if (t->speedHistoryCount < EvuTrack::SPEED_HISTORY) {
            t->speedHistoryCount++;
        }

        float sum = 0.0f;
        for (uint8_t i = 0; i < t->speedHistoryCount; i++) {
            sum += t->speedHistory[i];
        }
        t->speedAvgKmph = sum / (float)t->speedHistoryCount;
    }

    t->speedKmph = spd;

    // heading_deg: 0xFFFF means no valid heading, which is NOT 0 --
    // 0 is due north and is a perfectly good heading.
    t->headingDeg = (f.heading_deg == 0xFFFF)
                  ? -1.0f
                  : (f.heading_deg / 100.0f);

    // ---- EMERGENCY FLAG FALLING EDGE  (S2-08) ----
    t->prevEmergencyFlag = t->emergencyFlag;
    t->emergencyFlag     = (f.flags & EVF_EMERGENCY) != 0;

    if (t->prevEmergencyFlag && !t->emergencyFlag) {
        // The driver switched the siren off. This is the cheapest and
        // most reliable release signal in the entire system -- explicit,
        // authenticated, and from the only party who actually knows the
        // event is over.
        //
        // v1.0 discarded it and waited for a timeout instead, holding
        // the junction after the vehicle itself had said it was done.
        Serial.printf("[EVU] %s EMERGENCY FLAG CLEARED -- explicit release\n", id);
        t->state    = EVU_COMPLETED;
        t->evidence = LE_NONE;
        t->lastSeenMs = now;
        return;
    }

    if (!t->emergencyFlag) {
        // Never was an emergency. Tracked quietly, never acted on.
        t->state      = EVU_IDLE;
        t->evidence   = LE_NONE;
        t->lastSeenMs = now;
        return;
    }

    // ---- APPROACH ASSOCIATION  (S2-14) ----
    //
    // STICKY once established.
    //
    // A vehicle that turns away changes heading, which makes fresh
    // association fail -- so on the first U-turn fix the track dropped
    // out of the approach entirely and the divergence counter below was
    // never reached. The release came back as EXPIRED instead of
    // DIVERGED, and spec S2-09 was unreachable in practice.
    //
    // That is backwards. A vehicle that turns away is still the vehicle
    // we were tracking, and the whole point of divergence detection is
    // to NOTICE it leaving rather than to quietly lose it. Losing the
    // association discards exactly the information the check needs.
    //
    // So: association is computed while the track has no lane, and
    // retained afterwards. It is released when the track ends -- by
    // divergence, by completion, or by expiry -- which are the three
    // ways a vehicle genuinely stops being ours.
    int lane = (t->lane != 0) ? t->lane : gwAssociateApproach(*t);

    if (lane == 0 || t->distanceM > EVU_MAX_RANGE_M) {
        // Log WHY, once per reason change. A bench run produced a track
        // sitting silently at lane=0 with no indication whether the
        // cause was range, heading, or a genuine association failure --
        // three different problems with three different fixes.
        static uint8_t lastReason[MAX_EVU_TRACKS] = {0};
        int idx = (int)(t - evuTracks);
        uint8_t reason = (t->distanceM > EVU_MAX_RANGE_M) ? 1
                       : (t->headingDeg < 0.0f)           ? 2
                       : 3;

        if (idx >= 0 && idx < MAX_EVU_TRACKS && lastReason[idx] != reason) {
            lastReason[idx] = reason;
            switch (reason) {
                case 1:
                    Serial.printf("[EVU] %s not associated: %.0fm from the stop "
                                  "line, beyond the %.0fm tracking range. Check "
                                  "ICU_STOPLINE_LAT/LON in ICU_EVU.ino.\n",
                                  id, t->distanceM, EVU_MAX_RANGE_M);
                    break;
                case 2:
                    Serial.printf("[EVU] %s not associated: NO VALID HEADING "
                                  "(vehicle stationary). A stopped vehicle has "
                                  "no direction of travel, so no approach can "
                                  "be inferred.\n", id);
                    break;
                default:
                    Serial.printf("[EVU] %s not associated: heading %.0f deg "
                                  "does not match any approach within %.0f deg\n",
                                  id, t->headingDeg,
                                  APPROACH_HEADING_TOLERANCE_DEG);
                    break;
            }
        }

        t->lane     = 0;
        t->state    = EVU_IDLE;
        t->evidence = LE_NONE;
        t->lastSeenMs = now;
        return;
    }
    t->lane = lane;

    // ---- DIVERGENCE  (S2-09) ----
    //
    // BOTH conditions required: the distance must be growing AND it must
    // do so consistently. A single fix showing an increase is ordinary
    // GPS jitter, and an ambulance stationary in traffic produces plenty
    // of it. Acting on one would abandon exactly the vehicles that need
    // the junction most.
    if (t->prevDistanceM > 0.0f && t->distanceM > t->prevDistanceM + 5.0f) {
        if (t->growingCount < 255) t->growingCount++;
    } else if (t->distanceM < t->prevDistanceM) {
        t->growingCount = 0;
    }

    if (t->growingCount >= DIVERGE_CONFIRM_COUNT) {
        Serial.printf("[EVU] %s DIVERGED -- distance grew over %u fixes "
                      "(%.0fm -> %.0fm)\n",
                      id, (unsigned)t->growingCount,
                      t->prevDistanceM, t->distanceM);
        t->state    = EVU_DIVERGED;
        t->evidence = LE_NONE;
        t->lastSeenMs = now;
        return;
    }

    // ---- STOP LINE CROSSED ----
    //
    // Requires the distance to be growing as well, not just small.
    // Crossing alone is defeated by ordinary GPS jitter at the line: a
    // vehicle waiting AT the stop line would otherwise be repeatedly
    // declared to have passed it.
    if (t->distanceM < 15.0f && t->growingCount >= 2) {
        Serial.printf("[EVU] %s passed the stop line\n", id);
        t->state    = EVU_COMPLETED;
        t->evidence = LE_NONE;
        t->lastSeenMs = now;
        return;
    }

    // ---- ACTIVE ----
    t->state          = EVU_ACTIVE;
    t->directReceived = directToIcu;
    t->evidence       = directToIcu ? LE_EVU_DIRECT : LE_EVU_INDIRECT;

    // RECEDING BUT NOT YET CONFIRMED DIVERGED.
    //
    // Divergence needs DIVERGE_CONFIRM_COUNT consecutive growing fixes,
    // because one is ordinary GPS jitter. But during those fixes the
    // vehicle was still reported as EVU_INDIRECT, which the decision
    // layer maps to COMMIT -- so the console asked an officer to act on
    // a vehicle already driving away, for as long as confirmation took.
    //
    // A bench run showed two full fixes of COMMIT after the U-turn.
    //
    // Downgrading to DEGRADED here maps to PREPARE instead: the demand
    // stands, because the vehicle may yet turn back, but it stops
    // demanding action. That is precisely what the two-stage model is
    // for -- evidence that has become uncertain should produce the
    // cheap reversible stage, not the expensive one.
    if (t->growingCount > 0) {
        t->evidence = LE_EVU_DEGRADED;
    }

    // ---- ETA ----
    //
    // Real telemetry: the vehicle's own reported speed and a measured
    // distance to the stop line. Far better grounded than the acoustic
    // estimate, which infers speed from a two-microphone delta.
    if (t->growingCount > 0) {
        // Moving AWAY. There is no arrival time for a vehicle that is
        // leaving, and distance/speed would produce a RISING number --
        // a console showing an ETA that grows while the ambulance
        // departs is worse than showing nothing, because it reads as a
        // vehicle slowing down on approach.
        //
        // Below DIVERGE_CONFIRM_COUNT the track is still live and may
        // yet turn back, so the demand stands; only the arrival
        // estimate is withheld.
        t->etaS = -1;
    } else {
        // ---- 5D: ETA ----
        //
        // Three corrections over the naive distance/speed:
        //
        //  ROLLING SPEED -- damps GPS jitter that otherwise reaches the
        //  console as an ETA bouncing by seconds between fixes. An
        //  officer watching a number jump learns to distrust it.
        //
        //  V_FLOOR -- a stopped vehicle no longer produces an infinite
        //  ETA that reads as "not urgent" and loses arbitration to
        //  anything moving.
        //
        //  PATH_FACTOR -- straight-line distance makes every vehicle
        //  look closer and every ETA shorter than the road allows.
        // Guarded only against divide-by-zero. A speed this low is
        // handled by the -1 branch below, which says "no honest arrival
        // time" rather than inventing one.
        float vKmph = t->speedAvgKmph;
        if (vKmph < 1.0f) vKmph = 1.0f;

        float pathM  = t->distanceM * PATH_FACTOR;
        float etaSec = pathM / (vKmph / 3.6f);

        if (etaSec > 3600.0f) {
            t->etaS = -1;
        } else if (t->speedAvgKmph < 2.0f && t->distanceM > NEAR_ZONE_HINT_M) {
            // Stationary AND still far out. The floored ETA remains a
            // useful arbitration input, but it is not an honest
            // prediction, so the console shows "--" rather than a number
            // an officer might time an override against.
            t->etaS = -1;
        } else {
            t->etaS = (int16_t)etaSec;
        }
    }

    t->lastSeenMs = now;

    if (isNew) {
        Serial.printf("[EVU] %s TRACK OPEN  lane=%d pri=%u(%s) dist=%.0fm "
                      "hdg=%.0f spd=%.0f eta=%d %s\n",
                      id, lane, (unsigned)t->priority,
                      t->registered ? "registry" : "UNREGISTERED",
                      t->distanceM, t->headingDeg, t->speedKmph, (int)t->etaS,
                      directToIcu ? "direct" : "relayed");
    }
}

// =====================================================
// PERIODIC MAINTENANCE
// =====================================================

void gwUpdateEvuTracks() {
    // Bench simulator auto-stepping, if any is scheduled.
    //
    // MUST run BEFORE `now` is sampled. It delivers packets through
    // gwEvuOnRelay(), which stamps lastSeenMs = millis(). Sampling `now`
    // first left it a millisecond BEHIND those stamps, and the unsigned
    // subtraction below then wrapped to 4294967295 ms -- a 49-day
    // silence computed for a track that had just received a packet.
    //
    // A bench run showed exactly that: "-> DEGRADED (no packet for
    // 4294967295ms)" immediately followed by EXPIRED, killing a live
    // track one step into the test.
    //
    // Not a simulator-only hazard. Any future path that delivers a
    // packet between the sample and the comparison reproduces it, and
    // the symptom is an emergency vehicle silently disappearing.
    gwSimEvuService();

    unsigned long now = millis();

    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;

        // Guarded subtraction. If lastSeenMs is somehow ahead of now,
        // the packet is newer than this pass and the silence is zero --
        // never 49 days. Belt and braces alongside the ordering fix
        // above, because the consequence of getting it wrong is a track
        // that vanishes rather than one that lingers.
        unsigned long silence =
            (now >= t.lastSeenMs) ? (now - t.lastSeenMs) : 0;

        if (t.state == EVU_COMPLETED || t.state == EVU_DIVERGED ||
            t.state == EVU_REJECTED) {
            if (silence > EVU_EXPIRE_MS) {
                t.inUse = false;
            }
            continue;
        }

        if (t.state == EVU_ACTIVE && silence > EVU_GRACE_MS) {
            // DEGRADED, not gone. One missed packet is not an ambulance
            // vanishing, and dropping straight to expired would make the
            // system flicker on ordinary radio loss.
            t.state    = EVU_DEGRADED;
            t.evidence = LE_EVU_DEGRADED;
            Serial.printf("[EVU] %s -> DEGRADED (no packet for %lums)\n",
                          t.vehicleId, silence);
        }

        if (silence > EVU_EXPIRE_MS) {
            Serial.printf("[EVU] %s EXPIRED\n", t.vehicleId);
            t.state    = EVU_EXPIRED;
            t.evidence = LE_NONE;
            t.inUse    = false;
        }
    }
}

// =====================================================
// QUERIES FOR THE DECISION LAYER
// =====================================================

// True if the strongest live EVU track on this approach is currently
// moving AWAY from the stop line, but has not yet met the confirmation
// count for divergence.
//
// The decision layer uses this to cap the stage at PREPARE.
//
// A bench run showed the system escalating PREPARE -> COMMIT on a
// vehicle that was already departing: a fresh packet cleared the
// DEGRADED state, and nothing in the stage logic consulted direction.
// COMMIT means "stop cross traffic now", and a vehicle that has moved
// away on consecutive fixes does not warrant that -- even though the
// track is legitimately still open, because it may yet turn back.
//
// Withholding COMMIT while still holding the track is exactly the
// distinction the two-stage model exists to express: the demand
// survives, the request for action does not.
bool gwEvuIsReceding(int lane) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse)       continue;
        if (t.lane != lane) continue;
        if (t.state != EVU_ACTIVE && t.state != EVU_DEGRADED) continue;
        if (t.growingCount > 0) return true;
    }
    return false;
}

// Strongest live EVU demand on an approach.
bool gwEvuDemand(int lane, uint8_t *priority, uint8_t *evidence, int16_t *etaS) {
    bool found = false;
    uint8_t bestPri = 255;

    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;
        if (t.lane != lane) continue;
        if (t.evidence == LE_NONE) continue;
        if (t.state != EVU_ACTIVE && t.state != EVU_DEGRADED) continue;

        // Lower number = higher priority.
        if (t.priority < bestPri) {
            bestPri   = t.priority;
            *priority = t.priority;
            *evidence = t.evidence;
            *etaS     = t.etaS;
            found     = true;
        }
    }
    return found;
}

// Release reason from the EVU layer, or 0xFF if the EVU layer has
// nothing to say and the caller should fall back to acoustic state.
//
// Exists because the decision layer previously read only the acoustic
// track when choosing a reason, so an EVU divergence was reported as
// EXPIRED. The reason code is the only thing separating "the ambulance
// turned away" from "we lost it", and those say opposite things about
// whether the system is working.
// Distance to the stop line for the strongest live track on this
// approach, or 0 when there is none.
//
// Feeds the NEAR_ZONE clamp in ICU_Decision.ino. Returns 0 rather
// than a sentinel so a caller that forgets to check cannot
// accidentally treat "unknown" as "at the stop line".
float gwEvuDistanceM(int lane) {
    float best = 0.0f;
    uint8_t bestPri = 255;

    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;
        if (t.lane != lane) continue;
        if (t.state != EVU_ACTIVE && t.state != EVU_DEGRADED) continue;
        if (t.priority < bestPri) { bestPri = t.priority; best = t.distanceM; }
    }
    return best;
}

uint8_t gwEvuReleaseReason(int lane) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;
        if (t.lane != lane) continue;

        if (t.state == EVU_DIVERGED)  return LR_DIVERGED;

        // COMPLETED is reached by the emergency flag falling or by the
        // vehicle crossing the stop line. Both are the event ending
        // properly rather than being lost.
        if (t.state == EVU_COMPLETED) return LR_FLAG_OFF;
    }
    return 0xFF;
}

// End-state of an EVU track on this approach, for release reporting.
//
// Returns 0 when nothing ended. The decision layer previously read only
// the ACOUSTIC state here, so an EVU divergence was reported to the
// console as EXPIRED -- erasing the distinction between "the vehicle
// turned away" and "we stopped hearing anything", which is the whole
// reason release reasons exist.
uint8_t gwEvuEndState(int lane) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;
        if (t.lane != lane) continue;

        if (t.state == EVU_DIVERGED)  return EVU_DIVERGED;
        if (t.state == EVU_COMPLETED) return EVU_COMPLETED;
        if (t.state == EVU_EXPIRED)   return EVU_EXPIRED;
        if (t.state == EVU_REJECTED)  return EVU_REJECTED;
    }
    return 0;
}

void gwPrintEvuTracks() {
    bool any = false;
    for (int i = 0; i < MAX_EVU_TRACKS; i++) if (evuTracks[i].inUse) any = true;

    if (!any) { Serial.println("[EVU] no tracks"); return; }

    Serial.println("[EVU]");
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        EvuTrack &t = evuTracks[i];
        if (!t.inUse) continue;

        Serial.printf("  %-7s %-9s lane=%d pri=%u%s dist=%.0fm spd=%.0f "
                      "hdg=%.0f eta=%d %s%s\n",
                      t.vehicleId, evuStateName(t.state), t.lane,
                      (unsigned)t.priority,
                      t.registered ? "" : "(UNREG)",
                      t.distanceM, t.speedKmph, t.headingDeg, (int)t.etaS,
                      gwEvidenceShort(t.evidence),
                      (t.claimedPriority != 0 &&
                       t.claimedPriority != t.priority) ? "  *CLAIM MISMATCH*" : "");
    }
}


// =====================================================
// BENCH SIMULATION  --  SYNTHETIC VEHICLE MOVEMENT
// =====================================================
//
// WHY THIS IS NECESSARY
//
// Three defects remain unverified because all three need a MOVING
// vehicle: position plausibility (S2-05), divergence (S2-09) and
// approach association (S2-14).
//
// A stationary EVU cannot exercise any of them. Its GPS reports no
// usable heading, so no approach can be associated, so nothing
// downstream runs. That is the system behaving correctly -- a parked
// vehicle genuinely has no direction of travel -- but it means the
// logic is untested.
//
// Driving a real vehicle would not fix this either. It cannot produce a
// repeatable 40 km/h approach, a clean U-turn at a chosen distance, or
// a position that jumps 3 km between fixes. Those are the cases that
// matter, and the last one cannot be produced by a working GPS at all.
//
// WHAT MAKES THIS HONEST
//
// Every command below builds a real LoRaEventFrame and calls
// gwEvuOnRelay() -- the same entry point an authenticated relay
// reaches. The registry lookup, plausibility check, approach
// association, divergence counter, stop-line test, ETA and state
// machine are all production code. Only the coordinates are synthetic.
//
// The equivalent tool for the acoustic path found four real bugs,
// including one that would have silently disabled direction inference
// during long siren episodes.

struct EvuSimState {
    bool     active     = false;
    double   lat        = 0;
    double   lon        = 0;
    float    distanceM  = 0;
    float    speedKmph  = 0;
    float    headingDeg = 0;
    bool     emergency  = true;
    bool     departing  = false;
    // Auto-step scheduling. See gwSimEvuRun().
    uint8_t       autoStepsLeft = 0;
    unsigned long nextStepMs    = 0;
    float         autoStepSec   = 2.0f;
};

static EvuSimState evuSim[GW_NUM_APPROACHES + 1];

// ONE sequence counter for the whole simulator, not one per lane.
//
// SIM_01 is a single vehicle with a single track and therefore a single
// anti-replay watermark. Per-lane counters meant moving the simulated
// vehicle to another approach restarted its sequence, and the ICU
// correctly rejected the result as stale -- "simevu 2 approach" failed
// with "stale seq 1001 < 1008".
//
// The replay rule was right; the simulator was wrong to have two
// counters for one identity.
static uint32_t gwSimSeq = 1000;

// Vehicle id for the simulator on a given lane.
static const char* gwSimIdFor(int lane) {
    switch (lane) {
        case 1:  return "SIM_01";
        case 2:  return "SIM_02";
        case 3:  return "SIM_03";
        default: return "SIM_01";
    }
}

// Point at distance d from the stop line, along an approach's bearing.
static void gwSimDestination(float bearingDeg, float distM,
                             double &outLat, double &outLon) {
    const double R = 6371000.0;
    double br = bearingDeg * DEG_TO_RAD;
    double d  = distM / R;

    double lat1 = ICU_STOPLINE_LAT * DEG_TO_RAD;
    double lon1 = ICU_STOPLINE_LON * DEG_TO_RAD;

    double lat2 = asin(sin(lat1) * cos(d) + cos(lat1) * sin(d) * cos(br));
    double lon2 = lon1 + atan2(sin(br) * sin(d) * cos(lat1),
                               cos(d) - sin(lat1) * sin(lat2));

    outLat = lat2 * RAD_TO_DEG;
    outLon = lon2 * RAD_TO_DEG;
}

static void gwSimEvuEmit(int lane) {
    EvuSimState &v = evuSim[lane];

    LoRaEventFrame f;
    memset(&f, 0, sizeof(f));

    f.hdr.lane_id = (uint8_t)lane;
    f.hdr.node_id = 1;

    // Per-lane identity, so two simulated vehicles can be live at once.
    memcpy(f.vehicle_id, gwSimIdFor(lane), 6);

    f.flags = EVF_GPS_VALID;
    if (v.emergency) f.flags |= EVF_EMERGENCY;

    // The CLAIM. Deliberately set to 9 so every simulated run also
    // exercises the registry override -- a simulation that quietly used
    // the correct priority would never notice if the override broke.
    f.priority = 9;

    f.latitude    = (int32_t)lround(v.lat * 10000000.0);
    f.longitude   = (int32_t)lround(v.lon * 10000000.0);
    f.speed_kmph  = (uint16_t)lround(v.speedKmph * 100.0f);

    // Below walking pace the RDU reports no heading, because a
    // stationary GPS produces noise. Mirrored here so the simulation
    // cannot express something the real transport would never send.
    // Mirrors the RDU: below walking pace a real GPS reports no usable
    // heading, so the simulator must not either.
    //
    // EXCEPTION once a track is established. A real vehicle that slows
    // to a stop mid-approach has its association already fixed -- the
    // ICU keeps it (association is sticky) and the heading field stops
    // mattering. Continuing to send the last good heading models that,
    // and without it a simulated vehicle that stops would lose its lane
    // and the V_FLOOR path could never be reached.
    bool established = (gwFindEvuTrack(gwSimIdFor(lane)) != nullptr);

    f.heading_deg = (v.speedKmph >= 3.0f || established)
                  ? (uint16_t)lround(v.headingDeg * 100.0f)
                  : 0xFFFF;

    f.tx_seq   = ++gwSimSeq;
    f.rssi_if1 = -85;
    f.snr_if1_x10 = 100;

    gwEvuOnRelay(f, false);
}

// simevu <lane> approach <dist_m> <speed_kmph>
// Drops any existing ICU-side track for a vehicle, so a new simulated
// run starts clean.
//
// Without this, moving SIM_01 from lane 1 to lane 2 looked like a 650 m
// teleport and was correctly rejected -- the plausibility check doing
// its job on a vehicle that had apparently jumped across the junction.
// Real vehicles do not do that; a bench operator retargeting the
// simulator does.
void gwEvuDropTrack(const char *id) {
    for (int i = 0; i < MAX_EVU_TRACKS; i++) {
        if (evuTracks[i].inUse &&
            strncmp(evuTracks[i].vehicleId, id, 7) == 0) {
            evuTracks[i] = EvuTrack();
        }
    }
}

void gwSimEvuApproach(int lane, float distM, float speedKmph) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    // Clear only THIS lane's previous run, so a fresh approach is not
    // measured against a stale position.
    //
    // Other lanes are deliberately left alone: each has its own vehicle
    // identity now, and wiping them would make it impossible to have two
    // approaches live at once -- which is exactly the situation the
    // arbitration logic exists for and has never been tested against.
    gwEvuDropTrack(gwSimIdFor(lane));

    EvuSimState &v = evuSim[lane];
    v = EvuSimState();
    v.active    = true;
    v.distanceM = distM;
    v.speedKmph = speedKmph;

    // Heading is the INBOUND bearing -- the reciprocal of the approach's
    // outbound bearing. A vehicle approaching travels towards the stop
    // line, which is the whole basis of approach association: position
    // says which road, only heading says which way along it.
    v.headingDeg = gwSiteBearing(lane) + 180.0f;
    if (v.headingDeg >= 360.0f) v.headingDeg -= 360.0f;

    gwSimDestination(gwSiteBearing(lane), distM, v.lat, v.lon);

    Serial.printf("[SIMEVU] %s on L%d: from %.0fm at %.0f km/h "
                  "heading %.0f (lat %.6f lon %.6f)\n",
                  gwSimIdFor(lane), lane, distM, speedKmph,
                  v.headingDeg, v.lat, v.lon);

    gwSimEvuEmit(lane);
}

// simevu <lane> step [seconds]
// Advances the vehicle by speed x time, then transmits.
void gwSimEvuStep(int lane, float seconds) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) {
        Serial.printf("[SIMEVU] L%d no simulated vehicle -- run "
                      "'simevu %d approach <m> <kmph>' first\n", lane, lane);
        return;
    }

    float moved = (v.speedKmph / 3.6f) * seconds;

    if (v.departing) {
        v.distanceM += moved;
    } else {
        v.distanceM -= moved;
        if (v.distanceM < 0.0f) v.distanceM = 0.0f;
    }

    gwSimDestination(gwSiteBearing(lane), v.distanceM, v.lat, v.lon);

    Serial.printf("[SIMEVU] L%d step %.0fs -> %.0fm %s\n",
                  lane, seconds, v.distanceM,
                  v.departing ? "(departing)" : "(approaching)");

    gwSimEvuEmit(lane);
}

// simevu <lane> diverge
// Reverses heading and starts moving away. Exercises S2-09.
void gwSimEvuDiverge(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) return;

    v.departing  = true;
    v.headingDeg = gwSiteBearing(lane);   // outbound now

    Serial.printf("[SIMEVU] L%d U-TURN -- heading now %.0f, moving away. "
                  "Divergence needs %d consecutive growing fixes, so run "
                  "'step' at least %d more times.\n",
                  lane, v.headingDeg, DIVERGE_CONFIRM_COUNT,
                  DIVERGE_CONFIRM_COUNT);
}

// simevu <lane> teleport <metres>
// Jumps the position without a plausible travel time. Exercises S2-05.
void gwSimEvuTeleport(int lane, float jumpM) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) return;

    // Transmit the impossible position, then PUT THE SIMULATED VEHICLE
    // BACK where it was.
    //
    // The first version moved v.distanceM permanently. The ICU rejected
    // the jump and kept the track at its last good position -- correctly
    // -- but the simulator now believed the vehicle was 3 km away. Every
    // subsequent step was measured from there, so each one looked like
    // another 3 km jump and was also rejected. Divergence could never
    // run, and the test after this one silently tested nothing.
    //
    // A rejected position is by definition one the vehicle did NOT move
    // to. The simulator has to model that, or it stops agreeing with the
    // system it is testing.
    double savedLat = v.lat;
    double savedLon = v.lon;
    float  savedDst = v.distanceM;

    float bogus = v.distanceM + jumpM;
    gwSimDestination(gwSiteBearing(lane), bogus, v.lat, v.lon);

    Serial.printf("[SIMEVU] L%d TELEPORT: claiming %.0fm (really %.0fm). The "
                  "packet is correctly signed; only its CONTENTS are "
                  "impossible.\n",
                  lane, bogus, savedDst);

    gwSimEvuEmit(lane);

    v.lat       = savedLat;
    v.lon       = savedLon;
    v.distanceM = savedDst;

    Serial.printf("[SIMEVU] L%d vehicle remains at %.0fm -- a rejected "
                  "position is one it never moved to.\n", lane, savedDst);
}

// simevu <lane> off | on
void gwSimEvuEmergency(int lane, bool on) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) return;

    v.emergency = on;
    Serial.printf("[SIMEVU] L%d emergency flag %s\n", lane, on ? "ON" : "OFF");
    gwSimEvuEmit(lane);
}

// simevu <lane> run <count> [seconds_per_step]
//
// Emits a sequence of fixes on a TIMER rather than on keystrokes.
//
// Necessary because the simulator only transmits when a command is
// typed, and typing is slower than EVU_GRACE_MS. A bench run showed
// SIM_01 dropping to DEGRADED after 12 s of operator thinking time --
// the grace logic behaving exactly as designed, applied to a "vehicle"
// that had simply stopped being typed at.
//
// Two seconds per step matches the EVU's real transmit interval, so the
// track stays ACTIVE and the plausibility window sees realistic spacing.
//
// Scheduled from gwUpdateEvuTracks() rather than looping with delay():
// a blocking loop would stop the ICU answering PING, and the ERC's
// five-second fail-safe would clear the very alert being tested.
void gwSimEvuRun(int lane, int count, float secondsPerStep) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) {
        Serial.printf("[SIMEVU] L%d nothing to run -- 'approach' first\n", lane);
        return;
    }

    if (count < 1)   count = 1;
    if (count > 30)  count = 30;

    v.autoStepsLeft = (uint8_t)count;
    v.autoStepSec   = secondsPerStep;
    v.nextStepMs    = millis();

    Serial.printf("[SIMEVU] L%d running %d steps at %.1fs intervals\n",
                  lane, count, secondsPerStep);
}

// Called from gwUpdateEvuTracks().
void gwSimEvuService() {
    unsigned long now = millis();

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        EvuSimState &v = evuSim[lane];

        if (!v.active || v.autoStepsLeft == 0) continue;
        if ((long)(now - v.nextStepMs) < 0)    continue;

        v.autoStepsLeft--;
        v.nextStepMs = now + (unsigned long)(v.autoStepSec * 1000.0f);

        gwSimEvuStep(lane, v.autoStepSec);
    }
}

// simevu <lane> speed <kmph>
//
// Changes the simulated vehicle's speed WITHOUT restarting the track.
//
// Needed because V_FLOOR exists for a vehicle that stops MID-APPROACH,
// and there was no way to reach that state. Opening a track at zero
// speed does not work and should not: a stationary GPS reports no
// heading, so no approach can be associated and the track never becomes
// active. That is correct behaviour, but it meant the floor could not
// be exercised at all.
//
// A vehicle that was moving and then stops keeps its association --
// approach association is sticky once established -- so setting the
// speed on a LIVE track reaches the case that matters: an ambulance
// halted in traffic partway to the junction.
void gwSimEvuSpeed(int lane, float kmph) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    EvuSimState &v = evuSim[lane];
    if (!v.active) {
        Serial.printf("[SIMEVU] L%d no live vehicle -- 'approach' first\n", lane);
        return;
    }

    v.speedKmph = kmph;

    Serial.printf("[SIMEVU] L%d speed now %.0f km/h at %.0fm. The rolling mean "
                  "takes a few fixes to follow, which is the point -- run "
                  "'step' or 'run' to feed it.\n",
                  lane, kmph, v.distanceM);

    gwSimEvuEmit(lane);
}

void gwSimEvuClear(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    evuSim[lane] = EvuSimState();

    // Drop the ICU-side track too. Otherwise stopping the simulator
    // leaves a track the ICU keeps ageing into DEGRADED and then
    // EXPIRED, and the console shows a stale alert for a vehicle the
    // operator believes they removed.
    gwEvuDropTrack(gwSimIdFor(lane));
    Serial.printf("[SIMEVU] L%d simulated vehicle removed\n", lane);
}
