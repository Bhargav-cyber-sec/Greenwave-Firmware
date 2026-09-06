/***********************************************************************
 * ICU_Tracks.ino  --  PHASE 5B : ACOUSTIC TRACKS AND DIRECTION
 *
 * A new tab in the ICU sketch folder.
 *
 * This is where the ICU stops reasoning about SENSORS and starts
 * reasoning about VEHICLES. 5A established which node is FAR and which
 * is NEAR; this file uses that ordering to answer the only question
 * Scenario 1 can actually ask of a siren:
 *
 *          Is it coming towards us, or going away?
 *
 * ---------------------------------------------------------------------
 * WHY DIRECTION IS THE WHOLE PROBLEM
 *
 * In Scenario 1 there is no vehicle transmitter, no GPS, no camera, and
 * no sensor beyond the stop line. Two microphones and their timing are
 * the entire evidence base.
 *
 * A single microphone hearing a siren establishes almost nothing. It
 * cannot distinguish an approaching ambulance from a departing one, from
 * one on a parallel road, or from a recording played out of a car
 * window. The ONLY additional fact two nodes provide is the ORDER and
 * the INTERVAL in which they heard it.
 *
 * So direction inference is not a refinement here. It is the second
 * factor. Get it wrong and the system has one factor, which the
 * project's own IEEE paper concludes is not a sufficient basis for
 * preemption.
 *
 * ---------------------------------------------------------------------
 * THE TWO DEFECTS THIS FILE EXISTS TO FIX
 *
 * S1-01  v1.0 had the geometry INVERTED -- it treated the near node as
 *        the outer one. Every approach would have been read as a
 *        departure and every departure as an approach. Fixed in 5A by
 *        sorting on surveyed distance; this file consumes that ordering
 *        and never touches node_id.
 *
 * S1-03  v1.0 bounded the correlation window from ABOVE only: "both
 *        nodes within N seconds". A siren loud enough to reach both
 *        microphones at once produces a delta near zero, which satisfies
 *        any upper bound. So the single most likely false positive --
 *        one loud siren, two omnidirectional mics -- was not merely
 *        unhandled, it was classified as a confirmed approach.
 *
 *        Both bounds is the fix. Below t_min is not a fast vehicle; it
 *        is not a vehicle transiting between the nodes at all.
 *
 * ---------------------------------------------------------------------
 * THE CLOCK PROBLEM, AND WHY age_ms IS THE ANSWER
 *
 * Direction is a TIME DELTA between two independent boards that have no
 * synchronised clock. HBF_TIME_VALID is never set; the nodes have no
 * time source. Their millis() values are unrelated and comparing them
 * would be meaningless.
 *
 * Measuring the delta from FRAME ARRIVAL at the ICU does not work
 * either. Both nodes share one 434.5 MHz channel with heartbeats and
 * relays, and a frame can wait seconds for a clear window. At 60 km/h
 * across 100 m the true delta is about 6 s, so transport jitter of the
 * same order would not perturb the measurement -- it would DOMINATE it.
 * And the resulting error is not noise: it is a wrong direction, which
 * is the one thing being established.
 *
 * age_ms resolves this because it is a DURATION, not a timestamp:
 *
 *      detection_instant = (ICU arrival time) - age_ms
 *
 * Both terms are on the ICU's own clock. The node's clock never enters
 * the arithmetic, so no synchronisation is required. This is the entire
 * reason the field was added in Phase 1.
 ***********************************************************************/

#include "GreenwaveLink.h"

// From ICU_Geometry.ino (5A).
uint8_t  gwFarNode(int lane);
uint8_t  gwNearNode(int lane);
uint16_t gwSeparationM(int lane);
bool     gwApproachUsable(int lane);
uint8_t  gwNodeHealth(int lane, int node);
uint8_t  gwNodeRole(int lane, int node);
uint16_t gwNodeDistanceM(int lane, int node);

// =====================================================
// PLAUSIBLE VEHICLE SPEEDS
// =====================================================
//
// These two numbers ARE the correlation window. Nothing else sets it.
//
// [FIELD] Both want confirming against the site's actual speed profile.
// They bound what a vehicle could plausibly do between the two nodes,
// and they are deliberately WIDER than typical traffic: the window's job
// is to exclude the physically impossible, not the merely unusual.

// Fastest plausible. Sets t_min -- the floor below which a pair of
// detections cannot be one vehicle transiting between the nodes.
//
// Generous on purpose. An emergency vehicle on a clear road genuinely
// does exceed the posted limit, and t_min set too high would reject real
// approaches. It exists to catch deltas near ZERO, not to police speed.
#define V_MAX_KMPH   120.0f

// Slowest plausible. Sets t_max.
//
// Low on purpose, and this is the case the product exists for: an
// ambulance in the gridlock it was bought to clear may crawl the last
// 100 m. Set this too high and the window closes before the vehicle
// arrives -- failing precisely when it matters most.
#define V_MIN_KMPH   10.0f

// Detection is not instantaneous at a point. A siren becomes audible
// over a radius, and the classifier needs several windows to confirm, so
// each node's reported instant carries error of a second or more.
//
// Widening both bounds by this rather than narrowing them: a
// misclassified direction is worse than a wider window, because the
// window's failure mode is "ask a human", and direction's failure mode
// is "act on a departing vehicle".
#define DETECTION_SLOP_MS  2000UL

// How long a single-node observation waits for its partner before the
// track is resolved on what it has.
//
// Must exceed t_max for any plausible separation, or a slow vehicle's
// second detection arrives after the track has already been closed.
#define TRACK_PARTNER_WAIT_MS  45000UL

// Silence at the NEAR node before an established track is released.
//
// The primary release path in Scenario 1, because there is NO sensor
// past the stop line -- the ICU cannot observe the vehicle leaving
// (spec S1-07). All it can observe is the siren fading.
// Derived from the RDU's re-assert schedule, NOT chosen.
//
//   ACOUSTIC_REASSERT_MS = 8000   (RDU.ino, while a siren is audible)
//
// The old value was 12000, which is less than TWO re-assert intervals.
// Measured IF-2 delivery on this bench ran 83-100%, so a single dropped
// re-assert leaves a 16 s gap -- and 16 s exceeded the threshold.
//
// The consequence on a real approach: the ICU releases mid-event on one
// lost packet, while the vehicle is still coming. The 20 s approach
// re-lock then blocks re-alerting, so the console can go dark for over
// half a minute during a live emergency. The log would record
// SIREN_LOST, which is indistinguishable from the vehicle having
// genuinely passed -- so the failure would not even be visible
// afterwards.
//
// 25000 clears two missed re-asserts (16 s) plus heartbeat jitter and
// channel-busy deferral. Three consecutive losses still releases early,
// but three consecutive losses is a link that has failed, not noise.
//
// Erring long is the right direction here. Releasing late leaves an
// alert up a few seconds after the vehicle passed; releasing early
// abandons an approach mid-event.
//
// [FIELD] Re-derive if ACOUSTIC_REASSERT_MS changes. The relationship,
// not the number, is what must hold: at least 2x re-assert plus margin.
#define TRACK_SILENCE_RELEASE_MS  25000UL

// Re-trigger lockout after a track is served, so the tail of one
// vehicle's siren cannot immediately open a second track for the same
// vehicle.
#define TRACK_RETRIGGER_LOCKOUT_MS  20000UL

// =====================================================
// TRACK STATE
// =====================================================

enum AcousticTrackState : uint8_t {
    ATS_IDLE      = 0,
    ATS_TRACKING  = 1,   // one node has heard it; waiting for the partner
    ATS_CONFIRMED = 2,   // valid FAR -> NEAR inside the window
    ATS_AMBIGUOUS = 3,   // heard at both, delta below t_min: direction unknown
    ATS_OPPOSITE  = 4,   // NEAR -> FAR: departing, rejected
    ATS_DEGRADED  = 5,   // single surviving node, partner FAILED
    ATS_SERVED    = 6,   // acted upon; in re-trigger lockout
    ATS_EXPIRED   = 7
};

static const char* atsName(uint8_t s) {
    switch (s) {
        case ATS_TRACKING:  return "TRACKING";
        case ATS_CONFIRMED: return "CONFIRMED";
        case ATS_AMBIGUOUS: return "AMBIGUOUS";
        case ATS_OPPOSITE:  return "OPPOSITE";
        case ATS_DEGRADED:  return "DEGRADED";
        case ATS_SERVED:    return "SERVED";
        case ATS_EXPIRED:   return "EXPIRED";
        default:            return "IDLE";
    }
}

struct AcousticTrack {
    uint8_t  state = ATS_IDLE;

    // Detection instants, translated onto the ICU's clock by subtracting
    // age_ms from arrival.
    unsigned long farDetectMs  = 0;
    unsigned long nearDetectMs = 0;

    // EXPLICIT presence flags. Do NOT infer "not heard" from a zero
    // timestamp.
    //
    // A bench run caught exactly that collision: an observation whose
    // age_ms exceeded the ICU's own millis() produced a computed instant
    // of 0, which the old code read as the "never heard" sentinel and
    // silently discarded.
    //
    // Not a bench-only case. Reboot the ICU while a node is mid-episode
    // on a long siren and its first report legitimately carries an age
    // larger than the ICU's uptime. The FAR observation would vanish and
    // the approach could not confirm -- for the first minute after every
    // ICU restart, which is precisely when a blind spot is least
    // acceptable.
    bool farSeen  = false;
    bool nearSeen = false;

    // Separate from farSeen/nearSeen: whether an instant has been
    // recorded for the CURRENT episode. Keeping the first detection of
    // an episode rather than the latest is what stops the 8-second
    // re-assert stream dragging the far timestamp forward and shrinking
    // the measured delta toward zero.
    bool farSeenInstant  = false;
    bool nearSeenInstant = false;

    // Episode identifiers as reported by each node. Node-local, so the
    // two are unrelated -- they identify continuity AT A NODE, not
    // identity across nodes. Correlation across nodes is this file's job
    // and is done by time and geometry, never by matching these.
    uint32_t farEventId  = 0;
    uint32_t nearEventId = 0;

    float    farConfidence  = 0.0f;
    float    nearConfidence = 0.0f;
    bool     farSustained   = false;
    bool     nearSustained  = false;

    unsigned long firstSeenMs = 0;
    unsigned long lastSeenMs  = 0;    // any observation on this approach
    unsigned long lastNearMs  = 0;    // for the silence-release rule
    unsigned long servedAtMs  = 0;

    long     deltaMs   = 0;           // near - far. Negative = departing.
    uint32_t tMinMs    = 0;           // computed from separation
    uint32_t tMaxMs    = 0;

    uint8_t  evidence  = LE_NONE;
};

static AcousticTrack acoTrack[GW_NUM_APPROACHES + 1];

// =====================================================
// CORRELATION WINDOW
// =====================================================
//
// Derived per approach from the SURVEYED separation. Never a constant,
// because separation differs at every junction and a fixed window would
// silently mean a different thing at each one.

static void gwComputeWindow(int lane, uint32_t &tMinMs, uint32_t &tMaxMs) {
    uint16_t sep = gwSeparationM(lane);

    if (sep == 0) { tMinMs = 0; tMaxMs = 0; return; }

    float vMax = V_MAX_KMPH / 3.6f;      // m/s
    float vMin = V_MIN_KMPH / 3.6f;

    float tMin = (float)sep / vMax;      // seconds, fastest case
    float tMax = (float)sep / vMin;      // seconds, slowest case

    long lo = (long)(tMin * 1000.0f) - (long)DETECTION_SLOP_MS;

    // ------------------------------------------------------------
    // t_min COLLAPSING TO ZERO IS A SILENT LOSS OF PROTECTION.
    //
    // t_min before slop is separation / V_MAX. At V_MAX = 120 km/h that
    // is only 2 s at 67 m, so for any separation below roughly 67 m the
    // slop subtraction drives t_min to zero -- and a floor of zero
    // accepts everything, including the delta-near-zero case that t_min
    // exists specifically to catch.
    //
    // The protection would vanish exactly where it is needed most:
    // closely spaced nodes are the configuration where one siren most
    // easily reaches both microphones at once.
    //
    // Worse, it would vanish INVISIBLY. The window would still be
    // computed, still be logged, still look like a two-sided test.
    //
    // So a hard floor is enforced instead. A delta under half a second
    // is not a vehicle transiting between two nodes at any speed or any
    // spacing; it is one sound arriving at two microphones.
    // ------------------------------------------------------------
    #define T_MIN_ABSOLUTE_FLOOR_MS  500L

    bool floorApplied = (lo < T_MIN_ABSOLUTE_FLOOR_MS);
    if (floorApplied) lo = T_MIN_ABSOLUTE_FLOOR_MS;

    tMinMs = (uint32_t)lo;
    tMaxMs = (uint32_t)(tMax * 1000.0f) + DETECTION_SLOP_MS;

    // Warn once per approach when the geometry is too tight for the
    // timing method to discriminate well. Not an error -- the system
    // still works and still refuses to confirm on a near-zero delta --
    // but at this spacing most real approaches will land in AMBIGUOUS
    // rather than CONFIRMED, and that is a siting problem no firmware
    // can fix. Better said out loud than discovered as a low hit rate.
    // Warn ONLY when the absolute floor above actually had to intervene.
    //
    // The previous condition compared the raw t_min against 1.5x the
    // slop, which put the trip point at exactly 3.000 s -- precisely
    // where a 100 m separation lands at V_MAX. Float truncation
    // (2999.9998) tipped it, so a perfectly workable geometry warned on
    // every boot. A warning that fires on a good configuration is worse
    // than no warning: it teaches the reader to skip the line.
    //
    // Tying it to the floor makes it self-consistent -- it fires exactly
    // when the computed window was too narrow to express and had to be
    // widened artificially, which is the condition actually worth
    // reporting. For V_MAX = 120 km/h that is a separation under ~83 m.
    static bool warned[GW_NUM_APPROACHES + 1] = {false};
    if (lane >= 1 && lane <= GW_NUM_APPROACHES && !warned[lane] &&
        floorApplied) {
        warned[lane] = true;
        Serial.printf("[TRACK] L%d WARNING: separation %um is too small for the "
                      "timing method -- t_min had to be floored at %ldms. "
                      "Direction inference will often be AMBIGUOUS. "
                      "Consider wider node spacing.\n",
                      lane, (unsigned)sep, T_MIN_ABSOLUTE_FLOOR_MS);
    }
}

// =====================================================
// CLASSIFY
// =====================================================
//
// Called once both nodes have reported, or when the partner wait
// expires.

static void gwClassifyTrack(int lane) {
    AcousticTrack &t = acoTrack[lane];

    gwComputeWindow(lane, t.tMinMs, t.tMaxMs);

    bool haveFar  = t.farSeen;
    bool haveNear = t.nearSeen;

    // ---------- single node ----------
    if (haveFar != haveNear) {
        uint8_t farN  = gwFarNode(lane);
        uint8_t nearN = gwNearNode(lane);

        uint8_t partner = haveFar ? nearN : farN;
        uint8_t ph = (partner == 0) ? NH_UNKNOWN : gwNodeHealth(lane, partner);

        // DEGRADED authority is unlocked ONLY by a partner that is
        // genuinely FAILED.
        //
        // Not by SUSPECT, and this is the single most important line in
        // the file. IF-2 delivery is about 96%, so a briefly silent node
        // is routine. Under the old two-state model that routine event
        // promoted the survivor to degraded authority -- which LOWERS the
        // evidence bar. The system quietly relaxed its own safety
        // threshold several times an hour, and nothing logged it.
        //
        // It also closes an attack (spec S1-06). Jamming one node is far
        // easier than defeating authentication, and rewarding a jammer
        // with a lower evidence requirement at the surviving node makes
        // the cheap attack the effective one.
        // NH_FAILED ONLY. Not SUSPECT, and NOT LINK_SUSPECT.
        //
        // LINK_SUSPECT means the node went silent while the band
        // was noisy -- possibly jammed. Granting degraded authority
        // there would mean an attacker who jams one node of a pair
        // gets the surviving node promoted to a LOWER evidence
        // requirement, which is the opposite of what should happen
        // and is far cheaper than defeating authentication.
        //
        // The equality test already excludes it. Stated explicitly
        // because a later change to >= or a switch statement would
        // silently reintroduce the hole.
        if (ph == NH_FAILED) {
            t.state    = ATS_DEGRADED;

            // ASYMMETRIC BY DESIGN (spec S1-05). v1.0 treated "one node
            // down" as one situation; it is two, with opposite risk.
            //
            // FAR alone: something is out there, direction unknown, but
            // it is far away. Cheap to act on, and there is time.
            //
            // NEAR alone with no prior FAR: a siren appearing at the
            // inner node with nothing preceding it is more consistent
            // with a vehicle already past the junction, or one on a
            // crossing road, than with an approach. Acting on it means
            // acting on the case most likely to be a departure.
            t.evidence = haveFar ? LE_ACOUSTIC_DEGRADED : LE_NONE;

            Serial.printf("[TRACK] L%d DEGRADED %s-only (partner FAILED) -> %s\n",
                          lane, haveFar ? "FAR" : "NEAR",
                          haveFar ? "usable" : "NOT actionable");
            return;
        }

        // Partner healthy or merely suspect: keep waiting. Its SILENCE is
        // meaningful evidence that nothing has reached it yet.
        t.state    = ATS_TRACKING;
        t.evidence = LE_NONE;
        return;
    }

    if (!haveFar && !haveNear) {
        t.state    = ATS_IDLE;
        t.evidence = LE_NONE;
        return;
    }

    // ---------- both nodes reported ----------
    t.deltaMs = (long)t.nearDetectMs - (long)t.farDetectMs;

    // NEAR before FAR: the vehicle is moving away from the stop line.
    if (t.deltaMs < 0) {
        long mag = -t.deltaMs;

        if ((uint32_t)mag >= t.tMinMs) {
            t.state    = ATS_OPPOSITE;
            t.evidence = LE_NONE;
            Serial.printf("[TRACK] L%d OPPOSITE  near->far %lds  -- departing, rejected\n",
                          lane, mag / 1000);
            return;
        }

        // Magnitude below t_min: not a departure either, just two mics
        // hearing one source. Falls through to AMBIGUOUS.
        t.deltaMs = mag;
    }

    // ---------- BELOW t_min : THE FIX FOR S1-03 ----------
    //
    // A delta this small is not a fast vehicle. Nothing travelling
    // between the nodes can produce it. What produces it is ONE loud
    // siren reaching two omnidirectional microphones at nearly the same
    // moment -- which says the siren exists and says nothing whatever
    // about where it is going.
    //
    // v1.0 accepted this as a confirmed approach, because it bounded the
    // window from above only.
    //
    // Also the normal result of a bench setup with the nodes on one
    // table: correct behaviour, not a fault.
    if ((uint32_t)t.deltaMs < t.tMinMs) {
        t.state    = ATS_AMBIGUOUS;
        t.evidence = LE_ACOUSTIC_AMBIGUOUS;
        Serial.printf("[TRACK] L%d AMBIGUOUS delta=%ldms < tmin=%lums "
                      "-- one siren, two mics; direction UNKNOWN\n",
                      lane, t.deltaMs, (unsigned long)t.tMinMs);
        return;
    }

    // ---------- ABOVE t_max ----------
    //
    // Too slow to be one vehicle. Far more likely two separate events
    // that happen to be adjacent in time, and merging them would
    // attribute the second vehicle's arrival to the first one's timing.
    if ((uint32_t)t.deltaMs > t.tMaxMs) {
        t.state    = ATS_AMBIGUOUS;
        t.evidence = LE_ACOUSTIC_AMBIGUOUS;
        Serial.printf("[TRACK] L%d AMBIGUOUS delta=%ldms > tmax=%lums "
                      "-- too slow for one vehicle; likely two events\n",
                      lane, t.deltaMs, (unsigned long)t.tMaxMs);
        return;
    }

    // ---------- valid approach ----------
    t.state    = ATS_CONFIRMED;
    t.evidence = LE_ACOUSTIC_CONFIRMED;

    float sep   = (float)gwSeparationM(lane);
    float speed = (t.deltaMs > 0) ? (sep / ((float)t.deltaMs / 1000.0f)) * 3.6f : 0.0f;

    Serial.printf("[TRACK] L%d CONFIRMED far->near delta=%ldms "
                  "(window %lu..%lu) implied speed=%.0f km/h\n",
                  lane, t.deltaMs,
                  (unsigned long)t.tMinMs, (unsigned long)t.tMaxMs, speed);
}

// =====================================================
// INGEST AN ACOUSTIC OBSERVATION
// =====================================================

void gwTrackOnAcoustic(const AcousticEventFrame &f) {
    int lane = f.hdr.lane_id;
    int node = f.hdr.node_id;

    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    if (node < 1 || node > GW_NODES_PER_APPROACH) return;

    unsigned long now = millis();
    AcousticTrack &t = acoTrack[lane];

    // Re-trigger lockout: the fading tail of a served vehicle's siren
    // must not open a fresh track for the same vehicle.
    if (t.state == ATS_SERVED &&
        (now - t.servedAtMs) < TRACK_RETRIGGER_LOCKOUT_MS) {
        return;
    }

    uint8_t role = gwNodeRole(lane, node);

    // NO GEOMETRY, NO DIRECTION.
    //
    // Without a role this observation cannot be placed at either end of
    // the approach, and there is no safe default -- guessing is exactly
    // the inverted-geometry failure of v1.0. The detection is recorded
    // and the track stays unresolved.
    if (role == ROLE_UNKNOWN || !gwApproachUsable(lane)) {
        t.lastSeenMs = now;
        if (t.state == ATS_IDLE) {
            t.state       = ATS_TRACKING;
            t.firstSeenMs = now;
            Serial.printf("[TRACK] L%dN%d heard, but approach geometry is not "
                          "usable -- cannot infer direction\n", lane, node);
        }
        return;
    }

    // Translate the node's local detection instant onto the ICU's clock.
    // age_ms is a DURATION, so no clock synchronisation is needed.
    //
    // ------------------------------------------------------------
    // THE UPPER GUARD IS 65535, NOT 60000.
    //
    // An earlier version rejected any age at or above 60000 ms, which
    // silently discarded the age and collapsed the detection instant to
    // "now". A bench injection at exactly 60000 exposed it: the delta
    // came out as 0 and a clear approach was classified AMBIGUOUS.
    //
    // The consequence on real traffic is worse than on the bench.
    // age_ms measures time since the FIRST detection of a CONTINUOUS
    // siren episode, and a siren sounding for a minute while an
    // ambulance works through congestion is entirely ordinary. Under
    // the old guard the FAR node's age would be dropped, its detection
    // instant would jump forward to the present, the measured delta
    // would collapse toward zero, and a genuine approach would be
    // rejected as ambiguous.
    //
    // That failure would appear precisely during the long, slow events
    // this system exists to serve, and it would look like the acoustic
    // path simply not working rather than like a bug.
    //
    // 65535 is the only value that must be distrusted: the RDU
    // saturates there rather than wrapping, so it means "at least this
    // long, exact value unknown". Everything below it is a real
    // measurement and is used.
    // ------------------------------------------------------------
    unsigned long detectAt = now;

    if (f.age_ms == 65535) {
        // Saturated. The episode has run longer than the field can
        // express, so the instant is unknown. Fall back to arrival time
        // and let the delta arithmetic see it as a fresh observation --
        // conservative, because it can only push a classification
        // toward AMBIGUOUS, never toward a false CONFIRMED.
        Serial.printf("[TRACK] L%dN%d age_ms saturated -- episode longer than "
                      "65s, using arrival time\n", lane, node);
    } else if (f.age_ms > now) {
        // The detection predates ICU boot. Its instant cannot be placed
        // on our timeline at all -- there is no negative millis().
        //
        // Handled exactly like saturation: fall back to arrival time.
        // Conservative in the right direction, because it can only push
        // a classification toward AMBIGUOUS, never toward a false
        // CONFIRMED. The alternative -- clamping to 0 -- would fabricate
        // an enormous delta out of an unknown one.
        Serial.printf("[TRACK] L%dN%d detection predates ICU boot "
                      "(age=%ums, uptime=%lums) -- using arrival time\n",
                      lane, node, (unsigned)f.age_ms, (unsigned long)now);
    } else if (f.age_ms > 0) {
        detectAt = now - f.age_ms;
    }

    if (t.state == ATS_IDLE || t.state == ATS_EXPIRED ||
        t.state == ATS_SERVED) {
        // New episode on this approach.
        t = AcousticTrack();
        t.firstSeenMs = detectAt;
        t.state       = ATS_TRACKING;
    }

    t.lastSeenMs = now;

    float conf = f.confidence / 255.0f;
    bool  sust = (f.flags & ACF_SUSTAINED) != 0;

    if (role == ROLE_FAR) {
        t.farSeen = true;
        // Keep the FIRST detection of this episode, not the latest. The
        // node re-asserts every 8 s while a siren is audible, and taking
        // the newest would drag the far timestamp forward, shrinking the
        // measured delta towards zero -- turning a genuine approach into
        // an AMBIGUOUS one the longer the siren lasted.
        if (!t.farSeenInstant || f.event_id != t.farEventId) {
            t.farDetectMs   = detectAt;
            t.farEventId    = f.event_id;
            t.farSeenInstant = true;
        }
        t.farConfidence = conf;
        t.farSustained  = sust;
    } else {
        t.nearSeen = true;
        if (!t.nearSeenInstant || f.event_id != t.nearEventId) {
            t.nearDetectMs   = detectAt;
            t.nearEventId    = f.event_id;
            t.nearSeenInstant = true;
        }
        t.nearConfidence = conf;
        t.nearSustained  = sust;
        t.lastNearMs     = now;
    }

    Serial.printf("[TRACK] L%dN%d %s conf=%.2f sust=%d event=%lu age=%ums\n",
                  lane, node, gwRoleName(role), conf, sust ? 1 : 0,
                  (unsigned long)f.event_id, (unsigned)f.age_ms);

    gwClassifyTrack(lane);
}

// =====================================================
// PERIODIC MAINTENANCE
// =====================================================

void gwUpdateTracks() {
    unsigned long now = millis();

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        AcousticTrack &t = acoTrack[lane];

        if (t.state == ATS_IDLE) continue;

        // Lockout elapsed.
        if (t.state == ATS_SERVED) {
            if ((now - t.servedAtMs) >= TRACK_RETRIGGER_LOCKOUT_MS) {
                t.state = ATS_IDLE;
            }
            continue;
        }

        // Partner never arrived. Resolve on what we have rather than
        // waiting forever.
        if (t.state == ATS_TRACKING &&
            (now - t.firstSeenMs) > TRACK_PARTNER_WAIT_MS) {
            Serial.printf("[TRACK] L%d partner never reported in %lus -- expiring\n",
                          lane, TRACK_PARTNER_WAIT_MS / 1000);
            t.state    = ATS_EXPIRED;
            t.evidence = LE_NONE;
            continue;
        }

        // Siren gone from the NEAR node.
        //
        // The main release path in Scenario 1. There is no sensor past
        // the stop line, so the ICU cannot see the vehicle leave -- the
        // siren fading at the inner node is the only observable that the
        // event is over (spec S1-07).
        if ((t.state == ATS_CONFIRMED || t.state == ATS_DEGRADED) &&
            t.lastNearMs != 0 &&
            (now - t.lastNearMs) > TRACK_SILENCE_RELEASE_MS) {
            Serial.printf("[TRACK] L%d siren lost at NEAR for %lus -- releasing\n",
                          lane, (now - t.lastNearMs) / 1000);
            t.state      = ATS_SERVED;
            t.servedAtMs = now;
            t.evidence   = LE_NONE;
            continue;
        }

        // Nothing at all for a long time.
        if ((now - t.lastSeenMs) > TRACK_PARTNER_WAIT_MS) {
            t.state    = ATS_EXPIRED;
            t.evidence = LE_NONE;
        }
    }
}

// =====================================================
// QUERIES FOR 5C / 5E
// =====================================================

uint8_t gwAcousticEvidence(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return LE_NONE;
    return acoTrack[lane].evidence;
}

uint8_t gwAcousticState(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return ATS_IDLE;
    return acoTrack[lane].state;
}

float gwAcousticConfidence(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0.0f;
    AcousticTrack &t = acoTrack[lane];
    // The lower of the two, when both exist. An approach is only as well
    // evidenced as its weaker observation, and averaging would let a
    // confident far detection carry a marginal near one.
    if (t.farSeen && t.nearSeen)
        return (t.farConfidence < t.nearConfidence) ? t.farConfidence
                                                    : t.nearConfidence;
    return t.farSeen ? t.farConfidence : t.nearConfidence;
}

// Signed delta in ms for the current track. Positive = FAR then NEAR.
// Only meaningful when the state is CONFIRMED; callers must check.
long gwAcousticDeltaMs(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;
    return acoTrack[lane].deltaMs;
}

// Surveyed distance of the NEAR node to the stop line, in metres.
//
// Used by the ETA estimate. Returns 0 when the geometry is not
// established -- the caller must treat that as "no estimate possible"
// rather than as zero distance, which would read as "arriving now".
uint16_t gwNearDistanceM(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return 0;

    uint8_t n = gwNearNode(lane);
    if (n == 0) return 0;

    // The NEAR node is the closer of the two by definition, so its
    // distance is the smaller of the pair. Read it back from the
    // geometry layer rather than caching a copy here: a node re-flashed
    // with a new surveyed distance must change this immediately, not at
    // the next track.
    return gwNodeDistanceM(lane, n);
}

void gwMarkTrackServed(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    acoTrack[lane].state      = ATS_SERVED;
    acoTrack[lane].servedAtMs = millis();
}

// =====================================================
// DIAGNOSTICS
// =====================================================

void gwPrintTracks() {
    bool any = false;

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        if (acoTrack[lane].state == ATS_IDLE) continue;
        any = true;
        break;
    }

    if (!any) {
        Serial.println("[TRACKS] none active");
        return;
    }

    Serial.println("[TRACKS]");

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        AcousticTrack &t = acoTrack[lane];
        if (t.state == ATS_IDLE) continue;

        uint32_t lo, hi;
        gwComputeWindow(lane, lo, hi);

        Serial.printf("  L%d %-10s evidence=%-20s delta=%ldms window=%lu..%lums "
                      "far=%s near=%s conf=%.2f\n",
                      lane, atsName(t.state), gwEvidenceName(t.evidence),
                      t.deltaMs, (unsigned long)lo, (unsigned long)hi,
                      t.farSeen  ? "yes" : "no",
                      t.nearSeen ? "yes" : "no",
                      gwAcousticConfidence(lane));
    }
}


// =====================================================
// BENCH SIMULATION
// =====================================================
//
// Injects synthetic acoustic observations with an EXACT, chosen delta.
//
// WHY THIS IS NECESSARY, NOT A SHORTCUT
//
// Direction inference is a measurement of a time delta between two
// nodes 100 m apart. On a bench both RDUs sit on one desk wired to the
// same laptop, so every real siren reaches both microphones within a few
// milliseconds. That delta is genuinely ambiguous, the ICU correctly
// says AMBIGUOUS, and no amount of walking a phone around a desk will
// produce a trustworthy 6-second separation.
//
// Testing against a physical setup that cannot express the quantity
// being measured would mean testing speaker placement, not logic.
//
// WHAT MAKES THIS HONEST RATHER THAN A MOCK
//
// It calls gwTrackOnAcoustic() -- the same entry point a real
// authenticated frame reaches. The classification, the window
// arithmetic, the state machine, the FAR/NEAR lookup and the release
// timers are all the production paths. Only the origin of the two
// timestamps is synthetic.
//
// It also reaches cases hardware cannot produce on demand. Getting a
// real vehicle to depart at exactly 6 s, or two mics to disagree by
// exactly 60 s, means arranging traffic. Here it is one line, which
// means the rejection paths actually get exercised instead of being
// reasoned about and shipped.
//
// HOW THE DELTA IS CONSTRUCTED
//
// The ICU derives a detection instant as (arrival - age_ms). So sending
// the FAR observation with age_ms = D and the NEAR one with age_ms = 0,
// back to back, yields near - far = +D. Negative deltas swap the two.
//
// No clock manipulation and no special case in the track layer: this is
// precisely the arithmetic real frames go through.

void gwSimReset(int lane);

static uint32_t gwSimEventId = 0x51000000;   // distinct from real node ids

static void gwSimInject(int lane, int node, uint16_t ageMs,
                        float conf, bool sustained, uint32_t eventId) {
    AcousticEventFrame f;
    memset(&f, 0, sizeof(f));

    f.hdr.lane_id = (uint8_t)lane;
    f.hdr.node_id = (uint8_t)node;

    f.confidence = (uint8_t)(conf * 255.0f);
    f.flags      = ACF_MIC_OK | (sustained ? ACF_SUSTAINED : 0);
    f.event_id   = eventId;
    f.age_ms     = ageMs;

    gwTrackOnAcoustic(f);
}

// deltaMs > 0 : FAR first, then NEAR   -> approaching
// deltaMs < 0 : NEAR first, then FAR   -> departing
void gwSimApproach(int lane, long deltaMs, float conf, bool sustained) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) {
        Serial.printf("[SIM] lane %d out of range\n", lane);
        return;
    }

    uint8_t farN  = gwFarNode(lane);
    uint8_t nearN = gwNearNode(lane);

    if (farN == 0 || nearN == 0) {
        Serial.printf("[SIM] L%d has no usable geometry -- both nodes must be "
                      "reporting surveyed distances first\n", lane);
        return;
    }

    // age_ms is uint16 on the wire. A real node saturates rather than
    // wrapping, for exactly the reason noted in GreenwaveTypes.h, and the
    // simulation must not be able to express a delta a real node could
    // never report.
    // 65534, not 65535: the sentinel itself means "saturated, instant
    // unknown" and is handled separately in gwTrackOnAcoustic. The
    // simulation must not be able to express a value a real node would
    // never use as a measurement.
    long mag = (deltaMs < 0) ? -deltaMs : deltaMs;
    if (mag > 65534) {
        Serial.printf("[SIM] |delta| %ldms exceeds what age_ms can carry "
                      "(65534 max) -- clamping\n", mag);
        mag = 65534;
    }

    // A delta larger than the ICU's own uptime cannot be placed on its
    // timeline, so the injection would fall back to arrival time and the
    // measured delta would come out as ~0 -- producing AMBIGUOUS for the
    // wrong reason and looking like a logic failure.
    //
    // Said out loud rather than silently clamped: the operator asked for
    // a specific delta and needs to know they did not get it.
    unsigned long upNow = millis();
    if ((unsigned long)mag > upNow) {
        Serial.printf("[SIM] delta %ldms exceeds ICU uptime %lums -- the "
                      "detection would predate boot and cannot be placed. "
                      "Wait %lus and retry.\n",
                      mag, upNow, (unsigned long)((mag - upNow) / 1000 + 1));
        return;
    }

    gwSimEventId++;

    // Force a fresh track so a previous simulation cannot leak into this
    // one. Real episodes separate themselves by time; a bench operator
    // typing two commands ten seconds apart would otherwise be merging
    // them.
    gwSimReset(lane);

    if (deltaMs >= 0) {
        // FAR heard D ms ago, NEAR heard just now.
        gwSimInject(lane, farN,  (uint16_t)mag, conf, sustained, gwSimEventId);
        gwSimInject(lane, nearN, 0,             conf, sustained, gwSimEventId + 1);
    } else {
        // NEAR heard D ms ago, FAR heard just now: departing.
        gwSimInject(lane, nearN, (uint16_t)mag, conf, sustained, gwSimEventId);
        gwSimInject(lane, farN,  0,             conf, sustained, gwSimEventId + 1);
    }
}

// One node only. Used to exercise the degraded and partner-wait paths.
void gwSimSingle(int lane, bool useFar, float conf, bool sustained) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    uint8_t n = useFar ? gwFarNode(lane) : gwNearNode(lane);
    if (n == 0) {
        Serial.printf("[SIM] L%d geometry not established\n", lane);
        return;
    }

    gwSimEventId++;
    gwSimReset(lane);
    gwSimInject(lane, n, 0, conf, sustained, gwSimEventId);
}

// Keeps a simulated track alive by re-asserting at the NEAR node, the
// way a real RDU does while a siren remains audible.
//
// Needed because a single injection goes quiet immediately and the ICU
// correctly releases it as SIREN_LOST. Without this there is no way to
// observe a SUSTAINED alert on the bench -- and sustained behaviour is
// where the release timing, the ERC refresh path and the max-duration
// backstop actually live.
void gwSimReassert(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    AcousticTrack &t = acoTrack[lane];

    if (t.state != ATS_CONFIRMED && t.state != ATS_AMBIGUOUS &&
        t.state != ATS_DEGRADED) {
        Serial.printf("[SIM] L%d no live track to re-assert\n", lane);
        return;
    }

    uint8_t nearN = gwNearNode(lane);
    if (nearN == 0) return;

    // Same event_id as the current episode, so the track layer treats it
    // as continuation rather than a new vehicle -- and keeps the ORIGINAL
    // far/near detection instants, leaving the measured delta intact.
    gwSimInject(lane, nearN, 0, t.nearConfidence, true, t.nearEventId);

    Serial.printf("[SIM] L%d re-asserted at NEAR (track stays alive)\n", lane);
}

void gwSimReset(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    acoTrack[lane] = AcousticTrack();
}

// Print the computed window without injecting anything.
void gwPrintWindow(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    uint32_t lo, hi;
    gwComputeWindow(lane, lo, hi);

    uint16_t sep = gwSeparationM(lane);

    Serial.printf("[WINDOW] L%d sep=%um  tmin=%lums (%.1fs)  tmax=%lums (%.1fs)\n",
                  lane, (unsigned)sep,
                  (unsigned long)lo, lo / 1000.0f,
                  (unsigned long)hi, hi / 1000.0f);

    if (sep > 0) {
        Serial.printf("         a delta of %lums implies %.0f km/h; "
                      "%lums implies %.0f km/h\n",
                      (unsigned long)lo,
                      lo ? (sep / (lo / 1000.0f)) * 3.6f : 0.0f,
                      (unsigned long)hi,
                      hi ? (sep / (hi / 1000.0f)) * 3.6f : 0.0f);
    }
}
