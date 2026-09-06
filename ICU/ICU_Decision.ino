/***********************************************************************
 * ICU_Decision.ino  --  PHASE 5C / 5D / 5E
 *
 *   5C  LAYER 2 : reduce every track on an approach to ONE demand tuple
 *   5D  ETA     : estimate arrival where the evidence supports it
 *   5E  LAYER 3 : commitment staging, arbitration, release
 *
 * This is the last tab in the Scenario 1 chain. 5A established which
 * node is where. 5B established whether a siren is approaching. This
 * decides what to tell the officer, and when to stop telling them.
 *
 * ---------------------------------------------------------------------
 * WHY THE LAYERS ARE SEPARATE
 *
 * Spec Scenario 2 v2.0 section 12 requires that arbitration between
 * approaches never re-examine individual tracks. Layer 2 collapses an
 * approach to a tuple; layer 3 compares tuples and nothing else.
 *
 * The reason is not tidiness. In the automatic-control version of this
 * system, layer 3 is the function holding a green light. Every piece of
 * per-track detail that leaks into it is another way for one approach's
 * evidence to influence another approach's phase, and those interactions
 * are not reviewable once they exist. Keeping the interface to four
 * numbers means the arbitration logic can be read in full.
 *
 * The system is advisory today, which is precisely why the boundary is
 * cheap to hold now and expensive to retrofit later.
 ***********************************************************************/

#include "GreenwaveLink.h"

// From ICU_EVU.ino
bool    gwEvuDemand(int lane, uint8_t *priority, uint8_t *evidence, int16_t *etaS);
// From ICU_Safety.ino
bool    gwCommitAllowed(int lane);
void    ercQueue(const uint8_t *lanes, const uint8_t *prios, uint8_t count);
void    gwCommitRecord(int lane);
// From ICU_EVU.ino
float   gwEvuDistanceM(int lane);
bool    gwEvuIsReceding(int lane);
uint8_t gwEvuReleaseReason(int lane);

// From ICU_Geometry.ino
uint8_t  gwFarNode(int lane);
uint8_t  gwNearNode(int lane);
uint16_t gwSeparationM(int lane);
bool     gwApproachUsable(int lane);
uint8_t  gwNodeHealth(int lane, int node);

// From ICU_Tracks.ino
uint8_t gwAcousticEvidence(int lane);
uint8_t gwAcousticState(int lane);
float   gwAcousticConfidence(int lane);
long    gwAcousticDeltaMs(int lane);
uint16_t gwNearDistanceM(int lane);
void    gwMarkTrackServed(int lane);

// Track states, mirrored from ICU_Tracks.ino.
#define DEC_ATS_IDLE       0
#define DEC_ATS_TRACKING   1
#define DEC_ATS_CONFIRMED  2
#define DEC_ATS_AMBIGUOUS  3
#define DEC_ATS_OPPOSITE   4
#define DEC_ATS_DEGRADED   5
#define DEC_ATS_SERVED     6
#define DEC_ATS_EXPIRED    7

// =====================================================
// POLICY
// =====================================================

// Should an AMBIGUOUS acoustic track raise a PREPARE on the console?
//
// AMBIGUOUS means both nodes heard a siren but the delta was too small
// to be a vehicle transiting between them. Direction is genuinely
// unknown -- it could be approaching, departing, or on a crossing road.
//
// THE ARGUMENT FOR SHOWING IT (default)
// The cost is one line on a screen with no buzzer. The officer can look
// up and judge for themselves, which is the entire premise of advisory
// mode. Suppressing it means the system heard an emergency vehicle and
// deliberately told nobody.
//
// THE ARGUMENT AGAINST
// Alarm fatigue is the dominant failure mode of a device whose only
// output is a human's attention. A console that lights up for every
// siren within earshot gets ignored within a week, and then the
// CONFIRMED alerts are ignored too.
//
// The balance shifts with site geometry. At a junction near a hospital
// or on a siren-heavy corridor, set this to 0.
//
// It NEVER produces COMMIT. An unknown direction cannot justify asking
// anyone to stop cross traffic.
//
// [FIELD] Revisit per site once real ambiguous-event rates are known.
#define SHOW_AMBIGUOUS_AS_PREPARE  1

// May a single surviving node, with its partner FAILED, reach COMMIT?
//
// Spec decision D3, unresolved. The interim position is NO: degraded
// evidence raises PREPARE and stops there.
//
// The reasoning is that degraded mode must never become a way to LOWER
// the evidence bar. Disabling one node is easy -- a cut cable, a jammer,
// a dead battery -- and if that unlocked full commitment authority at
// the survivor, breaking one node would be an upgrade in what a single
// node can cause (spec S1-06).
//
// With a single node there is also no direction at all. The only thing
// keeping a departing vehicle from triggering a commitment is the
// two-node ordering, and in degraded mode it is gone.
#define ALLOW_DEGRADED_COMMIT  0

// Minimum classifier confidence for COMMIT.
//
// [FIELD] The RDU's own thresholds (strong >= 0.92, medium >= 0.80) were
// tuned in the lab. This is a second, independent gate at the ICU so
// that raising node sensitivity cannot silently raise commitment rates.
#define COMMIT_MIN_CONFIDENCE  0.70f

// An approach may not re-alert within this window of being released,
// preventing the fading tail of one vehicle's siren from immediately
// reopening the same event.
#define APPROACH_RELOCK_MS  20000UL

// Hard ceiling on a single alert, enforced here as well as on the ERC.
//
// Two independent limits on purpose. The console's backstop covers an
// ICU that hangs; this one covers a decision loop that is running but
// stuck in a state it cannot leave. Neither alone covers both.
#define MAX_ALERT_MS  300000UL

// =====================================================
// 5D : NEAR_ZONE URGENCY CLAMP  --  THE GRIDLOCK FIX
// =====================================================
//
// Inside this distance from the stop line a demand is treated as
// MAXIMALLY urgent regardless of its computed ETA.
//
// The problem it solves is specific and counter-intuitive. Urgency
// derived from ETA behaves BACKWARDS in exactly the conditions this
// product exists for:
//
//     ETA = remaining distance / speed
//
// An ambulance stopped in gridlock 40 m from the stop line has a
// speed near zero, so its ETA tends to infinity and its urgency
// collapses. A second vehicle moving freely 800 m away on another
// approach has a short ETA and wins the arbitration.
//
// So the vehicle that is closest, most stuck, and most in need of the
// junction being cleared is the one the system deprioritises -- and it
// stays deprioritised BECAUSE it is not moving, which is a self
// reinforcing trap. The worse the congestion, the less likely the
// system is to help.
//
// The clamp breaks it: once a vehicle is inside NEAR_ZONE, proximity
// alone establishes urgency. It is already at the junction; how long
// it takes to cover the last few metres is not the question.
//
// [FIELD] Should relate to the queue length on the approach. 80 m is
// a working default, not a measured one.
#define NEAR_ZONE_M  80.0f

// =====================================================
// 5D : ARBITRATION HYSTERESIS
// =====================================================
//
// A challenger must beat the incumbent by THIS MARGIN, not merely
// exceed it.
//
// Without it, two approaches with nearly equal scores swap the console
// back and forth on every re-evaluation. The inputs are noisy by
// nature -- confidence moves with each classifier window, ETA with each
// GPS fix -- so "nearly equal" is a normal state, not a rare one.
//
// The output of that flapping is an officer watching the display
// alternate between LEFT and BOTTOM every few seconds, which conveys
// less than showing either one steadily. A system that cannot make up
// its mind is worse than one that makes a slightly suboptimal choice
// and holds it.
//
// Sized just under one priority step (1000000), so it damps noise
// within a class without ever letting a lower-priority demand hold off
// a higher-priority one. A genuine Priority 1 arrival still displaces a
// Priority 2 incumbent immediately; two Priority 2 vehicles trading
// confidence do not.
//
// Note this only applies where preemption is permitted at all. A LIVE
// alert is never displaced mid-event regardless of margin -- see the
// no-preempt-of-preempt rule below.
#define ARBITRATION_HYSTERESIS  900000UL

// =====================================================
// DEMAND TUPLE  (LAYER 2)
// =====================================================

struct ApproachDemand {
    uint8_t  priority   = 0;      // registry class; 0 = unregistered
    uint8_t  evidence   = LE_NONE;
    float    confidence = 0.0f;
    int16_t  etaS       = -1;     // -1 = not computable
    uint8_t  stage      = LS_MONITOR;

    // Inside NEAR_ZONE_M of the stop line and still approaching.
    //
    // Carried in the tuple rather than recomputed during arbitration,
    // so layer 3 never reaches back into per-track state -- the
    // separation spec section 12 requires.
    bool     nearZone   = false;

    // Distance to the stop line in metres, 0 when unknown.
    //
    // Carried so arbitration can prefer the closer of two otherwise
    // equal demands. Previously nothing below NEAR_ZONE_M used distance
    // at all, so a stopped ambulance 100 m out tied with a moving one
    // 800 m away and the winner fell through to lane order.
    uint16_t distM      = 0;
};

static ApproachDemand demand[GW_NUM_APPROACHES + 1];

// Live alert bookkeeping.
static uint8_t       activeApproach   = 0;    // 0 = none
static unsigned long activeSinceMs    = 0;
static unsigned long lastReleaseMs[GW_NUM_APPROACHES + 1] = {0};
static uint8_t       lastSentStage[GW_NUM_APPROACHES + 1] = {0};

// Approach that won the previous arbitration pass. Carries hysteresis
// so noise in confidence or ETA cannot flap the winner.
static uint8_t       lastWinner = 0;

// =====================================================
// 5D : ETA
// =====================================================
//
// Only a CONFIRMED acoustic track supports an estimate, and it does so
// honestly: the delta between the two nodes and their surveyed
// separation give an implied speed, and the NEAR node's surveyed
// distance to the stop line is known. Both inputs are measurements.
//
//      implied speed  = separation / delta
//      eta            = near node distance / implied speed
//
// EVERYTHING ELSE RETURNS -1, WHICH RENDERS AS "--".
//
// That is not a gap to be filled later with something plausible. A
// single-node observation genuinely contains no distance information,
// and an AMBIGUOUS track contains no direction, so neither can produce
// an arrival time. The console shows "--" and the officer knows the
// system does not know.
//
// The estimate assumes the vehicle holds the speed it averaged between
// the nodes. It will not: it is heading into a junction and will
// decelerate. So the figure runs OPTIMISTIC -- the vehicle arrives later
// than predicted, not sooner. That is the correct direction for the
// error to point, because an officer acting early is safe and one acting
// late is not.
static int16_t gwComputeEta(int lane) {
    if (gwAcousticState(lane) != DEC_ATS_CONFIRMED) return -1;

    long delta = gwAcousticDeltaMs(lane);
    if (delta <= 0) return -1;

    uint16_t sep = gwSeparationM(lane);
    if (sep == 0) return -1;

    float speedMs = (float)sep / ((float)delta / 1000.0f);

    // A speed below walking pace means the delta is dominated by
    // detection uncertainty rather than by travel. Refuse rather than
    // publish a number built on noise.
    if (speedMs < 1.0f) return -1;

    uint16_t nearDist = gwNearDistanceM(lane);
    if (nearDist == 0 || nearDist == GW_DISTANCE_UNSET) return -1;

    float etaSec = (float)nearDist / speedMs;

    if (etaSec < 0.0f)    etaSec = 0.0f;
    if (etaSec > 3600.0f) return -1;      // implausible; say nothing

    return (int16_t)etaSec;
}

// =====================================================
// 5C : BUILD THE DEMAND TUPLE
// =====================================================

static void gwBuildDemand(int lane) {
    ApproachDemand &d = demand[lane];

    d = ApproachDemand();

    // ---------------------------------------------------------------
    // EVU FIRST. An authenticated vehicle outranks any acoustic
    // evidence on the same approach.
    //
    // Not because acoustic detection is unreliable, but because the two
    // answer different questions. Acoustic evidence says "a siren is
    // approaching". EVU evidence says "THIS vehicle, with THIS
    // registered priority, is at THIS position travelling at THIS
    // speed". When both are present the second strictly contains the
    // first.
    //
    // Spec Scenario 1 section 12: acoustic evidence ranks BELOW the
    // lowest EVU class. So an EVU track suppresses the acoustic path on
    // its approach entirely rather than being blended with it -- almost
    // certainly the same vehicle, and averaging a measurement with an
    // inference degrades the measurement.
    // ---------------------------------------------------------------

    uint8_t  evuPri;
    uint8_t  evuEvidence;
    int16_t  evuEta;

    if (gwEvuDemand(lane, &evuPri, &evuEvidence, &evuEta)) {
        d.priority   = evuPri;
        d.evidence   = evuEvidence;
        d.etaS       = evuEta;
        d.confidence = 1.0f;      // identity is proven, not inferred

        // DEGRADED means the track was authenticated and is now inside
        // its grace period with no fresh packet. Still actionable -- one
        // missed packet is not an ambulance vanishing -- but it does not
        // justify COMMIT on its own, because position is going stale and
        // position is what COMMIT timing depends on.
        d.stage = (evuEvidence == LE_EVU_DEGRADED) ? LS_PREPARE : LS_COMMIT;

        // DIRECTION CAPS THE STAGE.
        //
        // A vehicle moving away from the stop line may not reach COMMIT,
        // however strong its identity evidence is. Authentication proves
        // WHO it is; it says nothing about which way it is going, and
        // COMMIT is a request to stop cross traffic for something that
        // is arriving.
        //
        // The track is deliberately NOT dropped here. Below the
        // divergence confirmation count the vehicle may still turn back,
        // and discarding it would mean re-acquiring from scratch. The
        // demand stands at PREPARE until divergence either confirms or
        // clears.
        if (d.stage == LS_COMMIT && gwEvuIsReceding(lane)) {
            d.stage = LS_PREPARE;
            d.etaS  = -1;
        }

        // ---------------------------------------------------------
        // NEAR_ZONE CLAMP. See the note at NEAR_ZONE_M.
        //
        // Applied AFTER the receding check, deliberately. A vehicle
        // moving away is not urgent however close it is -- being
        // 30 m from the stop line and leaving is not an emergency,
        // and clamping it would resurrect the exact behaviour the
        // receding rule exists to prevent.
        // ---------------------------------------------------------
        float distM = gwEvuDistanceM(lane);

        d.distM = (distM > 0.0f && distM < 65535.0f)
                ? (uint16_t)distM : 0;

        if (d.stage != LS_MONITOR && !gwEvuIsReceding(lane) &&
            distM > 0.0f && distM <= NEAR_ZONE_M) {
            d.nearZone = true;

            // A stationary vehicle inside the zone has no honest
            // ETA, and the clamp is precisely the admission that
            // ETA has stopped being the right measure here.
            if (d.etaS < 0 || d.etaS > 60) d.etaS = -1;
        }

        return;
    }

    uint8_t st = gwAcousticState(lane);
    uint8_t ev = gwAcousticEvidence(lane);
    float   cf = gwAcousticConfidence(lane);

    d.evidence   = ev;
    d.confidence = cf;

    // Scenario 1 has no vehicle identity, so there is no registry entry
    // and no priority class. 0 renders as "?" on the console.
    //
    // Deliberately not defaulted to 1. An unclassified siren and a
    // registered Priority One ambulance must not look the same to the
    // operator, and inventing a class here would erase the distinction
    // that Scenario 2's registry exists to establish.
    d.priority = 0;

    switch (st) {

        case DEC_ATS_CONFIRMED:
            // Both nodes, correct order, inside the window.
            if (cf >= COMMIT_MIN_CONFIDENCE) {
                d.stage = LS_COMMIT;
            } else {
                // Direction is sound but the classifier was marginal.
                // PREPARE rather than COMMIT: the geometry says a vehicle
                // is approaching, the acoustics do not firmly say it is
                // an emergency one.
                d.stage = LS_PREPARE;
                Serial.printf("[DEMAND] L%d confirmed direction but confidence "
                              "%.2f < %.2f -- PREPARE only\n",
                              lane, cf, COMMIT_MIN_CONFIDENCE);
            }
            d.etaS = gwComputeEta(lane);
            break;

        case DEC_ATS_DEGRADED:
            // Single node, partner FAILED. evidence is LE_NONE when the
            // survivor is the NEAR node -- a siren appearing at the inner
            // node with nothing preceding it is more consistent with a
            // vehicle already past the junction than with an approach
            // (spec S1-05, asymmetric degraded policy).
            if (ev == LE_ACOUSTIC_DEGRADED) {
                d.stage = ALLOW_DEGRADED_COMMIT ? LS_COMMIT : LS_PREPARE;
            } else {
                d.stage = LS_MONITOR;
            }
            d.etaS = -1;          // one node carries no distance information
            break;

        case DEC_ATS_AMBIGUOUS:
#if SHOW_AMBIGUOUS_AS_PREPARE
            d.stage = LS_PREPARE;
#else
            d.stage = LS_MONITOR;
#endif
            d.etaS = -1;          // no direction, so no arrival time
            break;

        case DEC_ATS_TRACKING:
            // One node heard it, partner healthy and still silent. The
            // partner's silence is meaningful evidence that nothing has
            // reached it yet, so nothing is shown.
            d.stage = LS_MONITOR;
            break;

        case DEC_ATS_OPPOSITE:
            // Departing. Nothing is shown, and this is the case v1.0
            // would have preempted for.
            d.stage    = LS_MONITOR;
            d.evidence = LE_NONE;
            break;

        default:
            d.stage    = LS_MONITOR;
            d.evidence = LE_NONE;
            break;
    }
}

// =====================================================
// 5E : ARBITRATION AND STAGING
// =====================================================

// Rank two approaches. Higher wins.
//
// Ordering: stage first (COMMIT beats PREPARE), then evidence strength,
// then confidence. Priority class would lead in Scenario 2; in
// Scenario 1 every demand is unclassified, so it does not discriminate.
// Takes SCALARS, not the struct, deliberately.
//
// The Arduino IDE auto-generates prototypes and hoists them to the top
// of the sketch -- above the point where ApproachDemand is defined. A
// signature naming the struct therefore fails to compile with
// "'ApproachDemand' does not name a type", even though the definition
// precedes every real call.
//
// Passing scalars sidesteps the generator entirely. The alternative
// would be moving the struct into a shared header, which would put an
// ICU-internal type into a file the RDU and ERC also compile.
static uint32_t gwDemandScore(uint8_t stage, uint8_t evidence, float confidence,
                              bool nearZone, uint8_t priority, uint16_t distM) {
    if (stage == LS_MONITOR) return 0;

    uint32_t s = 0;

    // ---- stage dominates ----
    // A COMMIT always outranks a PREPARE. A confirmed arrival on one
    // approach matters more than a possible one on another, whatever
    // their relative priorities.
    s += (uint32_t)stage * 10000000UL;

    // ---- PRIORITY CLASS ----
    //
    // THIS WAS MISSING, AND ITS ABSENCE WAS A REAL DEFECT.
    //
    // The original comment here said priority "does not discriminate,
    // because in Scenario 1 every demand is unclassified". That was true
    // when only acoustic tracks existed. Once EVU tracks arrived with
    // real registry-derived classes, the field became meaningful and
    // this function was never revisited.
    //
    // The consequence: two approaches with the same stage and evidence
    // scored identically, so the winner was whichever lane the loop
    // reached first. A Priority 3 police unit on LEFT would beat a
    // Priority 1 ambulance on BOTTOM -- silently, and only when two
    // approaches were live at once, which is the case that had never
    // been tested.
    //
    // Lower number = higher priority, so it is inverted here. 0 means
    // unregistered or acoustic-only, which must rank BELOW every
    // registered class rather than above it -- spec Scenario 1 section
    // 12 puts acoustic evidence beneath the lowest EVU class.
    uint32_t priRank = (priority == 0) ? 0 : (10 - priority);
    s += priRank * 1000000UL;

    // ---- NEAR_ZONE ----
    //
    // Ranks below priority but above evidence. A closer vehicle of the
    // same class wins; it does not let a lower class jump the queue.
    // Proximity breaks ties within a class, it does not redefine them.

    // NEAR_ZONE outranks evidence and confidence.
    //
    // Proximity is the one input that cannot be wrong in the way
    // the others can: a vehicle 40 m from the stop line is 40 m
    // from the stop line, whatever its speed implies about ETA.
    if (nearZone) s += 100000UL;

    // Evidence enum runs strongest-to-weakest, so invert it.
    uint32_t evStrength = (evidence == LE_NONE) ? 0 : (10 - evidence);
    s += evStrength * 10000UL;

    // ---- PROXIMITY ----
    //
    // Below evidence, above confidence.
    //
    // A confirmed vehicle further away still outranks a doubtful one
    // nearby -- evidence quality is the more important question. But
    // between two demands of equal class and equal evidence, the closer
    // vehicle should win, and until now nothing below NEAR_ZONE_M made
    // that true. A stopped ambulance 100 m out tied with a moving one
    // 800 m away, and the winner fell through to lane order.
    //
    // This also removes the need for V_FLOOR, which was written to stop
    // a stationary vehicle's infinite ETA losing arbitration. That
    // problem never existed: this function has never read ETA. The real
    // gap was that it never read DISTANCE either -- and distance is the
    // honest measure, because it does not collapse when a vehicle stops.
    //
    // Range 0..200, so it cannot reach into the evidence band.
    if (distM > 0 && distM < 2000) {
        s += (uint32_t)((2000 - distM) / 10);
    }

    s += (uint32_t)(confidence * 100.0f);
    return s;
}

void gwUpdateDecision() {
    unsigned long now = millis();

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        gwBuildDemand(lane);
    }

    // ---- pick a winner ----
    int      best      = 0;
    uint32_t bestScore = 0;

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        // Re-lock: an approach just released cannot immediately re-alert
        // on the tail of the same siren.
        if (lastReleaseMs[lane] != 0 &&
            (now - lastReleaseMs[lane]) < APPROACH_RELOCK_MS) continue;

        uint32_t sc = gwDemandScore(demand[lane].stage,
                                    demand[lane].evidence,
                                    demand[lane].confidence,
                                    demand[lane].nearZone,
                                    demand[lane].priority,
                                    demand[lane].distM);

        // The incumbent, if any, carries hysteresis: a challenger must
        // clear it by a margin rather than merely exceed it.
        uint32_t effective = sc;

        if (lane == lastWinner && lastWinner != 0) {
            // Saturating add -- an incumbent near the top of the range
            // must not wrap to a tiny number and lose to everything.
            if (effective > 0xFFFFFFFFUL - ARBITRATION_HYSTERESIS) {
                effective = 0xFFFFFFFFUL;
            } else {
                effective += ARBITRATION_HYSTERESIS;
            }
        }

        if (effective > bestScore) { bestScore = effective; best = lane; }
    }

    // ---- an alert is already running ----
    if (activeApproach != 0) {
        ApproachDemand &d = demand[activeApproach];

        // Hard ceiling. Independent of the console's own backstop: this
        // one catches a decision loop that is running but wedged, which
        // the link timeout cannot see because the ICU is still PINGing.
        if ((now - activeSinceMs) > MAX_ALERT_MS) {
            Serial.printf("[DECISION] L%d exceeded MAX_ALERT_MS -- releasing\n",
                          activeApproach);
            ercRelease(activeApproach, LR_MAX_DURATION);
            gwMarkTrackServed(activeApproach);
            lastReleaseMs[activeApproach] = now;
            lastSentStage[activeApproach] = 0;
            activeApproach = 0;
            return;
        }

        uint8_t st = gwAcousticState(activeApproach);

        // Released by the track layer: siren lost at the near node,
        // expired, or served. Scenario 1 has no sensor past the stop
        // line, so siren loss at NEAR is the primary observable that the
        // event is over (spec S1-07).
        // ------------------------------------------------------------
        // THE ACOUSTIC TRACK STATE ONLY MATTERS IF ACOUSTIC EVIDENCE IS
        // WHAT IS DRIVING THIS ALERT.
        //
        // This condition used to consult gwAcousticState() regardless of
        // the demand's source. A bench run showed the consequence: lane
        // 1 held a stale acoustic track at EXPIRED from an earlier real
        // siren, and every EVU-driven alert on that approach was killed
        // the instant it opened --
        //
        //     [DECISION] L1 ALERT COMMIT evidence=EVU_INDIRECT eta=51
        //     [DECISION] L1 releasing (EXPIRED)
        //
        // four times in a row. Lane 3 survived only because it has no
        // RDUs, so its acoustic state was IDLE rather than EXPIRED.
        //
        // In the field this is worse than it looks: any approach whose
        // microphones had heard ANYTHING in the preceding minute would
        // refuse to hold an authenticated ambulance's alert. The busier
        // the approach, the more reliably it would fail -- and the
        // failure reads as EXPIRED, which is indistinguishable in the
        // log from the vehicle genuinely going away.
        //
        // A demand carries its own evidence class. If that class is an
        // EVU one, the acoustic layer has no standing to end it.
        // ------------------------------------------------------------
        bool evuDriven = (d.evidence == LE_EVU_DIRECT   ||
                          d.evidence == LE_EVU_INDIRECT ||
                          d.evidence == LE_EVU_DEGRADED);

        bool acousticEnded = !evuDriven &&
                             (st == DEC_ATS_SERVED || st == DEC_ATS_EXPIRED);

        if (d.stage == LS_MONITOR || acousticEnded) {

            // ASK THE EVU LAYER FIRST.
            //
            // This block previously consulted only the acoustic track
            // state, so an EVU divergence -- the vehicle turning away,
            // which the EVU layer detects and logs correctly -- came out
            // as EXPIRED on the console and in the record.
            //
            // The reason code is the only thing distinguishing "the
            // ambulance left" from "we lost it", and those say opposite
            // things about whether the system is working.
            uint8_t reason = gwEvuReleaseReason(activeApproach);

            if (reason == 0xFF) {
                reason = LR_EXPIRED;
                if (!evuDriven) {
                    if (st == DEC_ATS_SERVED)        reason = LR_SIREN_LOST;
                    else if (st == DEC_ATS_OPPOSITE) reason = LR_DIVERGED;
                }
            }

            Serial.printf("[DECISION] L%d releasing (%s)\n",
                          activeApproach, gwReleaseName(reason));

            ercRelease(activeApproach, reason);
            lastReleaseMs[activeApproach] = now;
            lastSentStage[activeApproach] = 0;
            activeApproach = 0;
            return;
        }

        // ---- 3.6: tell the console what is WAITING ----
        //
        // A live alert is never displaced, which is right -- but it means
        // a second emergency vehicle on another arm can be entirely
        // invisible to the officer. Listing the waiting approaches turns
        // "nothing else is happening" into "BOTTOM is queued behind
        // this", which is the difference between standing down and
        // staying ready.
        //
        // Ordered strongest first, so the top entry is the one that will
        // take the console when this alert releases.
        {
            uint8_t qLanes[GW_NUM_APPROACHES];
            uint8_t qPrios[GW_NUM_APPROACHES];
            uint8_t qCount = 0;

            for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
                if (lane == activeApproach) continue;
                if (demand[lane].stage == LS_MONITOR) continue;

                uint32_t sc = gwDemandScore(demand[lane].stage,
                                            demand[lane].evidence,
                                            demand[lane].confidence,
                                            demand[lane].nearZone,
                                            demand[lane].priority,
                                            demand[lane].distM);

                // Insertion sort, strongest first. At most two entries on
                // a three-way junction, so the cost is irrelevant and the
                // ordering matters more than the algorithm.
                uint8_t pos = qCount;
                for (uint8_t k = 0; k < qCount; k++) {
                    uint32_t other = gwDemandScore(demand[qLanes[k]].stage,
                                                   demand[qLanes[k]].evidence,
                                                   demand[qLanes[k]].confidence,
                                                   demand[qLanes[k]].nearZone,
                                                   demand[qLanes[k]].priority,
                                                   demand[qLanes[k]].distM);
                    if (sc > other) { pos = k; break; }
                }
                for (uint8_t k = qCount; k > pos; k--) {
                    qLanes[k] = qLanes[k-1];
                    qPrios[k] = qPrios[k-1];
                }
                qLanes[pos] = (uint8_t)lane;
                qPrios[pos] = demand[lane].priority;
                qCount++;
            }

            ercQueue(qLanes, qPrios, qCount);
        }

        // Still live. Refresh; ercAlert de-duplicates internally.
        ercAlert(activeApproach, d.stage, d.priority, d.evidence, d.etaS);

        // Escalating a LIVE alert into COMMIT is also a new
        // preemption and must be counted. Without this, an alert
        // that flickers PREPARE/COMMIT would bypass the limit
        // entirely by never passing through the start path above.
        if (d.stage == LS_COMMIT &&
            lastSentStage[activeApproach] != LS_COMMIT) {
            if (gwCommitAllowed(activeApproach)) {
                gwCommitRecord(activeApproach);
            } else {
                d.stage = LS_PREPARE;
            }
        }

        if (d.stage != lastSentStage[activeApproach]) {
            Serial.printf("[DECISION] L%d %s -> %s  evidence=%s eta=%d\n",
                          activeApproach,
                          gwStageName(lastSentStage[activeApproach]),
                          gwStageName(d.stage),
                          gwEvidenceName(d.evidence), (int)d.etaS);
            lastSentStage[activeApproach] = d.stage;
        }

        // PREEMPTION OF A LIVE ALERT IS NOT PERMITTED.
        //
        // A stronger demand on another approach waits. Swapping the
        // console mid-event would leave an officer who has already begun
        // an override for one road looking at a different one, with no
        // indication that the first vehicle is still coming.
        //
        // The waiting approach is not lost: it is re-evaluated the moment
        // this one releases, and MAX_ALERT_MS bounds how long that can
        // take.
        return;
    }

    // ---- nothing running: start one ----
    lastWinner = (uint8_t)best;

    // No alert running: nothing can be waiting behind one.
    if (activeApproach == 0) {
        ercQueue(nullptr, nullptr, 0);
    }

    if (best != 0 && demand[best].stage != LS_MONITOR) {
        ApproachDemand &d = demand[best];

        // ---- 5E: PREEMPTION RATE LIMIT ----
        //
        // Applied at the TRANSITION into COMMIT, never on refresh,
        // or a single long event would exhaust the hourly budget in
        // seconds and then release itself.
        //
        // A blocked demand is demoted to PREPARE, not discarded.
        // The vehicle may well be genuine -- the limit says the
        // approach has already had more preemptions this hour than
        // the authority permits, which is a statement about the
        // approach, not about this vehicle. Keeping it on screen
        // lets the officer judge; suppressing it entirely would
        // hide the abuse from the only person who can act on it.
        if (d.stage == LS_COMMIT) {
            if (gwCommitAllowed(best)) {
                gwCommitRecord(best);
            } else {
                d.stage = LS_PREPARE;
            }
        }

        activeApproach = best;
        activeSinceMs  = now;
        lastSentStage[best] = d.stage;

        Serial.printf("[DECISION] L%d ALERT %s evidence=%s conf=%.2f eta=%d\n",
                      best, gwStageName(d.stage), gwEvidenceName(d.evidence),
                      d.confidence, (int)d.etaS);

        ercAlert(best, d.stage, d.priority, d.evidence, d.etaS);
    }
}

// =====================================================
// DIAGNOSTICS
// =====================================================

void gwPrintDemand() {
    Serial.printf("[DECISION] active=%s\n",
                  activeApproach ? gwErcApproachName(activeApproach) : "none");

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        ApproachDemand &d = demand[lane];
        if (d.stage == LS_MONITOR && d.evidence == LE_NONE) continue;

        Serial.printf("  L%d %-9s %-20s conf=%.2f eta=%d score=%lu\n",
                      lane, gwStageName(d.stage), gwEvidenceName(d.evidence),
                      d.confidence, (int)d.etaS,
                      (unsigned long)gwDemandScore(d.stage, d.evidence,
                                                   d.confidence, d.nearZone,
                                                   d.priority, d.distM));
    }
}
