#ifndef GREENWAVE_LINK_H
#define GREENWAVE_LINK_H

/***********************************************************************
 * GREENWAVE LINK  --  ICU <-> ERC TEXT PROTOCOL  v1
 *
 * CANONICAL DEFINITION. Copy byte-identical into BOTH the ICU sketch
 * folder and the ERC sketch folder.
 *
 * WHY A SEPARATE HEADER FROM GreenwaveTypes.h
 *
 * GreenwaveTypes.h is the RADIO protocol: packed binary structs,
 * authenticated, size-locked, and shared by the ICU and the RDUs. This
 * file is the CONSOLE protocol: newline-terminated ASCII over a short
 * point-to-point UART inside one cabinet.
 *
 * They are deliberately not the same thing. The radio link is hostile --
 * anyone with a LoRa module can transmit on 434.5 MHz -- so every frame
 * there is authenticated. The ERC link is a wire between two boards in a
 * locked enclosure; an attacker who can reach it can also reach the
 * boards themselves, so a MAC on it would buy nothing.
 *
 * ASCII is the right choice here for a reason that matters in the field:
 * you can plug a USB-serial adapter onto this link and read the entire
 * decision stream in a terminal, with no decoder and no ICU. During
 * commissioning that is the difference between "the console is not
 * lighting up" and knowing which of the two boards is at fault.
 *
 * ---------------------------------------------------------------------
 * WIRING  (from the working link, do not change)
 *
 *   ICU UART2                        ERC UART1
 *   GPIO14 (TX)  ---------------->   GPIO34 (RX)
 *   GPIO15 (RX)  <----------------   GPIO15 (TX)
 *   GND          -----------------   GND
 *
 *   9600 8N1 both ends.
 *
 * 9600 is slow, and it is retained on purpose: it is the rate the
 * existing link was proven at. The busiest second in this protocol is
 * one PING plus one ALERT plus a few HEALTH lines -- roughly 150 bytes,
 * about 16% of what 9600 baud carries. There is no reason to touch a
 * working physical layer for headroom that is not needed.
 *
 * ---------------------------------------------------------------------
 * FORMAT
 *
 *   KEYWORD:field:field:field\n
 *
 * Uppercase keywords. Colon separated. Newline terminated. The ERC
 * upper-cases every incoming line EXCEPT those beginning NET:, WIFI: or
 * DATE:, because SSIDs and passwords are case sensitive. Any new command
 * carrying free text must be added to that exemption list in ERC.ino.
 *
 * Unknown keywords are IGNORED, not errors. This is what lets an ICU and
 * an ERC on different firmware versions still work for everything they
 * do share -- during an upgrade you will inevitably have one of each.
 ***********************************************************************/

#include <Arduino.h>

#define GW_LINK_VERSION 1

/***********************************************************************
 * TIMING
 ***********************************************************************/

// The ICU sends PING at this rate whenever it is alive, regardless of
// whether anything is happening. Silence is the failure signal, so
// silence must never be a normal state.
#define GW_PING_INTERVAL_MS   1000UL

// The ERC declares the link dead after this long with no traffic at all,
// and RETURNS ITSELF TO NORMAL. Five PING intervals.
//
// THIS IS THE PRIMARY FAIL-SAFE OF THE WHOLE ADVISORY DESIGN.
//
// Spec Scenario 1 v2.0 section 11.1 requires the ICU-to-controller
// interface to be a HEARTBEAT-GATED HOLD rather than a latching command:
// an ICU that raises an alert and then hangs must not leave that alert
// standing. Neither v1.0 document said what happens if the ICU crashes
// mid-event, and it is the first question a safety reviewer asks.
//
// In advisory mode the consequence is milder than a stuck green -- a
// console showing a stale alert, and an officer holding an override for
// an ambulance that passed ten minutes ago -- but the principle is
// identical and the fix costs nothing: the ERC decides for itself when
// the ICU has stopped talking. It does not wait to be told.
//
// The old ERC value was 25000 ms, sized when nothing arrived on a quiet
// link and 25 s of silence was normal. With PING at 1 Hz, silence is
// never normal, so this tightens to 5000.
#define GW_LINK_TIMEOUT_MS    5000UL

/***********************************************************************
 * ICU -> ERC COMMANDS
 *
 * Existing commands kept working unchanged, so a v1 ERC still responds
 * to a v2 ICU during a partial upgrade:
 *
 *   LEFT / RIGHT / BOTTOM / CENTER   start an alert on that approach
 *   CLEAR / NORMAL                   return to idle
 *   ACK                              remote acknowledge
 *   NODE:<APPR>:<FAR|NEAR>:<0|1>     drive one indicator LED directly
 *   TIME:HH:MM:SS  DATE:14 MAY 2026  clock push (ERC has no RTC)
 *   WIFI:... NET:...                 network status and scan results
 *
 * New in v1 of this protocol:
 ***********************************************************************/

// PING:<seq>
//   Liveness, 1 Hz. seq lets the ERC count losses on the wire rather
//   than only noticing total silence -- a link at 70% delivery still
//   never trips the timeout, but it is on its way to failing and you
//   want to know before it does.
#define GW_CMD_PING     "PING"

// ALERT:<APPROACH>:<STAGE>:<PRIORITY>:<EVIDENCE>:<ETA_S>
//   The full decision, in one line. Sent on entry to a stage and on any
//   change of the fields.
//
//   APPROACH  LEFT | RIGHT | BOTTOM
//   STAGE     MONITOR | PREPARE | COMMIT | CLEARING
//   PRIORITY  1..9, or 0 for unknown/unregistered
//   EVIDENCE  see LinkEvidence below
//   ETA_S     seconds until arrival, or -1 when not computable
//
//   EVIDENCE is on the wire, not just internal, because it is what tells
//   the officer how far to trust the alert. "Authenticated ambulance
//   1.2 km out" and "a siren that one working microphone heard once"
//   both produce a red screen; only one of them justifies stopping cross
//   traffic. Spec Scenario 1 section 12 is explicit that acoustic-only
//   evidence is weaker than the project's own published fusion rule
//   requires, so hiding that distinction from the operator would be
//   hiding the system's principal known weakness.
#define GW_CMD_ALERT    "ALERT"

// HOLD:<APPROACH>
//   Keep the current alert live. Refreshes it; changes nothing else.
//
//   This exists because the ERC today runs a FIXED 34 s sequence once
//   started, which the ICU can neither extend nor end. Every release
//   rule in the spec -- siren loss, the EVU emergency flag dropping,
//   divergence, stop-line crossing -- needs the ICU to be able to say
//   "still coming" or "done now". A fixed timer can express neither, and
//   an ambulance stuck in the gridlock this product exists to clear will
//   routinely take longer than 34 s to arrive.
#define GW_CMD_HOLD     "HOLD"

// RELEASE:<APPROACH>:<REASON>
//   End the alert. REASON is displayed and logged; see LinkRelease.
//
//   The reason travels because "the ambulance passed" and "we gave up
//   waiting" look identical on screen but mean opposite things about
//   whether the system is working. Without it, a site with a failing
//   microphone that quietly times out every event is indistinguishable
//   from a site running perfectly.
#define GW_CMD_RELEASE  "RELEASE"

// HEALTH:<LANE>:<NODE>:<ROLE>:<STATE>:<DIST_M>
//   Per-node status, pushed on change and periodically.
//
//   ROLE is FAR | NEAR | UNKNOWN, derived by the ICU from the surveyed
//   distances the nodes report -- never from node_id. DIST_M is that
//   surveyed distance, echoed back so a commissioning engineer can see
//   on the console itself that the geometry the ICU is reasoning with
//   matches the road. Inverted geometry is otherwise invisible: the
//   system simply preempts for departing traffic and ignores arrivals,
//   and nothing in any log looks wrong (spec defect S1-01).
#define GW_CMD_HEALTH   "HEALTH"

// QUEUE:<n>[:<APPROACH>:<PRIORITY>]...
//
// The approaches that have a live demand but are NOT being displayed,
// because another approach holds the console.
//
// WHY THE OPERATOR NEEDS THIS
//
// A live alert is never displaced mid-event -- swapping the display out
// from under an officer who has begun an override would be worse than
// making them wait. But the consequence is that a second emergency
// vehicle can be approaching a different arm of the same junction and
// be COMPLETELY INVISIBLE to the only person who could act on it.
//
// A bench run showed exactly that: LEFT held the console while BOTTOM
// carried a live, higher-priority, authenticated demand, and nothing on
// the screen indicated BOTTOM existed. The officer would clear LEFT,
// stand down, and only then discover a second vehicle had been waiting.
//
// n = 0 clears the display. Sent on change only.
#define GW_CMD_QUEUE    "QUEUE"

/***********************************************************************
 * ERC -> ICU REPLIES
 *
 * Existing, unchanged:
 *   RECEIVED:<cmd>   ACK   ACKNOWLEDGED   ACK_IGNORED   AUTO_ACK
 *   CLEARED   NORMAL_MODE   SEQUENCE_COMPLETE
 *   NODE_OK <cmd>   NODE_BAD   NODE_BUSY
 ***********************************************************************/

// PONG:<seq>
//   Echoes the sequence number from PING. The ICU can then measure
//   round-trip health of the link in BOTH directions. ICU-to-ERC health
//   alone is not enough: the ERC's ACK is the only evidence that a human
//   ever saw an alert, and it travels on the return path.
#define GW_RPY_PONG     "PONG"

/***********************************************************************
 * STAGE  (spec Scenario 1 v2.0 section 7)
 *
 * v1.0 had one binary outcome: preempt or do not. That forced every
 * piece of evidence to be judged against the cost of the most disruptive
 * possible action, and it threw away the entire lead time that the outer
 * node exists to provide -- under v1.0 an RDU_FAR detection produced no
 * action of any kind.
 *
 * Splitting the action is what lets weak evidence still be useful:
 * PREPARE is cheap and reversible, so it can act on a detection that
 * would never justify COMMIT.
 ***********************************************************************/
enum LinkStage : uint8_t {
    LS_MONITOR  = 0,   // tracking only. Nothing on screen, no buzzer.
    LS_PREPARE  = 1,   // probable approach. FAR indicator, no buzzer.
                       // The officer is informed, not yet asked to act.
    LS_COMMIT   = 2,   // confirmed. NEAR indicator, buzzer, override now.
    LS_CLEARING = 3    // released, winding down.
};

inline const char* gwStageName(uint8_t s) {
    switch (s) {
        case LS_MONITOR:  return "MONITOR";
        case LS_PREPARE:  return "PREPARE";
        case LS_COMMIT:   return "COMMIT";
        case LS_CLEARING: return "CLEARING";
        default:          return "MONITOR";
    }
}

/***********************************************************************
 * EVIDENCE CLASS  (spec Scenario 2 v2.0 section 12, layer 2)
 *
 * Ordered strongest to weakest. The ICU reduces every track on an
 * approach to one of these before layer 3 acts, so the commitment layer
 * applies policy WITHOUT re-reading individual tracks.
 ***********************************************************************/
enum LinkEvidence : uint8_t {
    LE_NONE                 = 0,

    // Authenticated EVU received directly by the ICU. Strongest.
    // Identity is proven. Note that identity being proven does NOT make
    // the vehicle's claims true -- priority comes from the ICU's own
    // registry, and position is plausibility-checked, because a genuine
    // signed packet can still carry a lie (spec defects S2-01, S2-05).
    LE_EVU_DIRECT           = 1,

    // Authenticated, but only reached the ICU relayed via an RDU. Valid,
    // and subject to extra plausibility checks: the ICU never saw the
    // transmission itself.
    LE_EVU_INDIRECT         = 2,

    // Was authenticated, now inside its grace period with no fresh
    // packet. Degraded, not invalid -- one missed packet is not an
    // ambulance vanishing.
    LE_EVU_DEGRADED         = 3,

    // No EVU. Both nodes healthy, valid FAR -> NEAR progression inside
    // the correlation window. This is normal-mode acoustic confirmation.
    //
    // Weaker than it looks: two-node temporal progression is a weaker
    // second factor than the visual confirmation the project's IEEE
    // paper assumes. It defeats a stationary spoofer playing a siren
    // recording; it does not defeat a moving one.
    LE_ACOUSTIC_CONFIRMED   = 4,

    // Single surviving node, partner FAILED. Weakest actionable class.
    //
    // Deliberately last: disabling one node must not become a way to
    // LOWER the evidence bar. Jamming a radio is far easier than
    // defeating authentication, so degraded mode was the softest target
    // in the v1.0 design (spec S1-06).
    LE_ACOUSTIC_DEGRADED    = 5,

    // Heard at both nodes too close together in time to be a vehicle
    // travelling between them -- one loud siren reaching two
    // omnidirectional microphones. Direction is UNKNOWN.
    //
    // Not a direction, and never treated as one. Under v1.0 this
    // satisfied "detected at both nodes" and was accepted as a valid
    // progression, because only an upper time bound was enforced
    // (spec S1-03).
    LE_ACOUSTIC_AMBIGUOUS   = 6
};

inline const char* gwEvidenceName(uint8_t e) {
    switch (e) {
        case LE_EVU_DIRECT:         return "EVU_DIRECT";
        case LE_EVU_INDIRECT:       return "EVU_INDIRECT";
        case LE_EVU_DEGRADED:       return "EVU_DEGRADED";
        case LE_ACOUSTIC_CONFIRMED: return "ACOUSTIC_CONFIRMED";
        case LE_ACOUSTIC_DEGRADED:  return "ACOUSTIC_DEGRADED";
        case LE_ACOUSTIC_AMBIGUOUS: return "ACOUSTIC_AMBIGUOUS";
        default:                    return "NONE";
    }
}

// Short form for the console, which has roughly 12 characters of width
// for this field. Same information, no line wrap.
inline const char* gwEvidenceShort(uint8_t e) {
    switch (e) {
        case LE_EVU_DIRECT:         return "EVU";
        case LE_EVU_INDIRECT:       return "EVU-RLY";
        case LE_EVU_DEGRADED:       return "EVU-DEG";
        case LE_ACOUSTIC_CONFIRMED: return "SIREN 2/2";
        case LE_ACOUSTIC_DEGRADED:  return "SIREN 1/2";
        case LE_ACOUSTIC_AMBIGUOUS: return "SIREN ?";
        default:                    return "-";
    }
}

/***********************************************************************
 * RELEASE REASON  (spec Scenario 2 v2.0 section 10.1)
 ***********************************************************************/
enum LinkRelease : uint8_t {
    // The EVU's emergency flag went from on to off. The driver switched
    // the siren off, which is an explicit, immediate, authenticated
    // "release now" from the only party who actually knows.
    //
    // The cheapest and most reliable release signal in the entire
    // system, and v1.0 discarded it completely (spec S2-08).
    LR_FLAG_OFF     = 0,

    // Position crossed the stop line AND remaining distance grew across
    // several consecutive fixes. Both halves are required: crossing
    // alone is defeated by ordinary GPS jitter at the line.
    LR_PASSED       = 1,

    // Siren no longer heard at RDU_NEAR for a sustained hold-off.
    // Primary release path when there is no EVU, because Scenario 1 has
    // no position telemetry and no sensor past the stop line -- the ICU
    // genuinely cannot observe progression (spec S1-07).
    LR_SIREN_LOST   = 2,

    // Heading reversed AND remaining distance grew monotonically. The
    // vehicle turned away. Both conditions required, or normal GPS
    // jitter at low speed declares divergence on its own.
    LR_DIVERGED     = 3,

    // No fresh evidence of any kind within the grace period.
    LR_EXPIRED      = 4,

    // Hard cap hit. A backstop, enforced independently of the decision
    // logic and never disabled. If this fires, something upstream is
    // wrong -- it is a diagnostic, not a normal ending.
    LR_MAX_DURATION = 5,

    // An operator cleared it at the console. Human authority wins.
    LR_OPERATOR     = 6,

    // ICU is shutting down, rebooting, or has lost confidence in its own
    // state. Fail to normal, loudly.
    LR_FAILSAFE     = 7
};

inline const char* gwReleaseName(uint8_t r) {
    switch (r) {
        case LR_FLAG_OFF:     return "FLAG_OFF";
        case LR_PASSED:       return "PASSED";
        case LR_SIREN_LOST:   return "SIREN_LOST";
        case LR_DIVERGED:     return "DIVERGED";
        case LR_EXPIRED:      return "EXPIRED";
        case LR_MAX_DURATION: return "MAX_DURATION";
        case LR_OPERATOR:     return "OPERATOR";
        case LR_FAILSAFE:     return "FAILSAFE";
        default:              return "EXPIRED";
    }
}

/***********************************************************************
 * APPROACH NAMING
 *
 * The ICU thinks in lane numbers. The ERC and the TCU each think in
 * names, and THEY DO NOT AGREE WITH EACH OTHER:
 *
 *      lane 1  ->  ERC "LEFT"      TCU "LEFT"
 *      lane 2  ->  ERC "BOTTOM"    TCU "CENTER"     <-- different
 *      lane 3  ->  ERC "RIGHT"     TCU "RIGHT"
 *
 * That disagreement is inherited from the working model and is retained
 * deliberately: renaming either end to make them match would require
 * re-testing a link that currently works.
 *
 * It is also exactly the kind of mapping that gets silently duplicated
 * into three places and then edited in two of them. So it lives here,
 * once, and everything calls these functions. Sending an alert for the
 * wrong approach is not a cosmetic bug -- it points an officer at the
 * wrong road while an ambulance approaches on another.
 *
 * The ERC also accepts CENTER as an alias for BOTTOM, which is why a
 * TCU-shaped name reaching the console does not fail loudly. Convenient,
 * and the reason this mapping must be got right in code rather than
 * relied upon to blow up when wrong.
 ***********************************************************************/
inline const char* gwErcApproachName(int lane) {
    switch (lane) {
        case 1: return "LEFT";
        case 2: return "BOTTOM";
        case 3: return "RIGHT";
        default: return nullptr;   // caller must check; never guess a lane
    }
}

inline const char* gwTcuApproachName(int lane) {
    switch (lane) {
        case 1: return "LEFT";
        case 2: return "CENTER";
        case 3: return "RIGHT";
        default: return nullptr;
    }
}

// Reverse lookup, for replies that name an approach rather than a lane.
// Accepts CENTER as an alias for BOTTOM so a TCU-style name arriving
// from the console maps to the same lane rather than to nothing.
inline int gwLaneFromErcName(const char* name) {
    if (name == nullptr) return 0;
    if (strcasecmp(name, "LEFT")   == 0) return 1;
    if (strcasecmp(name, "BOTTOM") == 0) return 2;
    if (strcasecmp(name, "CENTER") == 0) return 2;
    if (strcasecmp(name, "RIGHT")  == 0) return 3;
    return 0;                      // 0 means "no such approach"
}

/***********************************************************************
 * NODE ROLE NAMES  (for HEALTH lines)
 *
 * Mirrors NodeRole in GreenwaveTypes.h. Duplicated as strings here so
 * the ERC, which never includes the binary protocol header, can still
 * display roles without pulling in packed structs and static_asserts it
 * has no use for.
 ***********************************************************************/
inline const char* gwRoleName(uint8_t role) {
    switch (role) {
        case 1:  return "FAR";
        case 2:  return "NEAR";
        default: return "UNKNOWN";
    }
}

inline const char* gwHealthName(uint8_t h) {
    switch (h) {
        case 1:  return "HEALTHY";
        case 2:  return "SUSPECT";
        case 3:  return "FAILED";
        case 4:  return "RECOVERING";
        case 5:  return "MISCONFIG";
        case 6:  return "LINK_SUSPECT";
        default: return "UNKNOWN";
    }
}

/***********************************************************************
 * LINE LENGTH
 *
 * The longest defined line is an ALERT with the longest approach name,
 * stage and evidence class:
 *   "ALERT:BOTTOM:CLEARING:9:ACOUSTIC_AMBIGUOUS:9999\n"  = 48 bytes.
 * 96 leaves room for fields added later without revisiting every buffer.
 ***********************************************************************/
#define GW_LINK_MAX_LINE 96

#endif // GREENWAVE_LINK_H
