#ifndef ICU_EVU_TYPES_H
#define ICU_EVU_TYPES_H

/***********************************************************************
 * ICU_EvuTypes.h  --  TYPES FOR THE SCENARIO 2 TRACK LAYER
 *
 * WHY THESE LIVE IN A HEADER RATHER THAN IN ICU_EVU.ino
 *
 * The Arduino IDE auto-generates function prototypes and inserts them
 * near the TOP of the sketch, above the point where types defined in a
 * .ino file exist. Any function whose signature names such a type then
 * fails with "'EvuTrack' does not name a type" -- even though the
 * definition precedes every real call and the code is valid C++.
 *
 * For a single function the workaround is to pass scalars instead
 * (gwDemandScore in ICU_Decision.ino does exactly that). EvuTrack
 * appears in five signatures and is genuinely a structure, so the honest
 * fix is a header: the preprocessor runs before the generator, so
 * anything included here is already visible when the prototypes are
 * emitted.
 *
 * This file is ICU-ONLY. It is deliberately NOT in GreenwaveTypes.h,
 * which is the wire protocol shared with the RDU and must stay free of
 * decision-layer internals -- those structures are not transmitted, are
 * not size-locked, and changing one must never oblige anyone to reflash
 * a roadside node.
 ***********************************************************************/

#include <Arduino.h>

// ---------------------------------------------------------------------
// VEHICLE REGISTRY  --  the fix for spec defect S2-01
//
// Priority is a property the AUTHORITY assigns to a vehicle, not one the
// vehicle asserts about itself. EVU2.ino compiles in
// `#define PRIORITY_CLASS 1` and transmits it; anyone able to reflash a
// unit could otherwise outrank every genuine emergency in the network,
// and the packet would authenticate perfectly because it really is
// signed. It is simply lying about its own importance.
// ---------------------------------------------------------------------
struct VehicleRegistryEntry {
    const char *vehicleId;
    uint8_t     priority;      // 1 = highest
    bool        enabled;       // false = revoked; tracked, never acted on
    const char *description;
};

// ---------------------------------------------------------------------
// EVU TRACK STATE
// ---------------------------------------------------------------------
enum EvuTrackState : uint8_t {
    EVU_IDLE      = 0,
    EVU_ACTIVE    = 1,   // fresh authenticated packets, approaching
    EVU_DEGRADED  = 2,   // inside grace period, no fresh packet
    EVU_DIVERGED  = 3,   // turned away
    EVU_COMPLETED = 4,   // passed the stop line, or flag dropped
    EVU_EXPIRED   = 5,
    EVU_REJECTED  = 6    // implausible or revoked
};

// ---------------------------------------------------------------------
// ONE TRACKED VEHICLE
// ---------------------------------------------------------------------
struct EvuTrack {
    bool     inUse        = false;
    char     vehicleId[8] = {0};

    uint8_t  state           = EVU_IDLE;

    // AUTHORITATIVE priority, from the registry. This is what
    // arbitration uses.
    uint8_t  priority        = 9;

    // The vehicle's OWN CLAIM, from the packet. Kept only so a
    // disagreement can be logged -- a unit asserting an importance it
    // was not granted is a meaningful security signal. Never acted upon.
    uint8_t  claimedPriority = 0;

    bool     registered      = false;

    int      lane            = 0;      // associated approach, 0 = none

    double   lat = 0, lon = 0;
    float    speedKmph     = 0;
    float    headingDeg    = -1.0f;    // -1 = no valid heading
    float    distanceM     = 0;
    float    prevDistanceM = 0;

    // ---- 5D: rolling speed ----
    //
    // ETA used the vehicle's INSTANTANEOUS reported speed, which is a
    // GPS-derived figure that jitters several km/h between fixes even on
    // a steady approach. Divided into a distance, that jitter lands
    // straight on the console as an ETA jumping around by seconds --
    // and an officer watching a number bounce learns to distrust it.
    //
    // A rolling mean over the last few fixes damps that without adding
    // meaningful lag: at a 2 s transmit interval this averages roughly
    // the last eight seconds of travel.
    static const uint8_t SPEED_HISTORY = 4;
    float    speedHistory[4] = {0, 0, 0, 0};
    uint8_t  speedHistoryCount = 0;
    uint8_t  speedHistoryHead  = 0;
    float    speedAvgKmph      = 0;

    // Both kept so the FALLING EDGE can be detected -- the driver
    // switching the siren off is the most reliable release signal in the
    // system, and v1.0 discarded it (spec S2-08).
    bool     emergencyFlag     = false;
    bool     prevEmergencyFlag = false;

    uint32_t lastSeq    = 0;
    bool     seqStarted = false;

    unsigned long firstSeenMs = 0;
    unsigned long lastSeenMs  = 0;

    // Consecutive fixes showing the vehicle moving AWAY. Divergence
    // needs several: one fix is ordinary GPS jitter, and an ambulance
    // stationary in traffic produces plenty of it (spec S2-09).
    uint8_t  growingCount = 0;

    uint8_t  evidence = 0;             // LinkEvidence
    int16_t  etaS     = -1;            // -1 = not computable

    bool     directReceived = false;   // heard by the ICU itself, not relayed

    // ---- v7: GEOFENCE TRUST STATE ----
    //
    // Copied from the reporting RDU's EVF_* flags at ingest. NOT part of
    // the wire protocol and NOT a decision-layer computation: this is
    // provenance, recorded where the rest of the track's provenance
    // already lives (directReceived, claimedPriority).
    //
    // WHY THE TRACK AND NOT THE DEMAND TUPLE. gwBuildDemand() reads the
    // track through accessors and never reaches into it directly; the
    // demand tuple is layer 3's input and is deliberately free of
    // per-track state. Putting the flag here and exposing an accessor is
    // the same shape as gwEvuIsReceding() and gwEvuDistanceM(), which is
    // what "smallest point that respects the flag" resolves to.
    //
    // A track is FULLY TRUSTED only when the geofence acted as a real
    // admission gate and this vehicle cleared it. See EVF_GEOFENCE_ENFORCED.
    bool     geofenceEnforced = false;   // EVF_GEOFENCE_ENFORCED
    bool     geofencePassed   = false;   // EVF_GEOFENCE_PASS
    bool     geofenceBypassed = false;   // EVF_GEOFENCE_BYPASS

    // Sticky. A track that was ever reported by an unenforced node stays
    // unenforced for its whole life.
    //
    // Without this, a bypassed node and a production node covering the
    // same approach would let the track's trust level flip with whichever
    // node last relayed a packet -- and an attacker who could reach one
    // bypassed node would only need it to be the last reporter. Trust
    // that can be restored by a later packet is not a constraint.
    bool     everUnenforced   = false;
};

#endif // ICU_EVU_TYPES_H
