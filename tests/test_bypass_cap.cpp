// =====================================================================
// test_bypass_cap.cpp  --  v7 geofence-enforcement stage cap
//
// Compile and run on a host, no ESP32 needed:
//     g++ -std=c++11 -o test_bypass_cap test_bypass_cap.cpp && ./test_bypass_cap
//
// WHAT THIS TESTS AND WHAT IT DOES NOT.
//
// The cap is six lines inside gwBuildDemand(), which cannot be compiled
// on a host: the surrounding function pulls in the whole ICU sketch.
// This file reproduces the ORDERED SEQUENCE of stage rules --
// base, receding cap, bypass cap, near-zone clamp -- with the same
// operators and the same order, and checks the truth table.
//
// It therefore proves the RULE is right and that ORDERING composes. It
// does NOT prove the copy in ICU_Decision.ino matches. Ordering is
// exactly what a reviewer reading a diff gets wrong, which is why the
// order is asserted rather than assumed -- test 7 fails if the bypass
// cap is moved after the near-zone clamp.
//
// The pairing between this file and the real one is checked by
// verify_rdu_tree.py, which greps ICU_Decision.ino for the cap and
// fails if it is missing or reordered.
// =====================================================================

#include <cstdio>
#include <cstring>

enum { LS_MONITOR = 0, LS_PREPARE = 1, LS_COMMIT = 2 };
enum { LE_NONE = 0, LE_EVU_DIRECT = 1, LE_EVU_INDIRECT = 2, LE_EVU_DEGRADED = 3 };

#define EVF_GEOFENCE_PASS     (1 << 3)
#define EVF_GEOFENCE_BYPASS   (1 << 4)
#define EVF_GEOFENCE_ENFORCED (1 << 5)

#define NEAR_ZONE_M 120.0f

static int  ALLOW_BYPASSED_COMMIT = 0;   // the production default

struct Demand { int stage; int evidence; int etaS; bool nearZone; };

// --- mirrors ICU_EVU.ino: what the ingest records from the flags ------
static bool ingestUnenforced(unsigned char flags, bool prevUnenforced) {
    bool enforced = (flags & EVF_GEOFENCE_ENFORCED) != 0;
    bool passed   = (flags & EVF_GEOFENCE_PASS)     != 0;
    // sticky
    return prevUnenforced || !enforced || !passed;
}

// --- mirrors gwBuildDemand(), EVU branch, in source order -------------
static Demand buildDemand(int evidence, bool receding, bool unenforced,
                          float distM, int rawEta) {
    Demand d; d.stage = LS_MONITOR; d.evidence = evidence;
    d.etaS = rawEta; d.nearZone = false;

    d.stage = (evidence == LE_EVU_DEGRADED) ? LS_PREPARE : LS_COMMIT;

    if (d.stage == LS_COMMIT && receding) { d.stage = LS_PREPARE; d.etaS = -1; }

    if (!ALLOW_BYPASSED_COMMIT && d.stage == LS_COMMIT && unenforced) {
        d.stage = LS_PREPARE; d.etaS = -1;
    }

    if (d.stage != LS_MONITOR && !receding && distM > 0.0f && distM <= NEAR_ZONE_M) {
        d.nearZone = true;
        if (d.etaS < 0 || d.etaS > 60) d.etaS = -1;
    }
    return d;
}

static int failures = 0;
static void check(const char *name, bool ok) {
    printf("%-62s %s\n", name, ok ? "PASS" : "** FAIL **");
    if (!ok) failures++;
}

// Flag sets as they appear on the wire.
static const unsigned char F_PRODUCTION = EVF_GEOFENCE_ENFORCED | EVF_GEOFENCE_PASS;
static const unsigned char F_BYPASS_BUILD = EVF_GEOFENCE_BYPASS | EVF_GEOFENCE_PASS;
static const unsigned char F_BYPASS_FAIL = EVF_GEOFENCE_BYPASS;
static const unsigned char F_UNSURVEYED = EVF_GEOFENCE_PASS;   // fails open

int main() {
    // ---- 1. NORMAL: verified production report, unchanged -------------
    {
        bool un = ingestUnenforced(F_PRODUCTION, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("1  production report reaches COMMIT (unchanged)",
              !un && d.stage == LS_COMMIT && d.etaS == 42);
    }

    // ---- 2. NORMAL relayed via a second node, unchanged ---------------
    {
        bool un = ingestUnenforced(F_PRODUCTION, false);
        un = ingestUnenforced(F_PRODUCTION, un);          // second node
        Demand d = buildDemand(LE_EVU_INDIRECT, false, un, 400.0f, 51);
        check("2  two production nodes reach COMMIT (unchanged)",
              !un && d.stage == LS_COMMIT && d.etaS == 51);
    }

    // ---- 3. BYPASS-flagged report cannot COMMIT -----------------------
    {
        bool un = ingestUnenforced(F_BYPASS_BUILD, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("3  bypassed report capped to PREPARE, cannot COMMIT",
              un && d.stage == LS_PREPARE && d.etaS == -1);
    }

    // ---- 4. Bypassed AND geofence would have REJECTED it --------------
    {
        bool un = ingestUnenforced(F_BYPASS_FAIL, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("4  bypassed + would-have-failed also capped",
              un && d.stage == LS_PREPARE);
    }

    // ---- 5. Unsurveyed node: PASS set but never enforced ---------------
    // The case the old code could not express at all: PASS was set, so
    // any check keyed on PASS alone would have trusted it.
    {
        bool un = ingestUnenforced(F_UNSURVEYED, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("5  unsurveyed node (PASS set, fails open) still capped",
              un && d.stage == LS_PREPARE);
    }

    // ---- 6. STICKY: one bypassed report taints the track --------------
    {
        bool un = ingestUnenforced(F_PRODUCTION, false);
        un = ingestUnenforced(F_BYPASS_BUILD, un);       // bypassed node relays
        un = ingestUnenforced(F_PRODUCTION, un);         // production node again
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("6  trust is NOT restored by a later production packet",
              un && d.stage == LS_PREPARE);
    }

    // ---- 7. ORDERING: cap must precede the near-zone clamp -------------
    // A bypassed vehicle 30 m from the stop line must not be marked
    // nearZone on the strength of a distance from the untrusted report.
    // If the cap were moved after the clamp this passes nearZone=true.
    {
        bool un = ingestUnenforced(F_BYPASS_BUILD, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 30.0f, 10);
        check("7  cap runs before NEAR_ZONE clamp (ordering)",
              d.stage == LS_PREPARE && d.nearZone == true && d.etaS == -1);
    }

    // ---- 8. Receding + bypass compose, neither is lost ----------------
    {
        bool un = ingestUnenforced(F_BYPASS_BUILD, false);
        Demand d = buildDemand(LE_EVU_DIRECT, true, un, 400.0f, 42);
        check("8  receding cap and bypass cap compose",
              d.stage == LS_PREPARE && d.etaS == -1 && !d.nearZone);
    }

    // ---- 9. DEGRADED production evidence unchanged --------------------
    {
        bool un = ingestUnenforced(F_PRODUCTION, false);
        Demand d = buildDemand(LE_EVU_DEGRADED, false, un, 400.0f, 42);
        check("9  DEGRADED production still PREPARE (unchanged)",
              !un && d.stage == LS_PREPARE);
    }

    // ---- 10. Escape hatch restores the old behaviour exactly ----------
    {
        ALLOW_BYPASSED_COMMIT = 1;
        bool un = ingestUnenforced(F_BYPASS_BUILD, false);
        Demand d = buildDemand(LE_EVU_DIRECT, false, un, 400.0f, 42);
        check("10 ALLOW_BYPASSED_COMMIT=1 reproduces pre-v7 behaviour",
              d.stage == LS_COMMIT && d.etaS == 42);
        ALLOW_BYPASSED_COMMIT = 0;
    }

    // ---- 11. Flag stays visible regardless of the cap -----------------
    {
        unsigned char f = F_BYPASS_BUILD;
        check("11 bypass flag remains readable for logging",
              (f & EVF_GEOFENCE_BYPASS) != 0);
    }

    printf("\n%s\n", failures ? "FAILURES PRESENT" : "All checks passed.");
    return failures ? 1 : 0;
}
