/***********************************************************************
 * ICU_Safety.ino  --  PHASE 5E ABUSE CONTROLS + 5F WATCHDOG
 *
 * A new tab in the ICU sketch folder.
 *
 * Everything here exists to bound what the system can do when something
 * ELSE has gone wrong -- a wedged decision loop, a jammed node, a
 * malfunctioning or malicious vehicle. None of it improves normal
 * operation, and if any of it fires regularly, something upstream is
 * broken and should be investigated rather than tuned around.
 *
 * ---------------------------------------------------------------------
 * WHY THESE MATTER MORE IN ADVISORY MODE THAN THEY LOOK
 *
 * Today the worst outcome is an officer being asked to override for a
 * vehicle that does not deserve it. That is a nuisance.
 *
 * The purpose of building them now is that the day GW_ENABLE_TCU
 * becomes 1, every one of these becomes a limit on a green light -- and
 * a system that has never had rate limiting cannot have it retrofitted
 * safely, because nobody knows what its normal rate actually is. Running
 * them in advisory mode is how the thresholds get calibrated against
 * real traffic before they matter.
 ***********************************************************************/

#include "GreenwaveLink.h"

// =====================================================
// 5F : WATCHDOG
// =====================================================
//
// The ERC already fails safe if the ICU stops TALKING (link timeout),
// and it caps any single alert locally. Neither covers an ICU whose
// decision loop is wedged while its comms task keeps running -- from
// the console's side that is indistinguishable from a genuine, very
// long emergency.
//
// The ESP32 task watchdog covers it: if loop() stops reaching the reset
// call, the chip reboots. On reboot ercInit() sends CLEAR, so the
// console returns to normal (spec 11.1: an ICU restart never restores
// preemption state).
//
// A reboot is a heavy response. It is the right one here because a
// decision loop that has stopped iterating cannot release anything, and
// a stuck alert with no path to release is worse than a ten-second
// outage during which the console shows NORMAL.

#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// Generous relative to a loop that normally runs in single-digit
// milliseconds. This is a fault detector, not a performance budget --
// LoRa receive and the periodic health printout both occasionally take
// tens of milliseconds, and a watchdog that fires on those would reboot
// a working system.
#define ICU_WDT_TIMEOUT_S  8

static bool gwWdtActive = false;

void gwWatchdogInit() {
    // ------------------------------------------------------------
    // TWO DIFFERENT APIs, DEPENDING ON THE ESP32 CORE VERSION.
    //
    // ESP-IDF 4.x:  esp_task_wdt_init(timeout_seconds, panic)
    // ESP-IDF 5.x:  esp_task_wdt_init(&config_struct)
    //
    // The 5.x signature is not source compatible, so a build that
    // assumes either one fails outright on the other. Selected at
    // compile time rather than pinning a core version, because the
    // whole point of this function is to still be working in two
    // years when the toolchain has moved on.
    //
    // Idle-task subscription is OFF in both paths. Watching the idle
    // tasks means any long-running work on either core trips the
    // watchdog, which on this build includes normal LoRa activity.
    // Only loop() is watched, because loop() is what must keep
    // making decisions.
    // ------------------------------------------------------------

    esp_err_t rc;

#if ESP_IDF_VERSION_MAJOR >= 5
    esp_task_wdt_config_t cfg = {
        .timeout_ms     = ICU_WDT_TIMEOUT_S * 1000,
        .idle_core_mask = 0,        // do not watch the idle tasks
        .trigger_panic  = true      // panic + reboot, not just a warning
    };

    rc = esp_task_wdt_init(&cfg);

    // The Arduino core may already have started the watchdog for
    // loopTask. Re-initialising then returns INVALID_STATE, which is
    // not an error -- reconfigure to OUR timeout instead of leaving
    // whatever the core chose.
    if (rc == ESP_ERR_INVALID_STATE) {
        rc = esp_task_wdt_reconfigure(&cfg);
    }
#else
    rc = esp_task_wdt_init(ICU_WDT_TIMEOUT_S, true);
#endif

    if (rc != ESP_OK) {
        // Say so loudly and carry on unguarded rather than pretending.
        // A watchdog believed to be armed and silently absent is
        // worse than a known-absent one: it is exactly the safety
        // claim nobody would re-check.
        Serial.printf("[WDT] *** INIT FAILED (err 0x%X) -- RUNNING "
                      "WITHOUT A WATCHDOG ***\n", (unsigned)rc);
        gwWdtActive = false;
        return;
    }

    // Subscribe this task (loop). INVALID_ARG means already
    // subscribed by the core, which is fine.
    rc = esp_task_wdt_add(NULL);

    if (rc != ESP_OK && rc != ESP_ERR_INVALID_ARG) {
        Serial.printf("[WDT] *** subscribe failed (err 0x%X) -- "
                      "RUNNING WITHOUT A WATCHDOG ***\n", (unsigned)rc);
        gwWdtActive = false;
        return;
    }

    gwWdtActive = true;

    Serial.printf("[WDT] watchdog armed, %d s timeout. A wedged "
                  "decision loop reboots the ICU; the console then "
                  "sees CLEAR.\n", ICU_WDT_TIMEOUT_S);
}

void gwWatchdogFeed() {
    // No-op when the watchdog failed to arm. Calling reset() on an
    // unsubscribed task returns an error every loop pass, which
    // would bury the log without changing anything.
    if (gwWdtActive) esp_task_wdt_reset();
}

// =====================================================
// 5E : PREEMPTION RATE LIMIT
// =====================================================
//
// Caps how often ONE approach may reach COMMIT.
//
// The threat is not a clever attack. It is a unit -- stolen, faulty, or
// simply left switched on in a depot -- that requests priority
// continuously. Without a cap that approach holds the junction
// indefinitely and every other approach starves.
//
// Spec decision D11 leaves the actual figure to the traffic authority,
// because it is a policy question about acceptable disruption rather
// than an engineering one. What the firmware must provide is the
// mechanism and an honest record of when it fires.
//
// [FIELD] 12/hour is a working default: one preemption every five
// minutes on a single approach, sustained. Real junctions should be
// measured before this is set.
#define COMMIT_RATE_WINDOW_MS   3600000UL
#define COMMIT_RATE_MAX         12

// Ring of COMMIT entry timestamps per approach.
static unsigned long commitTimes[GW_NUM_APPROACHES + 1][COMMIT_RATE_MAX];
static uint8_t       commitCount[GW_NUM_APPROACHES + 1] = {0};
static uint8_t       commitHead[GW_NUM_APPROACHES + 1]  = {0};
static uint32_t      commitBlocked[GW_NUM_APPROACHES + 1] = {0};

// How many COMMITs this approach has had inside the window.
static uint8_t gwCommitsInWindow(int lane, unsigned long now) {
    uint8_t n = 0;
    for (uint8_t i = 0; i < commitCount[lane]; i++) {
        if ((now - commitTimes[lane][i]) < COMMIT_RATE_WINDOW_MS) n++;
    }
    return n;
}

// True if this approach may enter COMMIT now.
//
// PREPARE is never rate limited. It costs a line on a screen and no
// buzzer, and suppressing it would hide the very activity an operator
// needs to see in order to notice that an approach is being abused.
bool gwCommitAllowed(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return false;

    unsigned long now = millis();
    uint8_t inWindow = gwCommitsInWindow(lane, now);

    if (inWindow < COMMIT_RATE_MAX) return true;

    commitBlocked[lane]++;

    // ------------------------------------------------------------
    // RATE LIMIT THE RATE-LIMIT MESSAGE.
    //
    // This started out logging on every call, on the reasoning that the
    // frequency of blocking is itself the diagnostic. That reasoning
    // was right and the implementation was wrong: gwCommitAllowed() is
    // called every loop pass, so a single blocked approach produced
    // 29,443 identical lines and a 2.5 MB capture in one short bench
    // run.
    //
    // A diagnostic that floods the log destroys the log. Everything
    // else that happened during those minutes -- node health, track
    // transitions, the release that actually mattered -- was buried.
    //
    // The count is still kept in full. It is now REPORTED on the first
    // block and then at most every 10 s, carrying the accumulated
    // total, which conveys the same frequency information in a form a
    // human can actually read.
    // ------------------------------------------------------------

    static unsigned long lastBlockLog[GW_NUM_APPROACHES + 1] = {0};

    bool first = (commitBlocked[lane] == 1);

    if (first || (now - lastBlockLog[lane]) >= 10000UL) {
        lastBlockLog[lane] = now;
        Serial.printf("[ABUSE] L%d COMMIT BLOCKED -- %u preemptions in the "
                      "last hour, limit %d. Approach held at PREPARE. "
                      "(blocked %lu times)\n",
                      lane, inWindow, COMMIT_RATE_MAX,
                      (unsigned long)commitBlocked[lane]);
    }
    return false;
}

// Record a COMMIT entry. Called by the decision layer on transition,
// never on refresh -- otherwise a single long event would exhaust the
// hourly budget in seconds.
void gwCommitRecord(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;

    unsigned long now = millis();

    if (commitCount[lane] < COMMIT_RATE_MAX) {
        commitTimes[lane][commitCount[lane]++] = now;
    } else {
        commitTimes[lane][commitHead[lane]] = now;
        commitHead[lane] = (commitHead[lane] + 1) % COMMIT_RATE_MAX;
    }
}

// =====================================================
// 5E : RF NOISE FLOOR  ->  LINK_SUSPECT
// =====================================================
//
// A jammed node and a dead node look identical from the ICU: both stop
// arriving. But one is a maintenance ticket and the other is an attack
// in progress, and they call for opposite responses.
//
// It matters because node loss UNLOCKS degraded authority at the
// survivor, which LOWERS the evidence bar. Jamming one node of a pair
// is far easier than defeating authentication, so without this an
// attacker gets a cheaper path by breaking a radio than by breaking
// crypto (spec S1-06).
//
// The discriminator available here is the ICU's own receive noise
// floor. A node that goes silent while the band is quiet is probably
// broken. A node that goes silent while the band is unusually loud may
// be being drowned out -- and in that case the correct response is to
// SUPPRESS degraded action rather than grant it.
//
// This is a heuristic, not proof. It is deliberately used only to
// withhold authority, never to grant it: a false positive costs a
// missed degraded-mode alert, a false negative would hand an attacker
// exactly what they were trying to obtain.

// ------------------------------------------------------------
// ADAPTIVE, NOT ABSOLUTE.
//
// The first version compared the measured floor against a fixed
// -95 dBm. A bench run read -83 to -93 dBm with three radios on one
// desk, so the band registered as permanently noisy -- and a permanent
// LINK_SUSPECT would suppress degraded mode exactly when it is needed.
//
// No single number can be right for both a bench and a roadside
// cabinet. What matters is not the absolute level but whether the band
// is louder THAN IT NORMALLY IS AT THIS SITE, which is a question only
// the site can answer.
//
// So the ICU learns its own quiet baseline during a settling period
// after boot, then flags elevation relative to that. A cabinet beside a
// substation and a lab bench both end up with a meaningful threshold,
// and neither needs a technician to measure and enter one.
// ------------------------------------------------------------

static float    gwNoiseFloorDbm   = -120.0f;   // fast-moving current estimate
static float    gwNoiseBaseline   = 0.0f;      // learned quiet reference
static bool     gwNoiseFloorValid = false;
static bool     gwBaselineLocked  = false;
static uint32_t gwNoiseSamples    = 0;
static uint32_t gwLinkSuspectHits = 0;

// Samples spent learning the baseline. Sampled at 1 Hz, so ~2 minutes.
//
// Long enough to average out passing transmissions, short enough that
// the protection is live well before anything interesting happens.
#define RF_BASELINE_SAMPLES  120

// How far above the learned baseline counts as elevated.
//
// WIDENED 12 -> 15 dB after bench measurement.
//
// Three sessions learned baselines of -73, -86 and -90 dBm, and normal
// activity produced excursions of +8 and +10 dB against a +12 margin.
// That is only 2 dB of headroom: one more radio on the desk, or a
// neighbour's equipment, and the band would read as noisy permanently.
//
// A false positive here is not harmless. LINK_SUSPECT SUPPRESSES
// degraded-mode authority, so a permanently "noisy" band means a lane
// with one failed node never acts at all -- the protection silently
// becoming an outage.
//
// 15 dB is a factor of ~32 in power. Ordinary shared-band traffic does
// not sustain that; a jammer or a stuck transmitter does.
//
// The asymmetry justifies erring wide: missing a jammer costs one
// degraded-mode alert that should have been suppressed, while a false
// alarm costs every degraded-mode alert at that junction, permanently
// and invisibly.
//
// [FIELD] Verify against real interference before relying on it.
#define RF_ELEVATED_MARGIN_DB  15.0f

// Called from the LoRa poll when NO packet is being received.
void gwNoteRfNoise(int rssi) {
    float v = (float)rssi;

    if (!gwNoiseFloorValid) {
        gwNoiseFloorDbm   = v;
        gwNoiseBaseline   = v;
        gwNoiseFloorValid = true;
        gwNoiseSamples    = 1;
        return;
    }

    // Current estimate: slow EMA. The quantity of interest is the
    // sustained band condition, not individual bursts -- a passing
    // transmitter is not a jammer.
    gwNoiseFloorDbm += (v - gwNoiseFloorDbm) / 32.0f;

    if (gwNoiseSamples < 0xFFFFFFFF) gwNoiseSamples++;

    if (!gwBaselineLocked) {
        // Learning. Track the quiet floor with an even slower filter.
        gwNoiseBaseline += (v - gwNoiseBaseline) / 64.0f;

        if (gwNoiseSamples >= RF_BASELINE_SAMPLES) {
            gwBaselineLocked = true;
            Serial.printf("[SAFETY] RF baseline learned: %.0f dBm. Elevation "
                          "beyond +%.0f dB now reads as a noisy band.\n",
                          gwNoiseBaseline, RF_ELEVATED_MARGIN_DB);
        }
        return;
    }

    // Locked. Let the baseline drift DOWNWARD only.
    //
    // A genuinely quieter band should update the reference, but a noisy
    // one must not -- otherwise a slowly ramped jammer would simply
    // teach the ICU to accept it, which is the one failure mode this
    // whole mechanism exists to prevent.
    if (v < gwNoiseBaseline) {
        gwNoiseBaseline += (v - gwNoiseBaseline) / 256.0f;
    }
}

bool gwRfBandNoisy() {
    // Never report noisy before the baseline is learned. During
    // settling there is nothing to compare against, and a false
    // LINK_SUSPECT withholds degraded authority -- the conservative
    // choice here is to withhold the SUPPRESSION, not the alert.
    if (!gwNoiseFloorValid || !gwBaselineLocked) return false;

    return gwNoiseFloorDbm > (gwNoiseBaseline + RF_ELEVATED_MARGIN_DB);
}

float gwRfNoiseFloor() {
    return gwNoiseFloorValid ? gwNoiseFloorDbm : 0.0f;
}

float gwRfBaseline() {
    return gwBaselineLocked ? gwNoiseBaseline : 0.0f;
}

bool gwRfBaselineReady() {
    return gwBaselineLocked;
}

void gwNoteLinkSuspect(int lane, int node) {
    gwLinkSuspectHits++;
    Serial.printf("[ABUSE] L%dN%d silent while the band is noisy "
                  "(%.0f dBm, baseline %.0f) -- LINK_SUSPECT. Degraded is "
                  "WITHHELD on this approach: a jammed node must not be "
                  "cheaper than a defeated key.\n",
                  lane, node, gwRfNoiseFloor(), gwRfBaseline());
}


// =====================================================
// BENCH: PRE-LOAD THE RATE COUNTER
// =====================================================
//
// The hourly limit cannot be exercised in real time. Reaching it
// honestly needs COMMIT_RATE_MAX genuine preemptions, each separated by
// the approach re-lock, which is several minutes of paced typing -- and
// what that actually tests is the operator's patience.
//
// Worse, the obvious shortcut does not work and looks like a bug when
// it fails: repeatedly re-opening a track while the alert is ALREADY
// live never transitions into COMMIT, so nothing is counted. That is
// correct -- it is one continuous preemption, and counting refreshes
// would let a single long event exhaust an hour's budget in seconds --
// but it makes the limit untestable by hand.
//
// So this fills the ring to one short of the limit. The next genuine
// COMMIT then trips it, exercising the REAL enforcement path in
// gwCommitAllowed() and the real demotion to PREPARE.
//
// What it does NOT test is the sliding window expiring after an hour.
// That cannot be tested in real time by anything short of waiting an
// hour, and pretending otherwise would be worse than saying so.
void gwAbuseFill(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) {
        Serial.printf("[ABUSE] lane %d out of range\n", lane);
        return;
    }

    commitCount[lane] = 0;
    commitHead[lane]  = 0;

    for (int i = 0; i < COMMIT_RATE_MAX - 1; i++) {
        gwCommitRecord(lane);
    }

    Serial.printf("[ABUSE] L%d pre-loaded to %d/%d. The NEXT commit is "
                  "allowed; the one after that must be BLOCKED and held "
                  "at PREPARE.\n",
                  lane, COMMIT_RATE_MAX - 1, COMMIT_RATE_MAX);
}

void gwAbuseReset(int lane) {
    if (lane < 1 || lane > GW_NUM_APPROACHES) return;
    commitCount[lane]   = 0;
    commitHead[lane]    = 0;
    commitBlocked[lane] = 0;
    Serial.printf("[ABUSE] L%d rate counter cleared\n", lane);
}

// =====================================================
// DIAGNOSTICS
// =====================================================

void gwPrintSafety() {
    unsigned long now = millis();

    if (gwRfBaselineReady()) {
        Serial.printf("[SAFETY] wdt=%s  rf=%.0fdBm base=%.0fdBm (+%.0f)%s  "
                      "link_suspect=%lu\n",
                      gwWdtActive ? "armed" : "OFF",
                      gwRfNoiseFloor(), gwRfBaseline(),
                      gwRfNoiseFloor() - gwRfBaseline(),
                      gwRfBandNoisy() ? " *BAND NOISY*" : "",
                      (unsigned long)gwLinkSuspectHits);
    } else {
        Serial.printf("[SAFETY] wdt=%s  rf=%.0fdBm  baseline learning "
                      "(%lu/%d)  link_suspect=%lu\n",
                      gwWdtActive ? "armed" : "OFF",
                      gwRfNoiseFloor(),
                      (unsigned long)gwNoiseSamples, RF_BASELINE_SAMPLES,
                      (unsigned long)gwLinkSuspectHits);
    }

    for (int lane = 1; lane <= GW_NUM_APPROACHES; lane++) {
        uint8_t n = gwCommitsInWindow(lane, now);
        if (n == 0 && commitBlocked[lane] == 0) continue;

        Serial.printf("         L%d commits/hour=%u/%d%s blocked=%lu\n",
                      lane, n, COMMIT_RATE_MAX,
                      (n >= COMMIT_RATE_MAX) ? " *AT LIMIT*" : "",
                      (unsigned long)commitBlocked[lane]);
    }
}
