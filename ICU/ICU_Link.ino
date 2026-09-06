/***********************************************************************
 * ICU_Link.ino  --  ICU <-> ERC TRANSPORT LAYER
 *
 * PHASE 2 of the v2.0 implementation.
 *
 * Put this file in the SAME FOLDER as ICU.ino. The Arduino IDE compiles
 * every .ino in a sketch folder into one translation unit, so this is a
 * new tab in the same sketch -- not a second program. It exists as a
 * separate tab purely to keep the 1180-line ICU.ino from being edited in
 * a dozen places for one feature.
 *
 * ---------------------------------------------------------------------
 * WHAT THIS FILE IS, AND WHAT IT IS NOT
 *
 * It is the ICU's mouth and ears for the console. Transport only.
 *
 * It contains NO decision logic. It does not decide when to alert, what
 * stage to be in, or when to release -- those are Phase 5, and they live
 * in ICU.ino. This file only carries decisions that have already been
 * made, and reports what came back.
 *
 * That separation is the point. The spec's three-layer model exists so
 * that arbitration logic and output logic never end up in the same
 * function, because in the automatic-control version of this system that
 * function would be the one holding a green light.
 *
 * ---------------------------------------------------------------------
 * WIRING  (taken from the working Model_ICU link -- do not change)
 *
 *   ICU UART2 GPIO14 (TX)  ---->  ERC GPIO34 (RX)
 *   ICU UART2 GPIO15 (RX)  <----  ERC GPIO15 (TX)
 *   GND -------------------------  GND
 *
 * No conflict with the LoRa SPI bus, which uses GPIO 8-13.
 ***********************************************************************/

#include "GreenwaveLink.h"

// Defined in ICU_Tracks.ino / ICU_Geometry.ino. Declared here because
// the IDE concatenates .ino tabs alphabetically and ICU_Link is last.
void gwSimApproach(int lane, long deltaMs, float conf, bool sustained);
void gwSimSingle(int lane, bool useFar, float conf, bool sustained);
void gwSimReset(int lane);
void gwSimReassert(int lane);
void gwPrintWindow(int lane);
void gwSimEvuApproach(int lane, float distM, float speedKmph);
void gwSimEvuStep(int lane, float seconds);
void gwSimEvuDiverge(int lane);
void gwSimEvuTeleport(int lane, float jumpM);
void gwSimEvuEmergency(int lane, bool on);
void gwSimEvuClear(int lane);
void gwSimEvuSpeed(int lane, float kmph);
void gwSimEvuRun(int lane, int count, float secondsPerStep);
void gwPrintEvuTracks();
bool gwSiteCommand(int argc, char **argv);
void gwSitePrint();
void gwSiteLoad();
void gwAbuseFill(int lane);
void gwAbuseReset(int lane);
void gwPrintTracks();
void gwPrintGeometry();

// =====================================================
// TCU  --  DISABLED, DELIBERATELY KEPT
// =====================================================
//
// The physical demonstration model drives a Traffic Control Unit over a
// second UART with lines like "SIG:R,G,R,20". Real signal control.
//
// The current stage is ADVISORY: the ICU informs the console and a
// traffic officer performs the override manually. So this is 0.
//
// The code is kept rather than deleted because the interface question it
// represents is unresolved, not obsolete. Spec decision D1 asks whether
// Priority One should eventually drive the traffic controller's own
// preempt input or drive lamp outputs directly, and the answer changes
// the project's certification burden and liability exposure completely.
// Deleting the working transport now would mean rebuilding and re-testing
// it later to answer a question that is already on the record.
//
// Setting this to 1 does NOT make the system safe to control signals.
// Everything in spec section 11.3 -- pedestrian clearance that is never
// truncated, a hard maximum preemption duration, cross-traffic
// starvation limits, a conflict monitor -- is absent from this build.
// Advisory mode is what makes their absence acceptable.
#define GW_ENABLE_TCU  0

#define ERC_RX 15
#define ERC_TX 14

#if GW_ENABLE_TCU
  #define TCU_RX 16
  #define TCU_TX 17
  HardwareSerial TrafficSerial(1);
#endif

HardwareSerial ErcSerial(2);

// =====================================================
// LINK STATE
// =====================================================

static uint32_t      ercPingSeq       = 0;
static unsigned long lastPingSent     = 0;
static unsigned long lastErcRx        = 0;
static bool          ercLinkUp        = false;

// Round-trip health. The ICU knows its own transmissions left; only PONG
// proves the return path works. That path carries the operator's ACK,
// which is the only evidence anywhere in the system that a human saw an
// alert -- so an outbound-only link is a link that looks fine while the
// console's replies vanish.
static uint32_t ercPingsSent   = 0;
static uint32_t ercPongsRecv   = 0;
static uint32_t ercPongMismatch = 0;   // PONG seq that was not the last PING
static bool     ercStatsStarted = false;  // zeroed once, at first contact

// Console-reported state, as last heard. Never used to make decisions --
// it is what the ERC says about itself, and this layer does not judge it.
static char ercState[16] = "IDLE";
static bool ercAcked     = false;

// Acknowledgement accounting. Operator and automatic are counted apart
// on purpose -- see the AUTO_ACK handler for why collapsing them hides
// a deployment problem rather than a firmware one.
static uint32_t ercOperatorAcks   = 0;
static uint32_t ercAutoAcks       = 0;
static uint32_t ercLegacyAck      = 0;   // old ERC build still on the wire
static uint32_t ercAlertTimeouts  = 0;   // console backstop fired
static uint32_t ercCorruptLines   = 0;   // non-printable bytes on the wire
static uint32_t ercSelfEcho       = 0;   // our own TX seen on our own RX

// Per-approach outbound de-duplication.
//
// ALERT is idempotent, and a stage can be re-evaluated many times per
// second once Phase 5 lands. Re-sending an identical line at that rate
// would flood a 9600 baud link and, worse, make the serial log unreadable
// at exactly the moment somebody is trying to debug an event from it.
// So an ALERT goes out on CHANGE; HOLD carries the "still true" case.
struct ErcApproachTx {
    uint8_t  stage;
    uint8_t  priority;
    uint8_t  evidence;
    int16_t  etaS;
    bool     active;
    unsigned long lastSent;
};
static ErcApproachTx ercTx[GW_NUM_APPROACHES + 1];   // 1-based; [0] unused

// Even unchanged, an alert is refreshed at this interval. The ERC's
// link-timeout fail-safe is driven by PING, so this is not a keepalive --
// it is repair. A single dropped ALERT would otherwise leave the console
// showing PREPARE for an event that has since reached COMMIT, with
// nothing to correct it until the next genuine change.
#define ERC_ALERT_REFRESH_MS  3000UL

// =====================================================
// LOW-LEVEL SEND
// =====================================================

static void ercSendLine(const char* fmt, ...) {
    char line[GW_LINK_MAX_LINE];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);

    ErcSerial.println(line);
    ErcSerial.flush();

    // PING is excluded from the log. At 1 Hz it would be 86,400 lines a
    // day of "nothing happened", burying the handful of lines that matter
    // in the serial capture an engineer actually reads.
    if (strncmp(line, GW_CMD_PING, 4) != 0) {
        Serial.printf("[ERC TX] %s\n", line);
    }
}

// =====================================================
// PUBLIC SEND API   (called by ICU.ino, Phase 5)
// =====================================================

// Raise or update an alert on one approach.
// Sends only on change, or on the refresh interval. Safe to call every
// loop iteration.
void ercAlert(int lane, uint8_t stage, uint8_t priority,
              uint8_t evidence, int16_t etaSeconds)
{
    const char* appr = gwErcApproachName(lane);
    if (appr == nullptr) {
        Serial.printf("[ERC] bad lane %d, nothing sent\n", lane);
        return;
    }

    ErcApproachTx& t = ercTx[lane];

    bool changed = (!t.active)
                || (t.stage    != stage)
                || (t.priority != priority)
                || (t.evidence != evidence);

    // ETA moves continuously, so treating any change to it as a change
    // would defeat the de-duplication entirely. Only a whole-second shift
    // is worth a line, and only while the console is actually showing a
    // countdown.
    bool etaMoved = (t.etaS != etaSeconds);

    bool stale = (millis() - t.lastSent) > ERC_ALERT_REFRESH_MS;

    if (!changed && !etaMoved && !stale) return;

    t.stage    = stage;
    t.priority = priority;
    t.evidence = evidence;
    t.etaS     = etaSeconds;
    t.active   = true;
    t.lastSent = millis();

    ercSendLine("%s:%s:%s:%u:%s:%d",
                GW_CMD_ALERT, appr, gwStageName(stage),
                (unsigned)priority, gwEvidenceName(evidence),
                (int)etaSeconds);

#if GW_ENABLE_TCU
    // The TCU has no notion of PREPARE. Only a COMMIT means "change the
    // lights". Sending it anything on PREPARE would convert a reversible
    // advisory stage into a real phase change, which is exactly the
    // distinction the two-stage model exists to preserve.
    if (stage == LS_COMMIT) {
        const char* tcu = gwTcuApproachName(lane);
        if (tcu != nullptr) {
            TrafficSerial.println(tcu);
            Serial.printf("[TCU TX] %s\n", tcu);
        }
    }
#endif
}

// Refresh an existing alert without changing it. "Still coming."
void ercHold(int lane)
{
    const char* appr = gwErcApproachName(lane);
    if (appr == nullptr) return;
    if (!ercTx[lane].active) return;

    ercTx[lane].lastSent = millis();
    ercSendLine("%s:%s", GW_CMD_HOLD, appr);
}

// End an alert, with the reason it ended.
void ercRelease(int lane, uint8_t reason)
{
    const char* appr = gwErcApproachName(lane);
    if (appr == nullptr) return;

    ercSendLine("%s:%s:%s", GW_CMD_RELEASE, appr, gwReleaseName(reason));

    ercTx[lane].active = false;
    ercTx[lane].stage  = LS_MONITOR;
    ercTx[lane].etaS   = -1;

#if GW_ENABLE_TCU
    TrafficSerial.println("NORMAL");
#endif
}

// Approaches with a live demand that are NOT on the console.
//
// count 0 clears it. Sent on change only -- the waiting set is stable
// for seconds at a time, and re-sending it at the decision loop's rate
// would flood a 9600 baud link that also has to carry the alert itself.
void ercQueue(const uint8_t *lanes, const uint8_t *prios, uint8_t count)
{
    static uint8_t lastCount = 0xFF;
    static uint8_t lastLanes[GW_NUM_APPROACHES] = {0};
    static uint8_t lastPrios[GW_NUM_APPROACHES] = {0};

    bool changed = (count != lastCount);
    for (uint8_t i = 0; i < count && !changed; i++) {
        if (lanes[i] != lastLanes[i] || prios[i] != lastPrios[i]) changed = true;
    }
    if (!changed) return;

    lastCount = count;
    for (uint8_t i = 0; i < count && i < GW_NUM_APPROACHES; i++) {
        lastLanes[i] = lanes[i];
        lastPrios[i] = prios[i];
    }

    char line[GW_LINK_MAX_LINE];
    int  n = snprintf(line, sizeof(line), "%s:%u", GW_CMD_QUEUE, (unsigned)count);

    for (uint8_t i = 0; i < count; i++) {
        const char *nm = gwErcApproachName(lanes[i]);
        if (nm == nullptr) continue;
        n += snprintf(line + n, sizeof(line) - n, ":%s:%u",
                      nm, (unsigned)prios[i]);
        if (n >= (int)sizeof(line) - 12) break;   // never truncate mid-field
    }

    ercSendLine("%s", line);
}

// Per-node health. Pushed on change by ICU.ino.
void ercHealth(int lane, int node, uint8_t role, uint8_t health, uint16_t distM)
{
    ercSendLine("%s:%d:%d:%s:%s:%u",
                GW_CMD_HEALTH, lane, node,
                gwRoleName(role), gwHealthName(health), (unsigned)distM);
}

// Return everything to idle. Used on boot, on fail-safe, and whenever the
// ICU is no longer confident of its own state.
void ercClearAll(uint8_t reason)
{
    for (int l = 1; l <= GW_NUM_APPROACHES; l++) {
        if (ercTx[l].active) ercRelease(l, reason);
    }
    ercSendLine("CLEAR");
}

// =====================================================
// RECEIVE
// =====================================================

// True if every character is printable ASCII.
//
// A line that fails this came off the wire corrupted, and the right
// response is to drop it rather than attempt a partial match against
// it. Dispatch below is prefix-based -- strncmp(line, "PONG", 4) -- so
// a line whose leading bytes are noise is ignored, but one whose noise
// lands later would match a prefix while carrying a corrupted payload.
// Rejecting outright removes the question entirely.
static bool ercLineIsClean(const char* line)
{
    for (const char* p = line; *p; p++) {
        if (*p < 0x20 || *p > 0x7E) return false;
    }
    return true;
}

// True if this line is something the ICU SENDS, not something it can
// legitimately receive.
//
// The ICU is the only talker for these keywords; the ERC never sends
// them. So a line matching one of them did not come from the console --
// it is the ICU's own transmission arriving back on its own receiver.
static bool ercIsOwnTransmission(const char* line)
{
    return strncmp(line, GW_CMD_PING,    4) == 0
        || strncmp(line, GW_CMD_ALERT,   5) == 0
        || strncmp(line, GW_CMD_HOLD,    4) == 0
        || strncmp(line, GW_CMD_RELEASE, 7) == 0
        || strncmp(line, GW_CMD_HEALTH,  6) == 0
        || strncmp(line, "CLEAR",        5) == 0
        || strncmp(line, "NODE:",        5) == 0;
}

static void ercHandleLine(char* line)
{
    // ------------------------------------------------------------
    // REJECT OUR OWN TRANSMISSIONS BEFORE ANYTHING ELSE.
    //
    // NOTE THE ORDER: this runs BEFORE lastErcRx is stamped. That is the
    // whole point, not a detail.
    //
    // A bench run caught the ICU receiving "PING:10" -- a line it had
    // just sent. While the ERC is booting, its TX pin is high-impedance
    // for about twelve seconds, and the ICU's receive wire picks up
    // crosstalk from its own transmit wire running alongside it.
    //
    // Two things went wrong because of that, and the second is the one
    // that matters:
    //
    //   1. The link statistics zeroed on the echo instead of on real
    //      contact, so twelve genuine boot-time losses were counted
    //      against a link that had not dropped anything.
    //
    //   2. THE ICU TREATED ITS OWN TRANSMISSION AS PROOF THE CONSOLE WAS
    //      ALIVE. That is a false liveness signal, and it is exactly the
    //      shape of failure the fail-safe exists to catch: if the return
    //      wire breaks or the ERC dies while the ICU keeps transmitting,
    //      crosstalk on the adjacent wire could hold the link "UP"
    //      forever. The console would be gone and the ICU would never
    //      notice.
    //
    // Liveness must be established only by something ONLY the far end
    // could have sent. Stamping the timestamp first and filtering after
    // would leave the fail-safe defeated by a wire lying next to another
    // wire.
    // ------------------------------------------------------------

    if (ercIsOwnTransmission(line)) {
        ercSelfEcho++;
        return;
    }

    lastErcRx = millis();

    if (!ercLineIsClean(line)) {
        ercCorruptLines++;
        // Deliberately not printed per occurrence. On a noisy wire that
        // would be the loudest thing in the log while adding nothing
        // after the first one; the count in ercPrintStats() is the
        // useful form.
        return;
    }

    if (!ercLinkUp) {
        ercLinkUp = true;

        // Reset the link statistics the FIRST time the console answers.
        //
        // PINGs sent before the ERC was listening are not losses -- they
        // are transmissions into a board that was still booting, or into
        // a reset window. Counting them makes the lifetime figure
        // permanently wrong: five boot-time misses read as 83% over the
        // first thirty pings and never fully recover, because the
        // denominator grows but the numerator is stuck.
        //
        // That matters because this percentage is a DIAGNOSTIC. A number
        // that sits at 96% forever because of something that happened at
        // startup cannot be used to notice a link genuinely degrading
        // to 96%. Zeroing at first contact makes it measure the running
        // link, which is the only thing worth measuring.
        Serial.println("[ERC] link UP");
    }

    // ---- PONG:<seq> ----
    if (strncmp(line, GW_RPY_PONG, 4) == 0) {

        // Zero the statistics on the FIRST PONG specifically, not on the
        // first traffic of any kind.
        //
        // PONG is the narrowest possible proof that the console is
        // listening: it only exists as a reply, so it cannot be produced
        // by an echo, by noise, or by a board that is still booting.
        //
        // The previous version zeroed on first contact of any sort and
        // was tripped by the self-echo above -- it reset the counters
        // twelve seconds too early, then counted the twelve real
        // boot-window losses anyway. Worst of both.
        //
        // PINGs sent while the ERC was still booting are not losses.
        // They are transmissions into a board that was not there yet,
        // and counting them makes the lifetime figure permanently wrong:
        // the denominator grows while the numerator stays stuck, so the
        // percentage creeps toward 100% without ever arriving. A number
        // that behaves like that cannot be used to notice a link
        // genuinely degrading, which is the only reason to measure it.
        if (!ercStatsStarted) {
            ercStatsStarted = true;
            ercPingsSent    = 1;   // this PING is about to be credited
            ercPongsRecv    = 0;
            ercPongMismatch = 0;
            Serial.println("[ERC] console responding -- link stats zeroed");
        }

        ercPongsRecv++;
        char* colon = strchr(line, ':');
        if (colon != nullptr) {
            uint32_t seq = (uint32_t)strtoul(colon + 1, nullptr, 10);
            // A mismatch is normal when a PING is in flight; a PERSISTENT
            // mismatch means the console is falling behind rather than
            // dropping packets, which is a different fault with a
            // different fix. Counted, not logged per occurrence.
            if (seq != ercPingSeq) ercPongMismatch++;
        }
        return;
    }

    // ---- operator acknowledged an alert ----
    //
    // A HUMAN PRESSED THE BUTTON. This is the only evidence anywhere in
    // the system that an alert reached a person, so it is counted, not
    // just logged.
    if (strcmp(line, "ACKNOWLEDGED") == 0) {
        ercAcked = true;
        ercOperatorAcks++;
        Serial.println("[ERC RX] operator ACKNOWLEDGED");
        return;
    }

    // AUTO_ACK is the console silencing its own buzzer after 10 s with
    // nobody present. It is NOT an operator acknowledgement and must
    // never be counted as one -- the difference is whether a human was
    // there, which is the entire question an audit of a missed emergency
    // would be asking.
    //
    // ercAcked is deliberately NOT set. Tracked separately: a junction
    // whose auto-ack count materially exceeds its operator-ack count is
    // one where alerts are routinely reaching nobody, and that is a
    // deployment problem no amount of firmware will fix. It is invisible
    // unless the two are counted apart.
    if (strcmp(line, "AUTO_ACK") == 0) {
        ercAutoAcks++;
        Serial.println("[ERC RX] AUTO_ACK (no operator present)");
        return;
    }

    // Legacy bare "ACK" from an older ERC build.
    //
    // "ACK" is a command the ICU SENDS to the console, so receiving it
    // back is a vocabulary collision rather than a reply -- the same
    // word travelling in both directions with different meanings. Newer
    // ERC firmware sends ACKNOWLEDGED instead.
    //
    // Accepted, because during an upgrade you will inevitably have one
    // board of each version and an unrecognised acknowledgement is worse
    // than an ambiguous one. Flagged, because it means the two boards
    // are out of step and someone should reflash the console.
    if (strcmp(line, "ACK") == 0) {
        ercAcked = true;
        ercLegacyAck++;
        Serial.println("[ERC RX] ACK (legacy ERC firmware -- reflash the console)");
        return;
    }

    // Local safety backstop fired on the console: it held an alert for
    // longer than its own maximum and ended it without being told to.
    //
    // This is a FAULT REPORT, not a normal ending. It means the ICU did
    // not release when it should have -- a wedged decision loop, or a
    // release path that never fires. If it appears in a log, something
    // upstream needs investigating.
    if (strcmp(line, "ALERT_TIMEOUT") == 0) {
        ercAlertTimeouts++;
        Serial.println("[ERC RX] *** ALERT_TIMEOUT -- console backstop fired, "
                       "ICU failed to release ***");
        return;
    }

    if (strcmp(line, "ACK_IGNORED") == 0) {
        Serial.println("[ERC RX] ACK ignored (nothing active)");
        return;
    }

    // ---- console returned to idle ----
    if (strcmp(line, "CLEARED") == 0 || strcmp(line, "NORMAL_MODE") == 0) {
        strncpy(ercState, "IDLE", sizeof(ercState) - 1);
        ercAcked = false;
        Serial.printf("[ERC RX] %s\n", line);
        return;
    }

    if (strcmp(line, "SEQUENCE_COMPLETE") == 0) {
        strncpy(ercState, "COMPLETE", sizeof(ercState) - 1);
        Serial.println("[ERC RX] sequence complete");
        return;
    }

    // ---- alert acknowledged receipt ----
    if (strncmp(line, "RECEIVED:", 9) == 0) {
        strncpy(ercState, "ALERT", sizeof(ercState) - 1);
        ercAcked = false;
        Serial.printf("[ERC RX] %s\n", line);
        return;
    }

    if (strncmp(line, "NODE_", 5) == 0) {
        Serial.printf("[ERC RX] %s\n", line);
        return;
    }

    // Unknown lines are reported once and ignored. Not an error: an ERC
    // on newer firmware may say things this ICU has never heard of, and
    // the link must keep working for everything they do share.
    Serial.printf("[ERC RX] (unhandled) %s\n", line);
}

static void ercPoll()
{
    static char buf[GW_LINK_MAX_LINE];
    static uint8_t len = 0;

    while (ErcSerial.available()) {
        char c = (char)ErcSerial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            buf[len] = '\0';
            if (len > 0) ercHandleLine(buf);
            len = 0;
            continue;
        }

        if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            // Overlong line: discard it rather than truncate and act on a
            // half-parsed command. A partial "RELEASE" is worse than none.
            len = 0;
        }
    }
}

// =====================================================
// SERVICE  --  call from loop()
// =====================================================

void ercService()
{
    ercPoll();

    unsigned long now = millis();

    if (now - lastPingSent >= GW_PING_INTERVAL_MS) {
        lastPingSent = now;
        ercPingSeq++;
        ercPingsSent++;
        ercSendLine("%s:%lu", GW_CMD_PING, (unsigned long)ercPingSeq);
    }

    // Inbound silence. Note the asymmetry: the ICU keeps transmitting to
    // a console it cannot hear, on purpose. The ERC's own timeout will
    // clear its screen if the link is genuinely dead, and if the fault is
    // one-directional -- our RX pin, a broken return wire -- then the
    // console is still receiving and should still be told what is
    // happening. Going quiet here would blank a working display over a
    // fault on the path that does not carry alerts.
    if (ercLinkUp && (now - lastErcRx) > GW_LINK_TIMEOUT_MS) {
        ercLinkUp = false;
        Serial.println("[ERC] link DOWN (no reply; still transmitting)");
    }
}

// =====================================================
// INIT  --  call from setup()
// =====================================================

void ercInit()
{
    ErcSerial.setRxBufferSize(1024);
    ErcSerial.begin(9600, SERIAL_8N1, ERC_RX, ERC_TX);
    ErcSerial.setTimeout(20);

    // Discard whatever is already sitting in the FIFO.
    //
    // While this board is in reset its RX pin floats, and the ERC -- which
    // did NOT reset -- keeps transmitting into it. The UART latches
    // partial bytes and framing errors from that window, so the first
    // thing read after begin() is reliably garbage:
    //
    //     [ERC RX] (unhandled) ?PONG:1
    //     [ERC RX] (unhandled) ???
    //
    // Harmless in itself -- unknown lines are ignored by design. The
    // reason it is worth clearing is that a fragment can concatenate
    // with the next real line and turn a valid command into a
    // near-miss, which is a worse failure than a dropped one.
    //
    // 50 ms covers roughly five character times at 9600 baud, which is
    // enough for any in-flight byte to finish arriving before we flush.
    delay(50);
    while (ErcSerial.available()) ErcSerial.read();

#if GW_ENABLE_TCU
    TrafficSerial.begin(9600, SERIAL_8N1, TCU_RX, TCU_TX);
    TrafficSerial.setTimeout(20);
#endif

    for (int l = 0; l <= GW_NUM_APPROACHES; l++) {
        ercTx[l].stage    = LS_MONITOR;
        ercTx[l].priority = 0;
        ercTx[l].evidence = LE_NONE;
        ercTx[l].etaS     = -1;
        ercTx[l].active   = false;
        ercTx[l].lastSent = 0;
    }

    lastErcRx    = millis();
    lastPingSent = 0;

    // Spec section 11.1: on reboot the ICU starts in NORMAL with all
    // state cleared, and preemption state is NEVER restored from
    // persistent storage. The console may well be sitting in a stale
    // alert from before the reset, and it has no way to know the ICU
    // restarted, so the first thing we do is tell it.
    ercSendLine("CLEAR");

    Serial.printf("[ERC] link init  TX=GPIO%d RX=GPIO%d  9600 8N1  TCU=%s\n",
                  ERC_TX, ERC_RX, GW_ENABLE_TCU ? "ENABLED" : "disabled");
}

// =====================================================
// DIAGNOSTICS  --  called by the ICU's status printer
// =====================================================

void ercPrintStats()
{
    // A PONG can outnumber its PING for one sample, and the arithmetic
    // has to survive it rather than print 103.8%.
    //
    // The counters are zeroed on the FIRST PONG, to exclude the PINGs
    // sent into a console that was still booting. A PONG already in
    // flight for a pre-zeroing PING then lands afterwards and is counted
    // against a denominator that no longer includes its PING.
    //
    // Transient and self-correcting -- one extra reply, once, at
    // startup. But an impossible number in a health readout is worse
    // than the tiny inaccuracy it reports: the whole value of this line
    // is that a reader can trust it enough to notice a real change, and
    // a figure over 100% invites them to dismiss the metric instead.
    uint32_t pongs = (ercPongsRecv > ercPingsSent) ? ercPingsSent : ercPongsRecv;

    uint32_t lost = (ercPingsSent > pongs) ? (ercPingsSent - pongs) : 0;
    float pct = (ercPingsSent > 0)
              ? (100.0f * (float)pongs / (float)ercPingsSent) : 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    Serial.printf("[ERC LINK] %s  ping=%lu pong=%lu lost=%lu (%.1f%% return) "
                  "seqmismatch=%lu  state=%s acked=%d\n",
                  ercLinkUp ? "UP" : "DOWN",
                  (unsigned long)ercPingsSent, (unsigned long)pongs,
                  (unsigned long)lost, pct,
                  (unsigned long)ercPongMismatch,
                  ercState, ercAcked ? 1 : 0);

    Serial.printf("[ERC ACK ] operator=%lu auto=%lu legacy=%lu | backstop_fired=%lu\n",
                  (unsigned long)ercOperatorAcks, (unsigned long)ercAutoAcks,
                  (unsigned long)ercLegacyAck, (unsigned long)ercAlertTimeouts);

    if (ercSelfEcho > 0) {
        Serial.printf("[ERC WIRE] %lu self-echo line(s) ignored -- normal during "
                      "ERC boot; persistent means TX/RX crosstalk\n",
                      (unsigned long)ercSelfEcho);
    }

    if (ercCorruptLines > 0) {
        Serial.printf("[ERC WIRE] %lu corrupted line(s) rejected -- check the "
                      "GPIO15 return wire, its ground, and its length\n",
                      (unsigned long)ercCorruptLines);
    }
}

bool ercIsLinkUp()      { return ercLinkUp; }
bool ercIsAcked()       { return ercAcked;  }

// =====================================================
// BENCH CONSOLE  --  PHASE 3
// =====================================================
//
// Type commands into the ICU's USB serial monitor to drive the ERC
// directly, with no RDUs and no ambulance.
//
// This exists because the console is now the ONLY output of the system,
// and until Phase 4 there is no way to make it do anything: an alert
// requires acoustic detections from roadside nodes that have not been
// built yet. Without this you would have to finish the RDU firmware
// before you could see whether a single screen renders correctly, and
// then debug both at once.
//
// It is also the tool for the failure cases that hardware cannot easily
// produce on demand. Getting a real microphone to report AMBIGUOUS, or a
// real node to go MISCONFIGURED, means deliberately breaking equipment.
// Here it is one line of typing, which means those paths actually get
// tested rather than being reasoned about and shipped.
//
// KEEP THIS IN THE PRODUCTION BUILD. It sends alerts to a console for a
// human to judge; it cannot change a signal, and every command it issues
// is logged identically to a real one. The equivalent hook in a build
// that drives lamps would be a different question entirely -- but that
// build does not exist and, per spec decision D1, may never.
//
//   alert <lane> <stage> [pri] [evidence] [eta]
//   hold <lane>
//   release <lane> [reason]
//   health <lane> <node> <role> <state> <dist>
//   clear
//   stats
//   help
//
// Example bench session:
//   alert 1 prepare 1 acoustic-degraded -1
//   alert 1 commit  1 acoustic 12
//   release 1 passed

static uint8_t benchParseStage(const char* s) {
    if (strcasecmp(s, "monitor")  == 0) return LS_MONITOR;
    if (strcasecmp(s, "prepare")  == 0) return LS_PREPARE;
    if (strcasecmp(s, "commit")   == 0) return LS_COMMIT;
    if (strcasecmp(s, "clearing") == 0) return LS_CLEARING;
    return 0xFF;
}

static uint8_t benchParseEvidence(const char* s) {
    if (strcasecmp(s, "evu")                == 0) return LE_EVU_DIRECT;
    if (strcasecmp(s, "evu-relay")          == 0) return LE_EVU_INDIRECT;
    if (strcasecmp(s, "evu-degraded")       == 0) return LE_EVU_DEGRADED;
    if (strcasecmp(s, "acoustic")           == 0) return LE_ACOUSTIC_CONFIRMED;
    if (strcasecmp(s, "acoustic-degraded")  == 0) return LE_ACOUSTIC_DEGRADED;
    if (strcasecmp(s, "acoustic-ambiguous") == 0) return LE_ACOUSTIC_AMBIGUOUS;
    return 0xFF;
}

static uint8_t benchParseReason(const char* s) {
    if (strcasecmp(s, "flag-off")   == 0) return LR_FLAG_OFF;
    if (strcasecmp(s, "passed")     == 0) return LR_PASSED;
    if (strcasecmp(s, "siren-lost") == 0) return LR_SIREN_LOST;
    if (strcasecmp(s, "diverged")   == 0) return LR_DIVERGED;
    if (strcasecmp(s, "expired")    == 0) return LR_EXPIRED;
    if (strcasecmp(s, "max")        == 0) return LR_MAX_DURATION;
    if (strcasecmp(s, "operator")   == 0) return LR_OPERATOR;
    if (strcasecmp(s, "failsafe")   == 0) return LR_FAILSAFE;
    return 0xFF;
}

static void benchHelp() {
    Serial.println();
    Serial.println("--- ICU BENCH CONSOLE ---");
    Serial.println("  alert <lane> <stage> [pri] [evidence] [eta]");
    Serial.println("      stage    : monitor prepare commit clearing");
    Serial.println("      evidence : evu evu-relay evu-degraded");
    Serial.println("                 acoustic acoustic-degraded acoustic-ambiguous");
    Serial.println("      eta      : seconds, or -1 for unknown");
    Serial.println("  hold <lane>");
    Serial.println("  release <lane> [flag-off|passed|siren-lost|diverged|");
    Serial.println("                  expired|max|operator|failsafe]");
    Serial.println("  health <lane> <node> <far|near|unknown> <state> <dist_m>");
    Serial.println("      state    : healthy suspect failed recovering misconfig linksuspect");
    Serial.println("  clear | stats | tracks | help");
    Serial.println();
    Serial.println("  -- per-junction setup (saved in NVS, survives reflash) --");
    Serial.println("  site                             show current site");
    Serial.println("  site name <text>                 label this junction");
    Serial.println("  site here <lat> <lon>            stop-line position");
    Serial.println("  site bearing <lane> <deg>        approach direction");
    Serial.println("  site export                      one line for your notes");
    Serial.println("  site set <lat> <lon> <b1> <b2> <b3>   paste it back");
    Serial.println("  site clear                       back to compiled defaults");
    Serial.println("  abuse <lane> fill | reset        exercise the rate limit");
    Serial.println();
    Serial.println("  -- acoustic simulation (bench) --");
    Serial.println("  sim <lane> <deltaMs> [conf] [sustained]");
    Serial.println("      +ve = FAR then NEAR  (approaching)");
    Serial.println("      -ve = NEAR then FAR  (departing)");
    Serial.println("      e.g.  sim 1 6000    -> CONFIRMED, ~60 km/h");
    Serial.println("            sim 1 200     -> AMBIGUOUS, one siren two mics");
    Serial.println("            sim 1 -6000   -> OPPOSITE, departing");
    Serial.println("  simfar <lane> | simnear <lane>   single-node observation");
    Serial.println("  simhold <lane>                   re-assert, keeps alert alive");
    Serial.println("  simreset <lane>                  clear the track");
    Serial.println("  window <lane>                    show correlation window");
    Serial.println();
    Serial.println("  -- EVU movement simulation (bench) --");
    Serial.println("  simevu <lane> approach <m> <kmph>   open a track, inbound");
    Serial.println("  simevu <lane> step [sec]            advance the vehicle");
    Serial.println("  simevu <lane> run <n> [sec]         auto-step, keeps track ACTIVE");
    Serial.println("  simevu <lane> speed <kmph>          stop/slow mid-approach (V_FLOOR)");
    Serial.println("  simevu <lane> diverge               U-turn (S2-09)");
    Serial.println("  simevu <lane> teleport <m>          implausible jump (S2-05)");
    Serial.println("  simevu <lane> off | on              emergency flag (S2-08)");
    Serial.println("  simevu <lane> clear");
    Serial.println("      e.g.  simevu 1 approach 500 40");
    Serial.println("            simevu 1 step   (x4)   -> watch ETA fall");
    Serial.println("            simevu 1 diverge; step x3  -> DIVERGED");
    Serial.println("      lanes have SEPARATE vehicles: L1=SIM_01(pri2)");
    Serial.println("      L2=SIM_02(pri1, higher) L3=SIM_03(pri3)");
    Serial.println("      so two approaches can be live at once");
    Serial.printf ("  lanes 1..%d   (1=LEFT 2=BOTTOM 3=RIGHT on the console)\n",
                   GW_NUM_APPROACHES);
    Serial.println();
}

static uint8_t benchParseHealth(const char* s) {
    if (strcasecmp(s, "healthy")      == 0) return 1;
    if (strcasecmp(s, "suspect")      == 0) return 2;
    if (strcasecmp(s, "failed")       == 0) return 3;
    if (strcasecmp(s, "recovering")   == 0) return 4;
    if (strcasecmp(s, "misconfig")    == 0) return 5;
    if (strcasecmp(s, "linksuspect")  == 0) return 6;
    return 0;
}

static void benchExecute(char* line) {
    char* argv[8];
    int argc = 0;

    char* tok = strtok(line, " \t");
    while (tok != nullptr && argc < 8) {
        argv[argc++] = tok;
        tok = strtok(nullptr, " \t");
    }
    if (argc == 0) return;

    // ---- site : per-junction geometry, held in NVS ----
    //
    // Handled before everything else because it is the one command set
    // that must work even when nothing else is configured -- an ICU on
    // an unsurveyed site has no useful behaviour until this is set.
    if (gwSiteCommand(argc, argv)) {
        return;
    }

    // ---- help ----
    if (strcasecmp(argv[0], "help") == 0 || strcmp(argv[0], "?") == 0) {
        benchHelp();
        return;
    }

    // ---- stats ----
    if (strcasecmp(argv[0], "stats") == 0) {
        ercPrintStats();
        gwPrintGeometry();
        gwPrintTracks();
        return;
    }

    // ---- tracks ----
    if (strcasecmp(argv[0], "tracks") == 0) {
        gwPrintTracks();
        return;
    }

    // ---- clear ----
    if (strcasecmp(argv[0], "clear") == 0) {
        ercClearAll(LR_OPERATOR);
        Serial.println("[BENCH] cleared all approaches");
        return;
    }

    // Everything below needs a lane. Validate it once, here, rather than
    // in each branch -- a lane that does not exist must never reach
    // ercAlert(), which would send an alert naming no approach at all.
    if (argc < 2) { benchHelp(); return; }

    int lane = atoi(argv[1]);
    if (lane < 1 || lane > GW_NUM_APPROACHES) {
        Serial.printf("[BENCH] lane %d out of range (1..%d)\n",
                      lane, GW_NUM_APPROACHES);
        return;
    }

    // ---- alert ----
    if (strcasecmp(argv[0], "alert") == 0) {
        if (argc < 3) {
            Serial.println("[BENCH] alert <lane> <stage> [pri] [evidence] [eta]");
            return;
        }

        uint8_t stage = benchParseStage(argv[2]);
        if (stage == 0xFF) {
            Serial.printf("[BENCH] unknown stage \"%s\"\n", argv[2]);
            return;
        }

        uint8_t pri = (argc > 3) ? (uint8_t)atoi(argv[3]) : 1;

        uint8_t ev = LE_ACOUSTIC_CONFIRMED;
        if (argc > 4) {
            ev = benchParseEvidence(argv[4]);
            if (ev == 0xFF) {
                Serial.printf("[BENCH] unknown evidence \"%s\"\n", argv[4]);
                return;
            }
        }

        int16_t eta = (argc > 5) ? (int16_t)atoi(argv[5]) : -1;

        ercAlert(lane, stage, pri, ev, eta);
        Serial.printf("[BENCH] alert lane=%d stage=%s pri=%u ev=%s eta=%d\n",
                      lane, gwStageName(stage), (unsigned)pri,
                      gwEvidenceName(ev), (int)eta);
        return;
    }

    // ---- hold ----
    if (strcasecmp(argv[0], "hold") == 0) {
        ercHold(lane);
        Serial.printf("[BENCH] hold lane=%d\n", lane);
        return;
    }

    // ---- release ----
    if (strcasecmp(argv[0], "release") == 0) {
        uint8_t reason = LR_OPERATOR;
        if (argc > 2) {
            reason = benchParseReason(argv[2]);
            if (reason == 0xFF) {
                Serial.printf("[BENCH] unknown reason \"%s\"\n", argv[2]);
                return;
            }
        }
        ercRelease(lane, reason);
        Serial.printf("[BENCH] release lane=%d reason=%s\n",
                      lane, gwReleaseName(reason));
        return;
    }

    // ---- sim : inject a synthetic approach ----
    //
    //   sim <lane> <deltaMs> [conf] [sustained]
    //
    // Positive delta = FAR then NEAR = approaching.
    // Negative delta = NEAR then FAR = departing.
    //
    // Goes through gwTrackOnAcoustic(), the same entry point a real
    // authenticated frame reaches -- only the two timestamps are
    // synthetic. See the note in ICU_Tracks.ino for why a bench with
    // both nodes on one desk cannot produce this quantity physically.
    if (strcasecmp(argv[0], "sim") == 0) {
        if (argc < 3) {
            Serial.println("[SIM] sim <lane> <deltaMs> [conf] [sustained]");
            Serial.println("      +ve delta = approaching, -ve = departing");
            return;
        }

        long delta = atol(argv[2]);
        float conf = (argc > 3) ? atof(argv[3]) : 0.90f;
        bool  sust = (argc > 4) ? (atoi(argv[4]) != 0) : true;

        Serial.printf("[SIM] L%d injecting delta=%ldms conf=%.2f sustained=%d\n",
                      lane, delta, conf, sust ? 1 : 0);
        gwSimApproach(lane, delta, conf, sust);
        gwPrintTracks();
        return;
    }

    // ---- simfar / simnear : single-node observation ----
    if (strcasecmp(argv[0], "simfar") == 0 ||
        strcasecmp(argv[0], "simnear") == 0) {
        bool useFar = (strcasecmp(argv[0], "simfar") == 0);
        float conf = (argc > 2) ? atof(argv[2]) : 0.90f;

        Serial.printf("[SIM] L%d injecting %s-only observation\n",
                      lane, useFar ? "FAR" : "NEAR");
        gwSimSingle(lane, useFar, conf, true);
        gwPrintTracks();
        return;
    }

    // ---- simhold : keep a live track alive ----
    //
    // A single sim injection goes quiet and is correctly released as
    // SIREN_LOST after TRACK_SILENCE_RELEASE_MS. Run this every ~10 s to
    // hold an alert up, the way a real RDU's 8 s re-assert does.
    if (strcasecmp(argv[0], "simhold") == 0) {
        gwSimReassert(lane);
        return;
    }

    // ---- simevu : synthetic vehicle movement ----
    //
    // Exercises the three defects that need a MOVING vehicle and
    // therefore cannot be reached with the EVU sitting on a desk:
    // position plausibility (S2-05), divergence (S2-09) and approach
    // association (S2-14).
    //
    // Runs through gwEvuOnRelay() -- the real entry point. Only the
    // coordinates are synthetic.
    if (strcasecmp(argv[0], "simevu") == 0) {
        if (argc < 3) {
            Serial.println("[SIMEVU] simevu <lane> approach <dist_m> <kmph>");
            Serial.println("         simevu <lane> step [seconds]");
            Serial.println("         simevu <lane> run <count> [sec]  auto-step");
            Serial.println("         simevu <lane> speed <kmph>  change speed, keep track");
            Serial.println("         simevu <lane> diverge");
            Serial.println("         simevu <lane> teleport <metres>");
            Serial.println("         simevu <lane> off | on");
            Serial.println("         simevu <lane> clear");
            return;
        }

        const char* sub = argv[2];

        if (strcasecmp(sub, "approach") == 0) {
            float d = (argc > 3) ? atof(argv[3]) : 500.0f;
            float v = (argc > 4) ? atof(argv[4]) : 40.0f;
            gwSimEvuApproach(lane, d, v);
        } else if (strcasecmp(sub, "step") == 0) {
            float t = (argc > 3) ? atof(argv[3]) : 10.0f;
            gwSimEvuStep(lane, t);
        } else if (strcasecmp(sub, "speed") == 0) {
            float k = (argc > 3) ? atof(argv[3]) : 0.0f;
            gwSimEvuSpeed(lane, k);
        } else if (strcasecmp(sub, "diverge") == 0) {
            gwSimEvuDiverge(lane);
        } else if (strcasecmp(sub, "teleport") == 0) {
            float m = (argc > 3) ? atof(argv[3]) : 3000.0f;
            gwSimEvuTeleport(lane, m);
        } else if (strcasecmp(sub, "off") == 0) {
            gwSimEvuEmergency(lane, false);
        } else if (strcasecmp(sub, "on") == 0) {
            gwSimEvuEmergency(lane, true);
        } else if (strcasecmp(sub, "run") == 0) {
            int   n = (argc > 3) ? atoi(argv[3]) : 4;
            float t = (argc > 4) ? atof(argv[4]) : 2.0f;
            gwSimEvuRun(lane, n, t);
        } else if (strcasecmp(sub, "clear") == 0) {
            gwSimEvuClear(lane);
        } else {
            Serial.printf("[SIMEVU] unknown sub-command \"%s\"\n", sub);
            return;
        }

        gwPrintEvuTracks();
        return;
    }

    // ---- simreset ----
    if (strcasecmp(argv[0], "simreset") == 0) {
        gwSimReset(lane);
        Serial.printf("[SIM] L%d track cleared\n", lane);
        return;
    }

    // ---- window : show the correlation window, inject nothing ----
    if (strcasecmp(argv[0], "window") == 0) {
        gwPrintWindow(lane);
        return;
    }

    // ---- abuse : exercise the rate limit ----
    //
    //   abuse <lane> fill    pre-load to one short of the limit
    //   abuse <lane> reset   clear the counter
    //
    // See gwAbuseFill() for why the limit cannot be reached by hand.
    if (strcasecmp(argv[0], "abuse") == 0) {
        if (argc < 3) {
            Serial.println("[ABUSE] abuse <lane> fill | reset");
            return;
        }
        if (strcasecmp(argv[2], "fill") == 0)       gwAbuseFill(lane);
        else if (strcasecmp(argv[2], "reset") == 0) gwAbuseReset(lane);
        else Serial.println("[ABUSE] expected fill or reset");
        return;
    }

    // ---- health ----
    if (strcasecmp(argv[0], "health") == 0) {
        if (argc < 6) {
            Serial.println("[BENCH] health <lane> <node> <role> <state> <dist_m>");
            return;
        }

        int node = atoi(argv[2]);
        if (node < 1 || node > GW_NODES_PER_APPROACH) {
            Serial.printf("[BENCH] node %d out of range (1..%d)\n",
                          node, GW_NODES_PER_APPROACH);
            return;
        }

        uint8_t role = 0;
        if (strcasecmp(argv[3], "far")  == 0) role = 1;
        if (strcasecmp(argv[3], "near") == 0) role = 2;

        uint8_t st = benchParseHealth(argv[4]);
        if (st == 0) {
            Serial.printf("[BENCH] unknown state \"%s\"\n", argv[4]);
            return;
        }

        uint16_t dist = (uint16_t)atoi(argv[5]);

        ercHealth(lane, node, role, st, dist);
        Serial.printf("[BENCH] health L%dN%d %s %s %um\n",
                      lane, node, gwRoleName(role), gwHealthName(st),
                      (unsigned)dist);
        return;
    }

    Serial.printf("[BENCH] unknown command \"%s\" -- type help\n", argv[0]);
}

// Call from loop(). Non-blocking.
void ercBenchConsole() {
    static char buf[80];
    static uint8_t len = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();

        if (c == '\r') continue;

        if (c == '\n') {
            buf[len] = '\0';
            if (len > 0) benchExecute(buf);
            len = 0;
            continue;
        }

        if (len < sizeof(buf) - 1) {
            buf[len++] = c;
        } else {
            // Overlong input is discarded whole rather than truncated.
            // A half-parsed "release" is worse than no command at all.
            len = 0;
            Serial.println("[BENCH] line too long, discarded");
        }
    }
}
