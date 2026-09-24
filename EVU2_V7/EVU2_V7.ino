/*
============================================================================
GREENWAVE EVP TRANSMITTER V7 — COMPACT WIRE + TWO-TIER CERTIFICATES
ESP32 DEV MODULE + SX1278 LoRa + u-blox MAX-M10S GNSS

V5 CHANGES (see CHANGELOG block at the bottom of this header):
  - Static-lock machine DELETED          (was the single largest error source)
  - EMA position filter DELETED          (added ~7 m of along-track lag)
  - GNSS now configured over UBX at boot (Automotive model, 10 Hz, 115200)
  - Fix gating on hAcc + fixType, not HDOP
  - gps_epoch now timestamps the FIX, not packet assembly
  - heading_valid flag added (flags bit 4) — no wire-layout change
  - fix age exposed so the RDU/ICU can dead-reckon
  - UART RX buffer enlarged; diagnostic dump gated behind RANGE_TEST_BUILD
  - Arduino String removed from the hot path

WIRE FORMAT: unchanged by default. TelemetryPayload is byte-for-byte
identical to V4 and to GreenwaveTypes.h / ICU_EvuTypes.h. Only previously
reserved bits of the existing `flags` byte are now used. Signatures continue
to verify against unmodified RDU/ICU trees.

REQUIRED LIBRARY CHANGE:
  TinyGPS++  --->  SparkFun u-blox GNSS Arduino Library v3
  ("SparkFun u-blox GNSS v3" in Library Manager)
  TinyGPS++ is an NMEA parser only. It cannot send configuration to the
  module and cannot read hAcc, fixType or the fix epoch. Those three are
  the whole point of this revision.

  >> VERIFY: accessor names below (getMillisecond, getHeadingAccEst,
  >> getHorizontalAccEst) are from the v3 API. If your installed library
  >> version differs, check the names against its Example files before
  >> flashing. Everything else in this sketch is library-independent.
============================================================================
*/

#include <SPI.h>
#include <LoRa.h>
#include <math.h>
#include <Preferences.h>
#include <Ed25519.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// ---- V7 ----
// The wire format and the certificate format now live in ONE file each,
// copied verbatim into the RDU and ICU trees. V5 hand-duplicated the payload
// struct across three sketch folders with nothing checking they agreed; the
// only symptom of drift was universal signature failure with nothing in any
// log naming the cause.
#include "GreenwaveWireV6.h"
#include "GreenwaveCertV1.h"

// ESP32 hardware entropy. See gwGatherSeed() for why esp_fill_random() alone
// is NOT sufficient on this board.
#include "esp_random.h"
#include "bootloader_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#if __has_include("esp_mac.h")
  #include "esp_mac.h"          // Arduino-ESP32 3.x / IDF 5.x
#endif

// mbedTLS renamed these between IDF 4.x (mbedtls 2.x) and IDF 5.x (mbedtls 3.x).
// Arduino-ESP32 2.x ships the former, 3.x the latter, and picking the wrong one
// is a link error rather than anything subtle — but it is a link error that
// lands on whoever next opens the project on a different core version.
#if defined(MBEDTLS_VERSION_MAJOR) && (MBEDTLS_VERSION_MAJOR >= 3)
  #define GW_SHA256_STARTS(c, is224)   mbedtls_sha256_starts((c), (is224))
  #define GW_SHA256_UPDATE(c, b, l)    mbedtls_sha256_update((c), (b), (l))
  #define GW_SHA256_FINISH(c, o)       mbedtls_sha256_finish((c), (o))
#else
  #define GW_SHA256_STARTS(c, is224)   mbedtls_sha256_starts_ret((c), (is224))
  #define GW_SHA256_UPDATE(c, b, l)    mbedtls_sha256_update_ret((c), (b), (l))
  #define GW_SHA256_FINISH(c, o)       mbedtls_sha256_finish_ret((c), (o))
#endif

// ============================================================================
// HARDWARE PINS & PERIPHERALS
// ============================================================================
#define LORA_SS      5
#define LORA_RST     14
#define LORA_DIO0    2
#define LORA_SCK     18
#define LORA_MISO    19
#define LORA_MOSI    23

#define GPS_RX       16
#define GPS_TX       17

HardwareSerial GPSSerial(2);
SFE_UBLOX_GNSS_SERIAL myGNSS;

// FIX 9a (revised V5.1): the default HardwareSerial RX buffer is 256 bytes.
// A UBX-NAV-PVT frame is ~100 bytes and at 10 Hz they arrive continuously.
//
// The binding constraint is LoRa.endPacket(), which BLOCKS for the whole
// time-on-air. The certificate frame is 1181 ms:
//     10 Hz x 1.181 s x ~100 B  =  ~1180 bytes arriving while we parse nothing
// 1024 was sized against the 640 ms steady frame and overflowed on every 5th
// packet (visible as a 3 s uptime gap on every 193 B transmission).
// 2048 covers the cert frame with margin.
//
// Eliminating the on-air certificate removes this blocking window entirely,
// which is a second reason to do it beyond the airtime saving.
#define GPS_RX_BUFFER_BYTES  2048

// ============================================================================
// VEHICLE DATA & DEPLOYMENT PARAMETERS
// ============================================================================
// V6: numeric fleet ID. The human-readable label is kept for LOGS ONLY and is
// never transmitted. The mapping AMB_02 <-> 2 lives in the enrolment ledger on
// the provisioning laptop, not on any device.
//
// The ID is NOT in the steady frame. It travels once inside the certificate,
// where the issuing CA's signature over the TBS already binds it to this
// device's public key. The RDU caches pubkey -> vehicle_id and attributes every
// subsequent frame by WHICH CACHED KEY VERIFIED THE SIGNATURE.
//
// This is a security property, not just a saving: a certified low-priority
// vehicle can no longer assert another vehicle's ID in the payload, because the
// field is not in the payload.
//
// NOTE: this constant is now only used for the provisioning hint printed at
// boot. The authoritative vehicle_id is the one inside the stored certificate.
#define VEHICLE_ID_NUM    ((uint16_t)2)
#define VEHICLE_ID_LABEL  "AMB_02"      // logs only — never on the wire

static_assert(VEHICLE_ID_NUM != 0, "vehicle ID 0 is reserved for unassigned");

// SCENARIO 2 / S2-08 : the emergency flag must be able to fall. Unchanged
// from V4 — this is runtime state, not a compile-time constant, so the
// falling edge exists and the ICU can treat it as an explicit release.
bool sirenActive = true;

// [FIELD] In deployment this MUST be driven by the same switch that operates
// the light bar. -1 leaves the flag under serial control for bench work.
#define EMERGENCY_SWITCH_PIN  -1
#define EMERGENCY_SWITCH_ACTIVE_LOW  true

// ==========================================================
// PHASE 6.2 : IN-CAB ACKNOWLEDGEMENT -- NOT IMPLEMENTED, BY DESIGN
//
// Unchanged from V4 and deliberately so. This unit is TRANSMIT-ONLY. The
// trust model is one-directional: the EVU signs, the roadside verifies,
// nothing travels back. The vehicle holds no key capable of authenticating
// an inbound message.
//
// A downlink was prototyped and removed. It worked, and it would have been
// unauthenticated — anybody with a LoRa module and the frame layout could
// have transmitted a status into an ambulance cab. A forged reassurance
// that a driver acts on is worse than showing nothing at all.
//
// SO THE CAB IS TOLD NOTHING, deliberately.
// ==========================================================

// Confirmed 2026-07-26: 2000 ms is the correct transmission interval.
// DO NOT REDUCE THIS. printDutyCycle() reports ~37% already, which is
// outside any licence-exempt allowance the team has verified. Halving the
// interval doubles it. Latency is fixed by receiver-side dead reckoning
// (see "DEAD RECKONING CONTRACT" below), never by transmitting faster.
const unsigned long TX_INTERVAL_MS = 2000;
unsigned long previousTX = 0;

// RANGE_TEST_BUILD gates the range-test instrumentation AND (new in V5) the
// verbose diagnostic dump. Set to 0 for anything that leaves the bench.
#define RANGE_TEST_BUILD 1

// ---- V5.3 FIELD-TEST SUPPORT ----
// CSV_LOG_MODE 1 replaces the 25-line verbose dump with ONE comma-separated
// line per packet. Vastly easier to capture and analyse: select all in the
// serial monitor, paste into a .csv, done. Also ~10x less serial blocking,
// which matters because Serial.printf blocks the GNSS parser.
// Set to 0 to get the human-readable dump back.
// Set to 0 to restore the V5 block dump (the "========== GREENWAVE TX =========="
// form). Set to 1 for the one-line CSV that calibrate_from_log.py consumes.
#define CSV_LOG_MODE 0

// Radio parameters are now #defines so a field test is a one-line edit
// instead of hunting through setup(). MUST MATCH THE RDU EXACTLY — SF, BW,
// CR, preamble and sync word mismatches do not degrade gracefully, they just
// fail to link.
#define LORA_FREQ_HZ        433E6
#define LORA_SF                 9    // 9 = validated. 8 = untested, worth testing.
#define LORA_BW_HZ          125E3
#define LORA_CR_DENOM           6    // 6 = CR4/6. 5 = CR4/5, less FEC, faster.
#define LORA_PREAMBLE          12    // 8 is likely safe: the RDU never sleeps.
#define LORA_TX_POWER_DBM      20
#define LORA_SYNC_WORD       0xF3

// V7: TOA_STEADY_MS / TOA_CERT_MS are GONE.
//
// They were hand-maintained #defines describing a 97-byte and a 193-byte frame.
// Any change to the struct or to the radio parameters silently invalidated
// them, and the thing they invalidated was the duty-cycle audit — the one
// mechanism whose entire job is to stop a frame-size change going unnoticed.
//
// Airtime is now computed from sizeof() and the live LORA_* settings by
// gwLoRaAirtimeMs() in GreenwaveWireV6.h. It cannot go stale.
//
// Cross-check: that function reproduces both measured V5 figures exactly
// (97 B -> 640.0 ms, 193 B -> 1180.7 ms), so it is calibrated against this
// hardware and not merely against the datasheet.

// ============================================================================
// GNSS ACCEPTANCE GATES  (FIX 4 + FIX 5)
// ============================================================================
// V4 gated on satellite count and HDOP. Both were the wrong instrument:
//
//   HDOP describes SATELLITE GEOMETRY ONLY. It is completely blind to
//   multipath — the dominant error next to buildings, buses and overpasses,
//   i.e. exactly where this system operates. A vehicle beside a glass
//   facade can show excellent HDOP and a 20 m position error.
//
//   hAcc is the receiver's own horizontal accuracy estimate in millimetres,
//   computed from its internal covariance. It reflects actual uncertainty.
//
// Tune MAX_HACC_MM at YOUR intersections. Start loose, tighten on measured
// data. Do not copy a number out of a datasheet.
#define MAX_HACC_MM            10000UL   // 10.0 m — hard accept/reject gate
#define PRECISION_HACC_MM       3500UL   // 3.5 m — sets the "high precision" flag bit
#define MIN_FIX_TYPE                3    // 3 = 3D fix
// Sanity floor only. hAcc is the real gate and is strictly better: it
// reflects actual solution uncertainty including multipath, whereas satellite
// count does not. Lowered from 6 to 4 in V5.1 because 6 was sitting exactly
// at the observed count and one dropped SV would have flipped gps_valid to
// false on a fix whose hAcc was perfectly acceptable.
#define MIN_SATELLITES              4

// FIX 5: V4 accepted a fix up to 2000 ms old. At 40 km/h that alone is 22 m
// of error before the packet is even transmitted. At 10 Hz a fresh fix is
// never more than ~100 ms old, so 250 ms is generous.
#define MAX_FIX_AGE_MS            250UL

// GNSS course-over-ground is derived from the velocity vector and is
// essentially random at low speed. Below this, heading is marked INVALID
// rather than being coerced to 0.00 degrees (= due North, which V4 did and
// which is indistinguishable from a genuine northward heading).
#define HEADING_VALID_MIN_KMPH     5.0
#define HEADING_MAX_ACC_DEG       30.0

// GNSS runtime configuration
#define GNSS_NAV_RATE_HZ            10
#define GNSS_TARGET_BAUD        115200
#define GNSS_DEFAULT_BAUD         9600

// ============================================================================
// GNSS STATE
// ============================================================================
// FIX 1 + FIX 2: filteredLat/filteredLon/lockedLat/lockedLon/positionLocked/
// stationaryCounter/movementCounter are GONE. See the CHANGELOG.
//
// What remains is last-known-good, which is NOT a filter. It is only used so
// that a packet with gps_valid CLEAR carries the last real position instead
// of 0,0. 0,0 is a valid coordinate (Gulf of Guinea) and a consumer that
// forgets to check the flag would silently believe it.
//
// >> CONSUMERS MUST CHECK flags BIT 1. Position is meaningless without it.
double   lastGoodLat      = 0;
double   lastGoodLon      = 0;
bool     haveEverHadFix   = false;

// Set by the PVT callback path; used to compute true fix age.
unsigned long lastPvtMillis = 0;
bool          pvtFresh      = false;

// ============================================================================
// CRYPTOGRAPHIC STATE & NVS COUNTERS
// ============================================================================
Preferences secPrefs;
uint8_t evuPrivateKey[32];
uint8_t evuPublicKey[32];

// ---- CERTIFICATE SCHEDULING (V5.2) ----
// The certificate does NOT authenticate packets — the per-packet Ed25519
// signature does that, on every single frame. The certificate only delivers
// the public key so an RDU can cache it (spec §2.1). Once cached, resending
// it is pure airtime.
//
// So: burst on activation (when a cold RDU might first hear us), then back
// off hard.
#define CERT_BURST_COUNT            3    // certs sent immediately after siren ON
// Changed from 30 to 5 deliberately. This COSTS airtime and buys cold-start
// reliability, which is a defensible trade but should be a conscious one:
//
//   n=30   duty 29.30%   cold RDU waits up to 60 s   1 cert/min
//   n=5    duty 34.21%   cold RDU waits up to 10 s   6 certs/min
//
// The argument FOR 5: an ambulance at 40 km/h is inside a 300 m detection
// radius for roughly 27 seconds. At n=30 an RDU that has never heard this
// vehicle — a new unit, a rebooted unit, or one that came up mid-run and so
// missed the CERT_BURST_COUNT burst at dispatch — may never receive a
// certificate during the entire approach, and every frame it buffers for
// retro-verification expires unverified. At n=5 it gets one within 10 s,
// comfortably inside the window.
//
// The argument AGAINST: +4.91 percentage points of duty cycle on a figure
// that already fails every licence-exempt allowance the team has verified,
// and six blocking 1156 ms transmissions per minute instead of one, each of
// which starves the GNSS UART and narrows the RDU's transmit gap on IF-2.
//
// This trade only makes sense while OPEN-003 is unresolved and the duty cycle
// is not yet the binding constraint. If 865-867 MHz turns out to impose a
// duty-cycle limit, revisit this first — it is the cheapest 4.91 points in
// the file.
#define CERT_INTERVAL_STEADY        5    // every 5th packet (~10 s) thereafter
#define SEQ_SAVE_MARGIN         100
#define SEQ_SAVE_INTERVAL       60   // 60 for 100-hour endurance test (flash wear)

// Counts down from CERT_BURST_COUNT on every rising edge of sirenActive.
uint8_t certBurstRemaining = CERT_BURST_COUNT;

uint32_t sequenceNumber  = 0;
uint32_t seqSinceLastSave = 0;

// V7: the stored certificate is a 108-byte GwLeafCert, not a bare 64-byte CA
// signature. The v0 format signed the ASCII string "AMB_02:<pubkey hex>" with
// the ROOT key and carried no version, issuer, serial or validity period.
// Certificates issued under it are worthless here and are not migrated.
uint8_t evuCertificate[sizeof(GwLeafCert)];
bool    certProvisioned = false;

// Provisioning lock. Once set, SETCERT is refused unless the provisioning
// jumper is fitted at boot. An EVU sits in a vehicle for years with a USB port
// on it; leaving the write path permanently open is an unnecessary gift.
bool    provisioningLocked = false;
#define PROVISION_JUMPER_PIN  -1    // set to a real GPIO, pulled LOW to unlock
#define IDENTITY_FATAL_HALT   1     // 1 = refuse to run without a good RNG

// FIX 9b: was `String serialLineBuffer`. Arduino String heap-allocates on
// every += . Over a 100-hour endurance run that fragments the heap, and the
// failure presents as an unexplained reset with nothing in the log. Fixed
// buffer, no allocation.
// V7 BUGFIX: was 160, which silently truncated SETCERT.
//
// The V5 certificate was 64 bytes, so "SETCERT " + 128 hex = 136 chars fitted
// with room to spare. The V7 certificate is 108 bytes: "SETCERT " + 216 hex =
// 224 chars. Everything past character 159 was DROPPED, and hexToBytes() then
// correctly reported a length failure — about a string the operator never
// typed. The error message was true and useless.
//
// Sized off sizeof(GwLeafCert) so it tracks the certificate automatically
// rather than needing a human to remember. 8 for "SETCERT ", 2 per byte, plus
// slack for whitespace and the NUL.
#define SERIAL_LINE_MAX  (16 + sizeof(GwLeafCert) * 2 + 8)
char   serialLine[SERIAL_LINE_MAX];
size_t serialLineLen = 0;
bool   serialLineOverflow = false;

static_assert(SERIAL_LINE_MAX >= 8 + sizeof(GwLeafCert) * 2 + 1,
              "serial line buffer cannot hold a SETCERT command");

// ============================================================================
// RANGE-TEST LEG TAGGING
// Set with "LEG <n>" (0-255) before starting a distance leg.
// ============================================================================
uint8_t currentTestLeg = 0;

// ============================================================================
// TIMEZONE CONFIG — INDIA STANDARD TIME (UTC+5:30)
// ============================================================================
#define IST_OFFSET_SEC   19800UL   // 5*3600 + 30*60
#define IST_LABEL        "IST"

// ============================================================================
// PACKED BINARY LAYOUT — UNCHANGED FROM V4
// ============================================================================
// *** V7 IS A FLAG DAY. THE WIRE FORMAT AND THE CERTIFICATE BOTH CHANGED. ***
//
//   payload      33 B -> 20 B     steady frame  97 B -> 84 B, 640.0 -> 566.3 ms
//   certificate  96 B -> 108 B    cert   frame 193 B -> 192 B, 1180.7 -> 1156.1 ms
//   duty cycle   32.90% -> 34.21% at CERT_INTERVAL_STEADY = 5
//                (the compaction alone gives 29.30% at n=30; the cert
//                 interval was then set to 5 for cold-start reliability)
//
// Every device reflashed together, ICU first. Copy GreenwaveWireV6.h and
// GreenwaveCertV1.h verbatim into the RDU and ICU trees in the SAME commit and
// run ../verify_rdu_tree.py. proto_version exists so a mismatch is DIAGNOSABLE,
// not so it is survivable — there is no partial upgrade.
//
// WHAT CHANGED IN THE PAYLOAD (full justification in GreenwaveWireV6.h):
//   vehicle_id      6 B -> 0 B. Now inside the certificate, where the CA
//                   signature already binds it to this device's public key.
//   priority_class  1 B -> 2 bits of flags. Advisory only since S2-01; the
//                   AUTHORITATIVE class is now CA-signed inside the cert.
//   gps_epoch+ms    6 B -> 3 B GPS time-of-week at 100 ms. gps_ms was 16 bits
//                   holding ten distinct values, because nav rate is 10 Hz.
//   lat/lon         8 B -> 6 B, int24 at 1e-5 deg = 1.11 m. MAX_HACC_MM is
//                   10 m, so the old encoding resolved 300x finer than the
//                   best fix this firmware will accept.
//   speed           2 B -> 1 B at 0.5 km/h. Sensor noise is ~0.18 km/h.
//   heading         2 B -> 1 B at 1.41 deg. HEADING_MAX_ACC_DEG is 30.0.
//   test_leg        1 B -> 0 B. Instrumentation, off the wire entirely.
//   quality         NEW 1 B: hAcc + fix age, which PROTOCOL_V2 wanted at 4 B.
//   proto_version   NEW 1 B. The version byte V5 did not have.
//   altitude_m      UNCHANGED int16 — bytes 19-20 are free inside the airtime
//                   bucket, so full precision costs nothing.
//
// flags bit assignment (V6):
//   bit 0    emergency
//   bit 1    gps_valid        <-- position is meaningless if this is clear
//   bit 2    altitude_valid   <-- MOVED from bit 3
//   bit 3    heading_valid    <-- MOVED from bit 4
//   bit 4    precision_high   <-- MOVED from bit 5
//   bits 5-6 priority_class (advisory)
//   bit 7    reserved
typedef TelemetryPayloadV6 TelemetryPayload;

// Flag names forwarded so the diagnostics below read unchanged.
// FLAG_SIREN is GONE: V5 set it and FLAG_EMERGENCY from the same sirenActive
// variable (two bits, one fact). Reclaiming bit 2 is what lets priority_class
// move off its own byte and into flags bits 5-6.
#define FLAG_EMERGENCY      GW6_FLAG_EMERGENCY
#define FLAG_GPS_VALID      GW6_FLAG_GPS_VALID
#define FLAG_ALT_VALID      GW6_FLAG_ALT_VALID
#define FLAG_HEADING_VALID  GW6_FLAG_HEADING_VALID
#define FLAG_PRECISION_HIGH GW6_FLAG_PRECISION_HIGH

// ============================================================================
// DEAD RECKONING CONTRACT  (FIX 8 — the RDU/ICU side of the fix)
// ============================================================================
// The 2000 ms transmit cadence cannot be reduced (duty cycle). So the
// remaining ~50 m of delay error must be removed at the RECEIVER, not here.
//
// The RDU/ICU must project the received position forward before testing it
// against any geofence:
//
//     dt      = t_now - gps_epoch.gps_ms          (seconds)
//     bearing = heading_deg / 100                 (only if flags bit 4 set)
//     v       = speed_kmph / 100 / 3.6            (m/s)
//     p_now   = p_fix + v * dt * unit_vector(bearing)
//
// This is only correct because V5 fixes three things V4 got wrong:
//   1. gps_epoch now timestamps the FIX, so dt starts from the right origin
//   2. heading has a validity bit, so a bogus North is not extrapolated along
//   3. speed is the real speed — V4's static lock transmitted 0 during the
//      exact acceleration phase this equation depends on
//
// Do not implement dead reckoning until all three are deployed. Doing it on
// V4 data amplifies error instead of removing it.
//
// REQUIREMENT: this needs t_now and gps_epoch on a COMMON clock. If the RDU
// has its own GNSS, use its UTC. If it does not, you need the fix age on the
// wire — see PROTOCOL_V2 below, which is a coordinated 3-tree change and is
// deliberately NOT enabled by default.
// ============================================================================

// ============================================================================
// OPTIONAL: PROTOCOL V2 — NOT ENABLED. Read before switching on.
// ============================================================================
// Enabling this CHANGES THE WIRE FORMAT and will fail every signature until
// GreenwaveTypes.h and ICU_EvuTypes.h are updated in the same commit and
// verify_rdu_tree.py passes. There is currently NO version byte in the
// protocol, so there is no graceful degradation and no diagnostic — a
// mismatched node simply rejects everything.
//
// Adds: proto_version, hacc_cm, fix_age_ms.
// Do this ONCE, deliberately, while there are only three consumers. It gets
// harder every unit you deploy.
#define PROTOCOL_V2 0

#if PROTOCOL_V2
struct __attribute__((packed)) TelemetryPayloadV2 {
  uint8_t  proto_version;   // = 2. Add this now; retrofitting it is worse.
  char     vehicle_id[6];
  uint8_t  flags;
  uint8_t  priority_class;
  uint32_t seq;
  uint32_t gps_epoch;
  uint16_t gps_ms;
  int32_t  latitude;
  int32_t  longitude;
  int16_t  altitude_m;
  uint16_t speed_kmph;
  uint16_t heading_deg;
  uint16_t hacc_cm;         // horizontal accuracy estimate, centimetres
  uint16_t fix_age_ms;      // age of the fix at transmit — enables DR with no shared clock
  uint8_t  test_leg;
};
#endif

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
bool hexToBytes(const char *hex, size_t hexLen, uint8_t *out, size_t outLen) {
  if (hexLen != outLen * 2) return false;
  for (size_t i = 0; i < outLen; i++) {
    char hi = hex[i * 2];
    char lo = hex[i * 2 + 1];
    if (!isxdigit((int)hi) || !isxdigit((int)lo)) return false;
    auto nib = [](char c) -> uint8_t {
      if (c >= '0' && c <= '9') return c - '0';
      c = tolower(c);
      return 10 + (c - 'a');
    };
    out[i] = (nib(hi) << 4) | nib(lo);
  }
  return true;
}

// FIX 9b: was `String bytesToHex(...)`. Caller-supplied buffer, no heap.
void bytesToHex(const uint8_t *data, size_t len, char *out, size_t outLen) {
  static const char hexChars[] = "0123456789abcdef";
  if (outLen < len * 2 + 1) { if (outLen) out[0] = 0; return; }
  for (size_t i = 0; i < len; i++) {
    out[i * 2]     = hexChars[(data[i] >> 4) & 0x0F];
    out[i * 2 + 1] = hexChars[data[i] & 0x0F];
  }
  out[len * 2] = 0;
}

static long daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  long era = (y >= 0 ? y : y - 399) / 400;
  unsigned yoe = (unsigned)(y - era * 400);
  unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097L + (long)doe - 719468L;
}

static void civilFromDays(long z, int &y, int &m, int &d) {
  z += 719468L;
  long era = (z >= 0 ? z : z - 146096L) / 146097L;
  unsigned long doe = (unsigned long)(z - era * 146097L);
  unsigned long yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  long yy = (long)yoe + era * 400;
  unsigned long doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  unsigned long mp = (5 * doy + 2) / 153;
  d = (int)(doy - (153 * mp + 2) / 5 + 1);
  m = (int)(mp + (mp < 10 ? 3 : -9));
  y = (int)(yy + (m <= 2 ? 1 : 0));
}

// Retained for reference and for any future consumer. V7 no longer calls it:
// the wire carries GPS time-of-week, and the IST diagnostic line is built from
// gpsEpoch, which is computed inline from daysFromCivil().
uint32_t civilToEpoch(int year, int month, int day,
                      int hour, int minute, int second) {
  long days = daysFromCivil(year, month, day);
  return (uint32_t)days * 86400UL
       + (uint32_t)hour   * 3600UL
       + (uint32_t)minute * 60UL
       + (uint32_t)second;
}

void formatEpochAsIST(uint32_t utcEpoch, uint16_t ms, char *outBuf, size_t outBufLen) {
  uint32_t istTotalSec = utcEpoch + IST_OFFSET_SEC;
  long     days     = (long)(istTotalSec / 86400UL);
  uint32_t secOfDay = istTotalSec % 86400UL;

  int y, mo, da;
  civilFromDays(days, y, mo, da);
  int hh = secOfDay / 3600;
  int mm = (secOfDay % 3600) / 60;
  int ss = secOfDay % 60;
  snprintf(outBuf, outBufLen, "%04d-%02d-%02d %02d:%02d:%02d.%03d %s",
           y, mo, da, hh, mm, ss, ms, IST_LABEL);
}

// ============================================================================
// ENTROPY  (V7 — OPEN ITEM 1)
// ============================================================================
// V5 called Ed25519::generatePrivateKey() with no RNG initialisation anywhere
// in the file. TWO separate things were wrong, and fixing only one of them
// fixes nothing.
//
// PROBLEM 1 — esp_random() is not a TRNG unless an entropy source is running.
//   On ESP32 the hardware RNG is fed by noise from the RF subsystem or from
//   the SAR ADC. This board runs neither WiFi nor Bluetooth, so with no ADC
//   entropy enabled the peripheral degrades to a PSEUDO-random sequence.
//   ESP-IDF is explicit about this. bootloader_random_enable() switches the
//   SAR ADC on as an entropy source for exactly this case.
//
//   Safe here because the EVU uses only SPI, UART and GPIO. It would NOT be
//   safe on the RDU, which drives an INMP441 over I2S — bootloader_random_*
//   conflicts with ADC and I2S use.
//
// PROBLEM 2 — Ed25519::generatePrivateKey() does not call esp_random() at all.
//   It draws from rweather Crypto's own RNG object, which needs RNG.begin()
//   and stirring or it returns its default state. So adding esp_fill_random()
//   next to generatePrivateKey() would change nothing whatsoever.
//
//   The fix is not to seed that RNG, it is to REMOVE IT FROM THE TRUST PATH.
//   An Ed25519 private key IS 32 uniformly random bytes — clamping happens
//   inside derivePublicKey(). So we fill 32 bytes ourselves and derive.
//   generatePrivateKey() is never called again.
//
//   This is safe only because Ed25519 SIGNING IS DETERMINISTIC (RFC 8032): the
//   per-signature nonce is derived by hashing the key with the message, not
//   drawn from an RNG. After key generation, this firmware needs no randomness
//   at all. That is a much smaller thing to get right than a live RNG.
//
// WHY NOT JUST esp_fill_random() ON ITS OWN:
//   Because if the SAR ADC source silently fails, esp_fill_random() keeps
//   returning bytes and they look fine. Mixing several independent sources
//   through SHA-256 means the output is as strong as the BEST surviving
//   source rather than as weak as the worst. The extractor costs milliseconds,
//   once, on the first boot of a device's life.
//
//   The eFuse MAC is included as a per-device DIFFERENTIATOR, not as entropy —
//   it is public and an attacker knows it. Its job is to guarantee that two
//   boards can never produce the same key even if every real source fails on
//   both. That converts a catastrophic fleet-wide collision into a merely
//   predictable single key, which the health test below then catches.
// ============================================================================

static void gwGatherSeed(uint8_t out[32]) {
  mbedtls_sha256_context ctx;
  mbedtls_sha256_init(&ctx);
  GW_SHA256_STARTS(&ctx, 0);                 // 0 = SHA-256, not SHA-224

  const char *domain = "greenwave-evu-identity-v1";
  GW_SHA256_UPDATE(&ctx, (const uint8_t *)domain, strlen(domain));

  // --- source 1: hardware RNG with the SAR ADC entropy source enabled ---
  bootloader_random_enable();
  for (int i = 0; i < 8; i++) {
    uint8_t chunk[32];
    esp_fill_random(chunk, sizeof(chunk));
    GW_SHA256_UPDATE(&ctx, chunk, sizeof(chunk));
    // Sample across time as well as across draws: the ADC noise source needs
    // wall-clock time to accumulate, and taking 256 bytes in a tight loop
    // samples far less physical noise than taking them over ~80 ms.
    delay(10);
    memset(chunk, 0, sizeof(chunk));
  }
  bootloader_random_disable();

  // --- source 2: timing jitter between two independent clock domains ---
  // esp_timer runs off a different source than the CPU cycle counter, so the
  // low bits of their relationship are genuinely noisy.
  for (int i = 0; i < 64; i++) {
    uint32_t j = (uint32_t)esp_timer_get_time() ^ (uint32_t)ESP.getCycleCount();
    GW_SHA256_UPDATE(&ctx, (const uint8_t *)&j, sizeof(j));
  }

  // --- source 3: per-device differentiator. NOT entropy. See the note above.
  uint8_t mac[6] = {0};
  esp_efuse_mac_get_default(mac);
  GW_SHA256_UPDATE(&ctx, mac, sizeof(mac));

  // --- source 4: hardware RNG again, after everything else has run ---
  bootloader_random_enable();
  uint8_t tail[32];
  esp_fill_random(tail, sizeof(tail));
  GW_SHA256_UPDATE(&ctx, tail, sizeof(tail));
  bootloader_random_disable();

  GW_SHA256_FINISH(&ctx, out);

  // Wipe every intermediate. Part 6 rule 6: never leave key material lying in
  // RAM where a later crash dump or a heap reuse can surface it.
  mbedtls_sha256_free(&ctx);
  memset(tail, 0, sizeof(tail));
}

// ----------------------------------------------------------------------------
// STARTUP HEALTH TEST  (NIST SP 800-90B, section 4.4, in miniature)
// ----------------------------------------------------------------------------
// Runs against the RAW generator, never against the derived key. A generator
// that is stuck, all-zero, or has collapsed to a short period will fail here.
//
// This CANNOT prove the RNG is good. Statistical testing of a few kilobytes
// never can. What it does is catch the failures that actually happen on
// embedded hardware: a dead entropy source returning constants, a peripheral
// that was never enabled, a clock that is not running.
//
// It is deliberately FATAL. A unit that will not boot is a support call. A
// unit that boots with a predictable key is a forgeable identity that nothing
// downstream — not the certificate, not the CA, not the RDU — can detect.
static bool gwRngHealthCheck() {
  const size_t N = 1024;
  uint8_t buf[N];

  bootloader_random_enable();
  esp_fill_random(buf, N);
  bootloader_random_disable();

  // (a) stuck-at test: all bytes identical
  bool allSame = true;
  for (size_t i = 1; i < N; i++) if (buf[i] != buf[0]) { allSame = false; break; }
  if (allSame) {
    Serial.println("[RNG] FAIL: generator returned a constant.");
    return false;
  }

  // (b) repetition count test: no value may repeat too many times in a row.
  // For a byte source the 800-90B cutoff at alpha=2^-30 is well above 8.
  size_t run = 1, maxRun = 1;
  for (size_t i = 1; i < N; i++) {
    run = (buf[i] == buf[i - 1]) ? run + 1 : 1;
    if (run > maxRun) maxRun = run;
  }
  if (maxRun > 8) {
    Serial.printf("[RNG] FAIL: repetition run of %u bytes.\n", (unsigned)maxRun);
    return false;
  }

  // (c) coverage: over 1024 draws a uniform byte source hits ~254 of 256
  // distinct values. A collapsed or low-period source hits far fewer.
  bool seen[256] = {false};
  uint16_t distinct = 0;
  for (size_t i = 0; i < N; i++) if (!seen[buf[i]]) { seen[buf[i]] = true; distinct++; }
  if (distinct < 200) {
    Serial.printf("[RNG] FAIL: only %u distinct byte values in %u draws.\n",
                  (unsigned)distinct, (unsigned)N);
    return false;
  }

  // (d) bit balance: a stuck bit line shows up here and nowhere else.
  uint32_t ones = 0;
  for (size_t i = 0; i < N; i++) ones += __builtin_popcount(buf[i]);
  const uint32_t total = N * 8;
  if (ones < total * 40 / 100 || ones > total * 60 / 100) {
    Serial.printf("[RNG] FAIL: bit balance %lu/%lu.\n",
                  (unsigned long)ones, (unsigned long)total);
    return false;
  }

  Serial.printf("[RNG] Health test PASSED (%u distinct values, max run %u, "
                "bit balance %lu/%lu).\n",
                (unsigned)distinct, (unsigned)maxRun,
                (unsigned long)ones, (unsigned long)total);
  memset(buf, 0, N);
  return true;
}

// ============================================================================
// SECURITY SUBSYSTEM PROVISIONING
// ============================================================================
void loadOrGenerateIdentity() {
  secPrefs.begin("evu-sec", false);
  provisioningLocked = secPrefs.getBool("provlock", false);

  size_t privLen = secPrefs.getBytesLength("privkey");
  if (privLen == 32) {
    secPrefs.getBytes("privkey", evuPrivateKey, 32);
    secPrefs.getBytes("pubkey",  evuPublicKey,  32);
    Serial.println("[SEC] Loaded existing Ed25519 identity from flash.");
    // Key generation happens ONCE in a device's life. It does not happen on
    // reboot and it does not happen on reflash — NVS survives both. If you see
    // this branch not taken on a unit that was already provisioned, the cause
    // is "Erase All Flash Before Sketch Upload" in the Arduino IDE. Turn it
    // off. Same root cause as the counter regression in Part 6 rule 4.
  } else {
    Serial.println("[SEC] No identity found — generating a new Ed25519 keypair.");
    Serial.println("[SEC] This happens ONCE per device, on first boot.");

    // Gate on the health test. A predictable private key is undetectable
    // downstream: the certificate will be issued over it, the CA will sign it,
    // every RDU will verify it, and an attacker who can derive it owns the
    // identity permanently. Nothing later in the chain can catch this, so it
    // has to be caught here.
    if (!gwRngHealthCheck()) {
      Serial.println("[SEC] ****************************************************");
      Serial.println("[SEC] *  RNG HEALTH TEST FAILED — REFUSING TO GENERATE    *");
      Serial.println("[SEC] *  A KEY. A predictable key defeats the CA, the     *");
      Serial.println("[SEC] *  certificate chain and every roadside check.      *");
      Serial.println("[SEC] *  This board must not be provisioned.              *");
      Serial.println("[SEC] ****************************************************");
#if IDENTITY_FATAL_HALT
      while (true) { delay(1000); }
#else
      return;
#endif
    }

    gwGatherSeed(evuPrivateKey);

    // Ed25519::generatePrivateKey() is deliberately NOT used. It draws from
    // rweather Crypto's own unseeded RNG object. An Ed25519 private key is 32
    // uniformly random bytes; clamping happens inside derivePublicKey().
    Ed25519::derivePublicKey(evuPublicKey, evuPrivateKey);

    // Paranoia that costs nothing: a derived public key of all zeros means the
    // whole path is broken in a way the health test did not model.
    bool zero = true;
    for (int i = 0; i < 32; i++) if (evuPublicKey[i]) { zero = false; break; }
    if (zero) {
      Serial.println("[SEC] *** DERIVED AN ALL-ZERO PUBLIC KEY — HALTING ***");
      while (true) { delay(1000); }
    }

    secPrefs.putBytes("privkey", evuPrivateKey, 32);
    secPrefs.putBytes("pubkey",  evuPublicKey,  32);
    Serial.println("[SEC] New identity generated and persisted.");
    Serial.println("[SEC] >> The private key is in PLAINTEXT NVS (Open item 2).");
    Serial.println("[SEC] >> Flash encryption WITHOUT secure boot does not fix");
    Serial.println("[SEC] >> this: an attacker flashes their own firmware and");
    Serial.println("[SEC] >> the hardware decrypts NVS for it. Both eFuse burns");
    Serial.println("[SEC] >> must be in the manufacturing flow from board one.");
  }

  char pubHex[65];
  bytesToHex(evuPublicKey, 32, pubHex, sizeof(pubHex));
  Serial.print("[SEC] EVU Public Key: ");
  Serial.println(pubHex);

  // ---- certificate ----
  size_t certLen = secPrefs.getBytesLength("cert");
  if (certLen == sizeof(GwLeafCert)) {
    secPrefs.getBytes("cert", evuCertificate, sizeof(GwLeafCert));
    const GwLeafCert *c = (const GwLeafCert *)evuCertificate;

    GwCertStatus st = gwCertMatchesOwnKey(c, evuPublicKey);
    if (st == GW_CERT_OK) st = gwCertCheckStructure(&c->tbs);

    if (st != GW_CERT_OK) {
      certProvisioned = false;
      Serial.printf("[SEC] *** STORED CERTIFICATE REJECTED: %s ***\n",
                    gwCertStatusName(st));
      Serial.println("[SEC] This unit must be re-provisioned.");
    } else {
      certProvisioned = true;
      Serial.printf("[SEC] Certificate OK: vehicle %u, issuer %u, serial %lu, "
                    "class %u\n",
                    (unsigned)c->tbs.vehicle_id, (unsigned)c->tbs.issuer_id,
                    (unsigned long)c->tbs.serial,
                    (unsigned)c->tbs.vehicle_class);
      Serial.printf("[SEC]   not_after: %s\n",
                    c->tbs.not_after_days == GW_DATE_NEVER
                      ? "NEVER" : "see ledger (days since 2020-01-01)");
    }
  } else {
    memset(evuCertificate, 0, sizeof(evuCertificate));
    certProvisioned = false;
    if (certLen > 0) {
      Serial.printf("[SEC] Stored certificate is %u B, expected %u — ignoring.\n",
                    (unsigned)certLen, (unsigned)sizeof(GwLeafCert));
    }
    Serial.println("[SEC] No certificate provisioned.");
    Serial.println("[SEC] Run, on the provisioning laptop:");
    Serial.printf("[SEC]   ./provision_evu.py --port <port> --vehicle-id %u --label %s\n",
                  (unsigned)VEHICLE_ID_NUM, VEHICLE_ID_LABEL);
  }

  Serial.printf("[SEC] Provisioning interface: %s\n",
                provisioningLocked ? "LOCKED (jumper required)" : "OPEN");

  // The EVU holds NO CA private key of any tier, and never will. A device that
  // could issue certificates would be a total-compromise device: it is
  // operator-accessible and its flash is unencrypted, so one stolen unit would
  // mint identities for the whole fleet. It also holds no root or issuer
  // PUBLIC key, so it cannot validate the CA signature on its own certificate
  // — it only checks that the certificate belongs to its own key, which is
  // what catches the realistic provisioning error. Full chain validation is
  // the RDU's job, against GreenwaveTrustAnchors.h.
}
void handleSerialProvisioning() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c != '\n' && c != '\r') {
      if (serialLineLen < SERIAL_LINE_MAX - 1) {
        serialLine[serialLineLen++] = c;
      } else {
        // Do NOT truncate silently. A truncated line produces an error message
        // about a string the operator never typed, which is worse than no
        // message at all — it sends them looking for a fault in their input.
        serialLineOverflow = true;
      }
      continue;
    }

    if (serialLineOverflow) {
      Serial.printf("[SEC] Input line too long (over %u chars) — IGNORED.\n",
                    (unsigned)(SERIAL_LINE_MAX - 1));
      Serial.println("[SEC] Nothing was stored. Re-send the command.");
      serialLineOverflow = false;
      serialLineLen = 0;
      continue;
    }

    serialLine[serialLineLen] = 0;
    // trim trailing whitespace
    while (serialLineLen > 0 && isspace((int)serialLine[serialLineLen - 1]))
      serialLine[--serialLineLen] = 0;

    if (serialLineLen == 0) { serialLineLen = 0; continue; }

    // ---- SCENARIO 2 / S2-08 : emergency flag control ----
    // Turning it OFF produces the falling edge the ICU treats as an explicit
    // release — the only unambiguous "I am done" in the whole system.
    if (!strcasecmp(serialLine, "EMERG ON")) {
      if (!sirenActive) {
        sirenActive = true;
        certBurstRemaining = CERT_BURST_COUNT;   // V5.2 rising-edge cert burst
        Serial.println("[EMERG] ON -- emergency flag set, priority will be requested");
      } else {
        Serial.println("[EMERG] already ON");
      }
    }
    else if (!strcasecmp(serialLine, "EMERG OFF")) {
      if (sirenActive) {
        sirenActive = false;
        Serial.println("[EMERG] OFF -- flag cleared. The NEXT packet carries the "
                       "falling edge and the ICU should release this vehicle's demand.");
      } else {
        Serial.println("[EMERG] already OFF");
      }
    }
    else if (!strcasecmp(serialLine, "EMERG")) {
      Serial.printf("[EMERG] %s\n", sirenActive ? "ON" : "OFF");
    }
    // ---- V7 PROVISIONING INTERFACE ----
    // These four commands are what provision_evu.py drives. The exchange is
    // one-directional in the only way that matters: the device EMITS a public
    // key and ACCEPTS a certificate. No private key crosses the port in either
    // direction, ever, and no CA key of any tier exists on this device.
    else if (!strcasecmp(serialLine, "PUBKEY")) {
      char h[65];
      bytesToHex(evuPublicKey, 32, h, sizeof(h));
      Serial.printf("[SEC] EVU Public Key: %s\n", h);
    }
    else if (!strcasecmp(serialLine, "GETCERT")) {
      if (!certProvisioned) {
        Serial.println("[SEC] GETCERT: no certificate provisioned.");
      } else {
        char h[sizeof(GwLeafCert) * 2 + 1];
        bytesToHex(evuCertificate, sizeof(GwLeafCert), h, sizeof(h));
        Serial.printf("[SEC] CERT %s\n", h);
      }
    }
    else if (!strncasecmp(serialLine, "SETCERT ", 8)) {
      const char *hex = serialLine + 8;
      while (*hex == ' ') hex++;

      bool unlocked = !provisioningLocked;
#if PROVISION_JUMPER_PIN >= 0
      if (!unlocked) unlocked = (digitalRead(PROVISION_JUMPER_PIN) == LOW);
#endif
      if (!unlocked) {
        Serial.println("[SEC] SETCERT rejected: provisioning is LOCKED.");
        Serial.println("[SEC] Fit the provisioning jumper and power-cycle.");
        serialLineLen = 0;
        continue;
      }

      uint8_t candidate[sizeof(GwLeafCert)];
      if (!hexToBytes(hex, strlen(hex), candidate, sizeof(candidate))) {
        // Report what was actually RECEIVED, not just what was expected. The
        // difference between the two is the entire diagnostic.
        Serial.printf("[SEC] SETCERT rejected: expected %u hex characters, "
                      "received %u.\n",
                      (unsigned)(sizeof(GwLeafCert) * 2), (unsigned)strlen(hex));
        serialLineLen = 0;
        continue;
      }

      // Validate BEFORE storing. The v0 command accepted any well-formed 64
      // bytes and persisted them unchecked, so the first sign of a bad
      // provisioning step was a roadside unit rejecting every frame with
      // nothing in any log naming the cause.
      //
      // The EVU cannot verify the CA signature — it holds no trust anchor, by
      // design. It CAN check the certificate is for its own key, and that
      // catches the provisioning error that actually happens in practice:
      // pasting unit A's certificate into unit B.
      const GwLeafCert *c = (const GwLeafCert *)candidate;
      GwCertStatus st = gwCertMatchesOwnKey(c, evuPublicKey);
      if (st == GW_CERT_OK) st = gwCertCheckStructure(&c->tbs);

      if (st != GW_CERT_OK) {
        Serial.printf("[SEC] SETCERT rejected: %s\n", gwCertStatusName(st));
        if (st == GW_CERT_KEY_MISMATCH) {
          Serial.println("[SEC] That certificate belongs to a DIFFERENT unit.");
          Serial.println("[SEC] Check you are on the right serial port.");
        }
        serialLineLen = 0;
        continue;
      }

      memcpy(evuCertificate, candidate, sizeof(GwLeafCert));
      secPrefs.putBytes("cert", evuCertificate, sizeof(GwLeafCert));
      certProvisioned = true;
      certBurstRemaining = CERT_BURST_COUNT;
      Serial.printf("[SEC] Certificate stored: vehicle %u, issuer %u, "
                    "serial %lu, class %u\n",
                    (unsigned)c->tbs.vehicle_id, (unsigned)c->tbs.issuer_id,
                    (unsigned long)c->tbs.serial,
                    (unsigned)c->tbs.vehicle_class);
    }
    else if (!strcasecmp(serialLine, "LOCKPROV")) {
      provisioningLocked = true;
      secPrefs.putBool("provlock", true);
      Serial.println("[SEC] Provisioning interface LOCKED.");
      Serial.println("[SEC] SETCERT now requires the provisioning jumper.");
    }
#if RANGE_TEST_BUILD
    // RNGTEST dumps RAW generator output for offline analysis with dieharder
    // or the NIST STS.
    //
    // The three-boards-different-keys check catches IDENTICAL keys but says
    // nothing about LOW-ENTROPY ones. This is how you actually measure the
    // source: dump a few megabytes and run it through a test suite.
    //
    // GATED BEHIND RANGE_TEST_BUILD ON PURPOSE. In a production build this is
    // a free oracle on the same generator that produced the identity key. It
    // must not ship. Set RANGE_TEST_BUILD to 0 for anything leaving the bench.
    else if (!strncasecmp(serialLine, "RNGTEST", 7)) {
      long kb = (serialLine[7] == ' ') ? atol(serialLine + 8) : 64;
      if (kb < 1)    kb = 1;
      if (kb > 1024) kb = 1024;
      Serial.printf("[RNG] Dumping %ld KB of raw generator output as hex.\n", kb);
      Serial.println("[RNG] BEGIN");
      bootloader_random_enable();
      for (long i = 0; i < kb * 32; i++) {
        uint8_t b[32];
        char    h[65];
        esp_fill_random(b, sizeof(b));
        bytesToHex(b, sizeof(b), h, sizeof(h));
        Serial.println(h);
      }
      bootloader_random_disable();
      Serial.println("[RNG] END");
    }
#endif
    else if (!strncasecmp(serialLine, "LEG ", 4)) {
      int val = atoi(serialLine + 4);
      if (val >= 0 && val <= 255) {
        currentTestLeg = (uint8_t)val;
        Serial.printf("[TEST] Leg tag set to %u — every packet from now on carries this tag.\n",
                      currentTestLeg);
      } else {
        Serial.println("[TEST] LEG rejected: expected an integer 0-255, e.g. LEG 50");
      }
    }
    else if (!strcasecmp(serialLine, "GNSS")) {
      // Quick health check without waiting for a TX cycle.
      Serial.printf("[GNSS] fixType=%u SIV=%u hAcc=%lu mm age=%lu ms rate=%u Hz\n",
                    (unsigned)myGNSS.getFixType(),
                    (unsigned)myGNSS.getSIV(),
                    (unsigned long)myGNSS.getHorizontalAccEst(),
                    (unsigned long)(millis() - lastPvtMillis),
                    (unsigned)myGNSS.getNavigationFrequency());
    }
    else {
      Serial.println("[SEC] Unknown command. Use: PUBKEY | GETCERT | "
                     "SETCERT <216-hex> | LOCKPROV | LEG <0-255> | "
                     "EMERG ON|OFF | GNSS");
    }

    serialLineLen = 0;
  }
}

void loadPersistedCounter() {
  uint32_t stored = secPrefs.getUInt("seqctr", 0);
  sequenceNumber = stored + SEQ_SAVE_MARGIN;
  secPrefs.putUInt("seqctr", sequenceNumber);
  Serial.printf("[SEC] Restored counter (with anti-rollback margin) = %lu\n",
                (unsigned long)sequenceNumber);
}

void maybePersistCounter() {
  seqSinceLastSave++;
  if (seqSinceLastSave >= SEQ_SAVE_INTERVAL) {
    secPrefs.putUInt("seqctr", sequenceNumber);
    seqSinceLastSave = 0;
  }
}

// ============================================================================
// DUTY CYCLE AUDIT
//
// ~37%, far outside any licence-exempt duty-cycle allowance the author is
// aware of — and not fixable in firmware, because the 64-byte Ed25519
// signature dominates the frame and SF9 is required by measurement (SF7 gave
// 39% packet loss at TC-TX-002).
//
// This is a BAND / REGULATORY decision, not a code change: either the
// applicable Indian allowance for 433.05-434.79 MHz turns out to be generous,
// or this system moves to 865-867 MHz. It must be settled before PCB layout
// and antenna design, because it changes both.
// ============================================================================
void printDutyCycle() {
  // V7: computed from sizeof() and the live LORA_* settings, never from a
  // hand-maintained constant. Change the struct, the certificate or a radio
  // parameter and this number follows automatically — the only way an audit
  // like this stays honest.
  const uint32_t steadyBytes = sizeof(TelemetryPayload) + 64;
  const uint32_t certBytes   = steadyBytes + sizeof(GwLeafCert);

  const double toaSteady = gwLoRaAirtimeMs(steadyBytes, LORA_SF,
                                           (uint32_t)LORA_BW_HZ,
                                           LORA_CR_DENOM, LORA_PREAMBLE);
  const double toaCert   = gwLoRaAirtimeMs(certBytes, LORA_SF,
                                           (uint32_t)LORA_BW_HZ,
                                           LORA_CR_DENOM, LORA_PREAMBLE);

  // Steady-state figure. The activation burst is excluded deliberately: it is
  // a few packets once per emergency run, not a sustained rate, and quoting a
  // burst-inflated average would misrepresent the ongoing load.
  const double n      = (double)CERT_INTERVAL_STEADY;
  const double avgToa = ((n - 1.0) * toaSteady + toaCert) / n;
  const double duty   = 100.0 * avgToa / (double)TX_INTERVAL_MS;

  Serial.println("---- IF-1 AIRTIME BUDGET ----");
  Serial.printf("  payload        %u B  (wire v%u)\n",
                (unsigned)sizeof(TelemetryPayload), GW_WIRE_VERSION);
  Serial.printf("  certificate    %u B  (cert format v%u)\n",
                (unsigned)sizeof(GwLeafCert), GW_CERT_VERSION);
  Serial.printf("  steady frame   %lu B  %.1f ms\n",
                (unsigned long)steadyBytes, toaSteady);
  Serial.printf("  cert frame     %lu B  %.1f ms  (every %d, steady state)\n",
                (unsigned long)certBytes, toaCert, CERT_INTERVAL_STEADY);
  Serial.printf("  cert burst     %d packets on emergency activation\n",
                CERT_BURST_COUNT);
  Serial.printf("  interval       %lu ms\n", TX_INTERVAL_MS);
  Serial.printf("  DUTY CYCLE     %.2f %%  (steady state)\n", duty);

  // LoRa airtime is quantised into 6-symbol steps, ~4.5 bytes wide at
  // SF9/CR4-6. Printing the headroom turns "adding a frame field costs
  // airtime" from a warning into a number: while this reads > 0 the next field
  // is free; when it reads 0 the next byte costs a full 24.576 ms step.
  Serial.printf("  bucket headroom %lu B free before airtime steps up\n",
                (unsigned long)gwFreeBytesInAirtimeBucket(steadyBytes, LORA_SF,
                                                          LORA_CR_DENOM,
                                                          LORA_PREAMBLE));

  if (duty > 10.0) {
    Serial.println("  ** OVER 10% - not compliant with any licence-exempt");
    Serial.println("  ** allowance the team has verified. See OPEN-003/OPEN-010.");
    Serial.println("  ** Compaction took this from 32.9% to 29.3%. That is real,");
    Serial.println("  ** and it is NOT the fix. Struct size cannot close a");
    Serial.println("  ** 29-point gap. The fix is TRANSMISSION GATING");
    Serial.println("  ** (emergency-only + GNSS corridor), which moves the");
    Serial.println("  ** HOURLY AVERAGE that regulators assess, or a band change.");
    Serial.println("  ** Both are decisions, not code.");
  }
  Serial.println("-----------------------------");
}

// ============================================================================
// GNSS CONFIGURATION  (FIX 3)
// ============================================================================
// V4 wired GPS_TX and never used it. Not one byte was ever written to the
// module. It therefore ran in whatever state its battery-backed RAM happened
// to hold — which differs between units, differs after a power cut, and
// differs after anyone connects u-center. Two "identical" prototypes behaved
// differently and nothing in the logs explained why.
//
// This function pushes a known configuration at EVERY boot. It does not rely
// on saved state and does not need to.
//
// What each setting buys:
//   UBX-only        NMEA at 9600 with default multi-constellation output can
//                   occupy most of a second per epoch. 10 Hz is physically
//                   impossible at that baud. UBX-NAV-PVT is one ~100-byte
//                   binary message carrying position, velocity, heading, fix
//                   type, SIV, time AND hAcc.
//   115200 baud     Required for 10 Hz. The datasheet is explicit: for high
//                   navigation update rates, increase the baud rate and
//                   reduce the number of enabled messages.
//   Automotive      Default dynamic model is Portable. Automotive changes the
//                   internal Kalman filter's motion assumptions for a road
//                   vehicle. Free accuracy, one line.
//   10 Hz           Does NOT increase transmit rate. It means the fix we send
//                   every 2000 ms is ~100 ms old instead of up to 2000 ms old.
// ============================================================================
bool configureGNSS() {
  GPSSerial.setRxBufferSize(GPS_RX_BUFFER_BYTES);   // FIX 9a — must precede begin()

  // The module may be at either baud depending on prior state. Try target
  // first, fall back to default and promote it.
  GPSSerial.begin(GNSS_TARGET_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(100);

  if (!myGNSS.begin(GPSSerial)) {
    Serial.println("[GNSS] Not at 115200 — trying 9600 and promoting.");
    GPSSerial.begin(GNSS_DEFAULT_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(100);
    if (!myGNSS.begin(GPSSerial)) {
      Serial.println("[GNSS] *** MODULE NOT RESPONDING at 9600 or 115200 ***");
      return false;
    }
    myGNSS.setSerialRate(GNSS_TARGET_BAUD);
    delay(150);
    GPSSerial.begin(GNSS_TARGET_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
    delay(150);
    if (!myGNSS.begin(GPSSerial)) {
      Serial.println("[GNSS] *** baud promotion failed ***");
      return false;
    }
    Serial.println("[GNSS] Promoted to 115200.");
  }

  myGNSS.setUART1Output(COM_TYPE_UBX);        // UBX only, NMEA off
  myGNSS.setNavigationFrequency(GNSS_NAV_RATE_HZ);
  myGNSS.setAutoPVT(true);                    // module pushes PVT, no polling stall
  myGNSS.setDynamicModel(DYN_MODEL_AUTOMOTIVE);

  // NOTHING IS PERSISTED TO THE MODULE, DELIBERATELY.
  // v2's saveConfigurationVDataOnly() does not exist in v3 (VALSET interface),
  // and it should not be substituted with saveConfiguration() — that writes
  // all layers including module flash, which is the opposite of what was
  // wanted. More to the point, persistence is not needed: this function runs
  // on EVERY boot and never reads saved state. That is the whole point of
  // FIX 3. A unit that reconfigures from scratch every time is deterministic;
  // two identical boards behave identically. Persisting config would make the
  // V4 failure mode (module running on whatever its backup RAM held) harder
  // to reason about, not easier.

  Serial.printf("[GNSS] Configured: UBX-only, Automotive, %u baud\n",
                GNSS_TARGET_BAUD);

  // READ BACK the nav rate rather than printing what we asked for.
  // The serial log alone cannot distinguish 1 Hz from 10 Hz: TX_INTERVAL_MS is
  // an exact multiple of a 100 ms nav period, so fix epochs land on the same
  // phase either way. Only the module's own answer settles it.
  uint8_t rateHz = myGNSS.getNavigationFrequency();
  Serial.printf("[GNSS] Nav rate readback: %u Hz (requested %u)%s\n",
                (unsigned)rateHz, GNSS_NAV_RATE_HZ,
                (rateHz == GNSS_NAV_RATE_HZ) ? "" : "   <-- MISMATCH");
  return true;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
#if EMERGENCY_SWITCH_PIN >= 0
  pinMode(EMERGENCY_SWITCH_PIN,
          EMERGENCY_SWITCH_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
#endif
#if PROVISION_JUMPER_PIN >= 0
  pinMode(PROVISION_JUMPER_PIN, INPUT_PULLUP);   // LOW = jumper fitted
#endif

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("GREENWAVE EVP TRANSMITTER V7");
#if RANGE_TEST_BUILD
  Serial.println("*** RANGE-TEST BUILD - leg tagging (LOG ONLY), verbose dump AND\n"
                 "*** RNGTEST raw-entropy dump ACTIVE. DO NOT SHIP. ***");
#else
  Serial.println("Production build - range-test instrumentation disabled");
#endif
  Serial.println("ESP32 + SX1278 + MAX-M10S");
  Serial.println("========================================");

  // Layout drift canary. The signature covers exactly sizeof(TelemetryPayload)
  // bytes and this struct is hand-duplicated across three trees. Printing the
  // size means a drift produces a VISIBLE symptom instead of universal,
  // unexplained signature failure. Compare against RDU and ICU boot logs.
  // Layout drift canary. The static_asserts in the shared headers catch a
  // local edit at compile time, but the three trees still hold three COPIES of
  // those files and nothing checks the copies agree. Printing sizes and
  // versions means drift produces a VISIBLE symptom instead of universal,
  // unexplained signature failure. Compare against the RDU and ICU boot logs.
  Serial.printf("[PROTO] wire v%u payload %u B | cert v%u %u B — must match "
                "GreenwaveWireV6.h and GreenwaveCertV1.h in the RDU/ICU trees\n",
                GW_WIRE_VERSION, (unsigned)sizeof(TelemetryPayload),
                GW_CERT_VERSION, (unsigned)sizeof(GwLeafCert));

  if (!configureGNSS()) {
    Serial.println("[GNSS] Continuing without GNSS — all packets will carry "
                   "gps_valid CLEAR.");
  }

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("[ERROR] LoRa Init Failed");
    delay(1000);
  }

  LoRa.setTxPower(LORA_TX_POWER_DBM);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  // SF9 by measurement (SF7 gave 39% loss at TC-TX-002). SF8 was NEVER
  // TESTED and is the single largest airtime saving available (~283 ms).
  // CR 4/6 adds FEC without SF10/CR8's ~2 s airtime. Every one of these must
  // match RDU1.ino exactly — SF/BW/CR/sync-word/preamble mismatches don't
  // degrade gracefully, they just fail to link.
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW_HZ);
  LoRa.setCodingRate4(LORA_CR_DENOM);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.enableCrc();
  printDutyCycle();
  Serial.println("[LORA] TX READY");
  Serial.printf("[LORA CONFIG] SF=%d BW=%dkHz CR=4/%d Preamble=%d "
                "TXPower=%ddBm SyncWord=0x%02X\n",
                LORA_SF, (int)(LORA_BW_HZ / 1000), LORA_CR_DENOM,
                LORA_PREAMBLE, LORA_TX_POWER_DBM, LORA_SYNC_WORD);
#if LORA_SF != 9 || LORA_CR_DENOM != 6 || LORA_PREAMBLE != 12
  Serial.println("[LORA] ** RADIO PARAMS CHANGED FROM VALIDATED BASELINE **");
  Serial.println("[LORA] ** Duty-cycle figure above is WRONG. Recompute. **");
  Serial.println("[LORA] ** RDU must be reflashed to match. **");
#endif

  loadOrGenerateIdentity();
  loadPersistedCounter();
  previousTX = millis();
  Serial.println("Broadcast Started");
#if CSV_LOG_MODE
  Serial.println();
  Serial.println("--- CSV MODE. Select all below, save as .csv, run "
                 "calibrate_from_log.py ---");
  // lat/lon/speed/heading are now DECODED values. hacc_mm and fix_age_ms
  // remain TRUE pre-encode values. calibrate_from_log.py must be re-read.
  Serial.println("uptime_s,seq,leg,lat,lon,speed_kmph,heading_deg,heading_valid,"
                 "fix_type,sats,hacc_mm,fix_age_ms,precision_high,gps_valid,"
                 "emergency,packet_bytes,cert");
#endif
}

// ============================================================================
// EMERGENCY SWITCH
// ============================================================================
// Debounced, because a bouncing contact would otherwise generate a burst of
// rising and falling edges — and the ICU treats a falling edge as an explicit
// release. A single bounce could release a live preemption.
static void pollEmergencySwitch() {
#if EMERGENCY_SWITCH_PIN >= 0
  static int           lastRaw      = -1;
  static unsigned long lastChangeMs = 0;
  static bool          stable       = true;

  int  raw  = digitalRead(EMERGENCY_SWITCH_PIN);
  bool want = EMERGENCY_SWITCH_ACTIVE_LOW ? (raw == LOW) : (raw == HIGH);

  if (raw != lastRaw) {
    lastRaw      = raw;
    lastChangeMs = millis();
    stable       = false;
    return;
  }

  if (!stable && (millis() - lastChangeMs) > 80) {
    stable = true;
    if (want != sirenActive) {
      sirenActive = want;
      if (want) certBurstRemaining = CERT_BURST_COUNT;  // V5.2 rising edge only
      Serial.printf("[EMERG] switch -> %s\n", want ? "ON" : "OFF");
    }
  }
#endif
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  pollEmergencySwitch();
  handleSerialProvisioning();

  // Non-blocking: drains the UART and parses any complete UBX frame.
  // setAutoPVT means the module pushes PVT at 10 Hz without us polling, so
  // this never stalls waiting for a response.
  if (myGNSS.getPVT(0)) {           // 0 = do not wait
    lastPvtMillis = millis();
    pvtFresh      = true;
  }

  unsigned long now = millis();
  if (now - previousTX < TX_INTERVAL_MS) return;

  // previousTX = now, not previousTX += TX_INTERVAL_MS. Accumulate-style
  // scheduling is correct only if the loop never falls behind. If anything
  // stalls it for longer than one interval, previousTX ends up in the past
  // and the next iterations fire back-to-back trying to catch up. Bursting
  // is the worst possible behaviour on a duty-cycle-limited link, and it
  // collides with the lane node's gap-scheduling logic, which assumes a
  // predictable 2000 ms cadence.
  previousTX = now;

  // --------------------------------------------------------------------
  // GNSS ACCEPTANCE  (FIX 1, 2, 4, 5)
  //
  // What used to be here: an EMA filter, a static-lock state machine, a
  // stationary counter, a movement counter, and a locked position that was
  // transmitted INSTEAD of the live one with speed forced to zero.
  //
  // All of it is gone. What follows is: take the fix, check whether it is
  // good enough, send it. No smoothing, no freezing, no state.
  //
  // Why the static lock had to go rather than be tuned:
  //   - It transmitted a frozen historical position while claiming speed 0
  //   - Unlock needed 3 confirmations at 2000 ms = 6 seconds minimum
  //   - So an ambulance pulling away from a red light reported "parked" for
  //     6 seconds — up to 67 m at 40 km/h, ~100 m at 60 km/h
  //   - The speed=0 was arguably worse: the ICU needs speed for ETA, and it
  //     was zeroed during the exact acceleration phase that matters
  //   - A vehicle creeping at 2 km/h never exceeded MOVE_SPEED_THRESHOLD, so
  //     it stayed locked through congestion
  //   - It is a base-station/survey feature applied to a moving vehicle
  //
  // If stationary-vehicle smoothing is wanted, it belongs in the ICU, which
  // is field-updatable. A fleet of ambulances is not.
  // --------------------------------------------------------------------
  bool     gpsValid      = false;
  bool     headingValid  = false;
  bool     altitudeValid = false;
  bool     precisionHigh = false;
  double   latitude = 0, longitude = 0, speedKmph = 0, headingDeg = 0;
  double   altitudeM = 0;
  uint8_t  fixType = 0;
  uint8_t  siv     = 0;
  uint32_t hAccMm  = 0xFFFFFFFF;
  uint32_t fixAgeMs = 0xFFFFFFFF;
  uint32_t gpsEpoch = 0;      // diagnostics only in V6 — not a wire field
  uint16_t gpsMs    = 0;      // diagnostics only in V6 — not a wire field
  uint32_t gpsTowMs = 0;      // V6 wire field, 100 ms resolution once encoded

  if (pvtFresh) {
    fixAgeMs = now - lastPvtMillis;
    fixType  = myGNSS.getFixType();
    siv      = myGNSS.getSIV();
    hAccMm   = myGNSS.getHorizontalAccEst();          // millimetres

    // FIX 4 + FIX 5: gate on the receiver's own uncertainty estimate and on
    // real freshness. HDOP is gone — it describes satellite geometry only and
    // is blind to multipath, which is the dominant error at an intersection.
    if (fixType >= MIN_FIX_TYPE &&
        siv     >= MIN_SATELLITES &&
        hAccMm  <= MAX_HACC_MM &&
        fixAgeMs <= MAX_FIX_AGE_MS) {

      gpsValid  = true;
      latitude  = myGNSS.getLatitude()  / 10000000.0;
      longitude = myGNSS.getLongitude() / 10000000.0;

      lastGoodLat    = latitude;
      lastGoodLon    = longitude;
      haveEverHadFix = true;

      // mm/s -> km/h
      speedKmph = myGNSS.getGroundSpeed() * 0.0036;

      altitudeM     = myGNSS.getAltitudeMSL() / 1000.0;   // mm -> m
      altitudeValid = true;

      precisionHigh = (hAccMm <= PRECISION_HACC_MM);

      // FIX 7: heading validity.
      //
      // V4 wrote `(heading < 0 ? 0 : heading)`, so an invalid course went out
      // as 0.00 degrees — due North — indistinguishable from a real northward
      // heading. GNSS course-over-ground comes from the velocity vector and is
      // essentially random below walking pace. The consequence was the ICU
      // assigning a vehicle to the wrong approach leg and preempting the
      // wrong phase, confidently.
      //
      // Now it is a flag. Heading is only marked valid when the vehicle is
      // actually moving fast enough for it to mean anything AND the receiver's
      // own heading accuracy estimate agrees.
      double headAccDeg = myGNSS.getHeadingAccEst() / 100000.0;
      headingDeg        = myGNSS.getHeading()       / 100000.0;   // deg*1e-5 -> deg

      if (speedKmph >= HEADING_VALID_MIN_KMPH && headAccDeg <= HEADING_MAX_ACC_DEG) {
        headingValid = true;
      } else {
        headingDeg = 0;     // still zero on the wire, but bit 4 is now CLEAR
      }

      // FIX 6: gps_epoch is the epoch of THE FIX.
      //
      // V4 built this from centisecond + gps.time.age(), which approximated
      // the moment of PACKET ASSEMBLY. Meanwhile the position itself could be
      // up to 2000 ms older, with nothing on the wire saying so. Any receiver
      // dead-reckoning from that timestamp extrapolated from the wrong origin
      // and was confident about it.
      //
      // NAV-PVT carries the navigation solution's own UTC. These fields
      // describe the same instant as the latitude and longitude beside them.
      //
      // V6: the wire carries GPS TIME-OF-WEEK at 100 ms resolution (3 B)
      // instead of an absolute epoch (4 B) plus a millisecond field (2 B).
      //
      // Two reasons. gps_ms was 16 bits holding TEN distinct values, because
      // setNavigationFrequency(10) makes getMillisecond() a multiple of 100.
      // And the RDU has no wall clock, so it cannot use an absolute epoch at
      // all; the ICU, which does have a clock, reconstructs the absolute time
      // from its own week number plus this TOW.
      //
      // gpsEpoch is still computed, for the IST line in the serial diagnostics.
      // It is a LOCAL variable now, not a wire field.
      if (myGNSS.getTimeValid() && myGNSS.getDateValid()) {
        long days = daysFromCivil(myGNSS.getYear(), myGNSS.getMonth(),
                                  myGNSS.getDay());

        gpsEpoch = (uint32_t)days * 86400UL
                 + (uint32_t)myGNSS.getHour()   * 3600UL
                 + (uint32_t)myGNSS.getMinute() * 60UL
                 + (uint32_t)myGNSS.getSecond();
        gpsMs    = myGNSS.getMillisecond();

        // 1970-01-01 was a Thursday, so dayOfWeek = (days + 4) mod 7 with
        // 0 = Sunday, matching the GPS week boundary.
        uint8_t dow = (uint8_t)(((days % 7) + 7 + 4) % 7);

        gpsTowMs = gw6TowMsFromUtc(dow, myGNSS.getHour(), myGNSS.getMinute(),
                                   myGNSS.getSecond(), gpsMs);
      }
    }
  }

  // Position when gps_valid is CLEAR: last known good, not 0,0.
  // 0,0 is a real coordinate in the Gulf of Guinea and a consumer that forgets
  // to check the flag would silently believe it. Last-known at least fails in
  // a plausible place. It is still NOT a valid position.
  if (!gpsValid && haveEverHadFix) {
    latitude  = lastGoodLat;
    longitude = lastGoodLon;
  }

  // --------------------------------------------------------------------
  // ASSEMBLE PACKET
  // --------------------------------------------------------------------
  TelemetryPayload packet;
  memset(&packet, 0, sizeof(packet));

  packet.proto_version = GW_WIRE_VERSION;

  packet.flags = 0;
  if (sirenActive)   packet.flags |= FLAG_EMERGENCY;
  if (gpsValid)      packet.flags |= FLAG_GPS_VALID;
  if (altitudeValid) packet.flags |= FLAG_ALT_VALID;
  if (headingValid)  packet.flags |= FLAG_HEADING_VALID;
  if (precisionHigh) packet.flags |= FLAG_PRECISION_HIGH;

  // FLAG_SIREN deleted: V5 set it and FLAG_EMERGENCY from the same sirenActive
  // variable — two bits carrying one fact. Reclaiming bit 2 is what lets
  // priority_class live in flags instead of its own byte.
  //
  // The class here is the one the CA authorised, read straight out of the
  // stored certificate. That is a change of meaning from V5, where the EVU
  // asserted a compile-time constant about itself and the ICU could only treat
  // it as advisory (spec defect S2-01). The AUTHORITATIVE copy is the CA-signed
  // vehicle_class inside the certificate; this flags copy is a convenience for
  // frames arriving before the cert has been cached, and a disagreement between
  // the two remains a useful security signal.
  {
    uint8_t cls = 0;
    if (certProvisioned) {
      cls = ((const GwLeafCert *)evuCertificate)->tbs.vehicle_class;
    }
    gw6SetPriority(&packet.flags, cls);
  }

  packet.seq = sequenceNumber;

  // Fix time as GPS time-of-week. gpsTowMs already timestamps THE FIX (FIX 6),
  // not packet assembly.
  gw6EncodeTowMs(packet.tow_ds, gpsTowMs);

  // Every codec rounds rather than truncates. Truncation biases toward the
  // equator in the southern and western hemispheres, and a deterministic error
  // in a fixed direction is exactly what FIX 2 deleted the EMA filter for.
  gw6EncodeCoord(packet.lat, latitude);
  gw6EncodeCoord(packet.lon, longitude);

  packet.altitude_m = (int16_t)lround(altitudeValid ? altitudeM : 0);
  packet.speed      = gw6EncodeSpeed(speedKmph);
  packet.heading    = gw6EncodeHeading(headingValid ? headingDeg : 0.0);

  // NEW in V6. This is what makes the FIX 8 dead-reckoning contract
  // implementable at an RDU with no wall clock: the receiver reads the fix age
  // straight off the wire instead of needing t_now - gps_epoch on a shared
  // clock it does not have.
  packet.quality = gw6EncodeQuality(gpsValid ? hAccMm   : 0xFFFFFFFFUL,
                                    gpsValid ? fixAgeMs : 0xFFFFFFFFUL);

  // test_leg is GONE from the wire. The leg tag survives as a CSV column only:
  // it is range-test instrumentation and has no business in a pilot frame,
  // independent of the fact that removing it saves 0 ms at this frame size.

  // --------------------------------------------------------------------
  // SIGN AND TRANSMIT
  // --------------------------------------------------------------------
  uint8_t signature[64];
  Ed25519::sign(signature, evuPrivateKey, evuPublicKey, &packet, sizeof(packet));
  // V5.2 CERTIFICATE SCHEDULE — burst on activation, then back off.
  //
  // Burst: for the first CERT_BURST_COUNT packets after the emergency flag
  // rises, send the cert every packet. This is the only moment a cold RDU is
  // likely to be meeting this vehicle for the first time.
  //
  // Steady: every CERT_INTERVAL_STEADY packets thereafter. By then every RDU
  // in range has cached the key, so this is only a safety net for an RDU that
  // boots mid-run. Packets arriving before its cert lands are held in the
  // RDU's retro-verification buffer (spec §2.3), not dropped.
  //
  // Verification coverage is 100% either way — the per-packet signature never
  // stopped being sent.
  // An unprovisioned unit must not transmit a certificate block: it would be
  // 108 bytes of zeros, costing 590 ms of airtime to say nothing. The steady
  // frame is still signed and still verifiable by any RDU that has previously
  // cached this key.
  bool includeCertThisPacket;
  if (!certProvisioned) {
    includeCertThisPacket = false;
  } else if (certBurstRemaining > 0) {
    includeCertThisPacket = true;
    certBurstRemaining--;
  } else {
    includeCertThisPacket = (sequenceNumber % CERT_INTERVAL_STEADY == 0);
  }

  LoRa.beginPacket();
  LoRa.write((uint8_t *)&packet, sizeof(packet));
  LoRa.write(signature, sizeof(signature));

  if (includeCertThisPacket) {
    // The whole 108-byte certificate, verbatim as issued. vehicle_id travels
    // HERE and nowhere else — the issuing CA's signature over the TBS binds it
    // to this device's public key, so it cannot be substituted.
    LoRa.write(evuCertificate, sizeof(GwLeafCert));
  }

  // *** RESTORED IN V7 ***
  // This call was commented out in the V5 file, so beginPacket() and write()
  // filled the SX1278 FIFO and NOTHING WAS EVER PUT ON THE AIR. The frame was
  // still assembled, still signed, and still logged as "LoRa TX : SUCCESS", so
  // the CSV gave no hint the radio was silent. Check any V5 field data
  // against this before trusting it.
  //
  // endPacket() BLOCKS for the whole time on air. That is why
  // GPS_RX_BUFFER_BYTES is sized against the certificate frame: at 10 Hz and
  // ~100 B per UBX-NAV-PVT, nothing drains the GNSS UART for the duration.
  if (!LoRa.endPacket()) {
    Serial.println("[LORA] ** endPacket() FAILED — frame not transmitted **");
  }

  // NOTE: no LoRa.receive() here, and that is deliberate. The radio returns
  // to standby after endPacket() and this unit hears nothing. Opening a
  // receive window would be one line — and would create an unauthenticated
  // inbound path into an ambulance cab. See the architecture note in the
  // header.

  sequenceNumber++;
  maybePersistCounter();

  // --------------------------------------------------------------------
  // DIAGNOSTICS  (FIX 9a)
  //
  // The full dump is several hundred bytes. At 115200 baud that is 30-50 ms
  // of blocking, during which GNSS bytes arrive and — with the old 256-byte
  // RX buffer — were dropped. The enlarged buffer covers it, but a production
  // build has no reason to pay the cost at all, so the verbose form is now
  // gated. The one-line form below always runs.
  // --------------------------------------------------------------------
#if CSV_LOG_MODE
  // ---- V5.3 CSV MODE: one line per packet ----
  // Header is printed once at boot (see setup). Field order must not change
  // without updating sim/calibrate_from_log.py.
  // V6: lat/lon/speed/heading are printed DECODED, read back through the same
  // codecs the RDU uses, so quantisation is visible in the log used to measure
  // it. hacc_mm and fix_age_ms stay as TRUE pre-encode values so
  // calibrate_from_log.py can measure quantisation error rather than inherit it.
  Serial.printf("%lu,%lu,%u,%.5f,%.5f,%.1f,%.1f,%u,%u,%u,%lu,%lu,%u,%u,%u,%u,%u\n",
                (unsigned long)(millis() / 1000),   // uptime_s
                (unsigned long)packet.seq,          // seq
                (unsigned)currentTestLeg,           // leg (NOT on the wire)
                gw6DecodeCoord(packet.lat),         // lat (decoded)
                gw6DecodeCoord(packet.lon),         // lon (decoded)
                gw6DecodeSpeed(packet.speed),       // speed_kmph
                gw6DecodeHeading(packet.heading),   // heading_deg
                (unsigned)(headingValid ? 1 : 0),   // heading_valid
                (unsigned)fixType,                  // fix_type
                (unsigned)siv,                      // sats
                (unsigned long)hAccMm,              // hacc_mm (true)
                (unsigned long)fixAgeMs,            // fix_age_ms (true)
                (unsigned)(precisionHigh ? 1 : 0),  // precision_high
                (unsigned)(gpsValid ? 1 : 0),       // gps_valid
                (unsigned)(sirenActive ? 1 : 0),    // emergency
                (unsigned)(sizeof(packet) + 64 +
                           (includeCertThisPacket ? sizeof(GwLeafCert) : 0)),
                (unsigned)(includeCertThisPacket ? 1 : 0));  // cert
#elif RANGE_TEST_BUILD
  // This declaration was MISSING in V5 as well. The verbose branch sits behind
  // `#elif CSV_LOG_MODE`, and CSV_LOG_MODE shipped as 1, so this block was
  // never compiled and the error stayed latent. Switching the log mode is what
  // surfaced it — the V5 block dump has never built.
  //
  // formatEpochAsIST() emits "YYYY-MM-DD HH:MM:SS.mmm IST" = 27 chars + NUL.
  char istTimeStr[40];

  if (gpsEpoch) formatEpochAsIST(gpsEpoch, gpsMs, istTimeStr, sizeof(istTimeStr));
  else          snprintf(istTimeStr, sizeof(istTimeStr), "NO GPS TIME FIX");

  uint32_t packetBytes = (uint32_t)(sizeof(packet) + sizeof(signature) +
                          (includeCertThisPacket ? sizeof(GwLeafCert) : 0));
  uint32_t up = millis() / 1000;

  Serial.printf(
    "\n========== GREENWAVE TX ==========\n"
    "Vehicle ID       : %s\n"
    "Priority (claim) : %u\n\n"
    "Emergency        : %s\n"
    "GPS Valid        : %s\n"
    "Siren            : %s\n\n"
    "Latitude         : %.7f\n"
    "Longitude        : %.7f\n"
    "Altitude         : %s\n\n"
    "Speed            : %.2f km/h\n"
    "Heading          : %.2f deg  [%s]\n\n"
    "Fix Type         : %u (3 = 3D)\n"
    "Satellites       : %u\n"
    "hAcc             : %lu mm  [%s]\n"
    "Fix Age          : %lu ms  (gate %lu ms)\n\n"
    "Sequence         : %lu\n"
    "Test Leg         : %u\n"
    "Fix Epoch (UTC)  : %lu.%03u\n"
    "Fix Epoch (IST)  : %s\n\n"
    "Packet Size      : %u bytes\n"
    "Certificate      : %s\n"
    "Signature        : GENERATED\n"
    "LoRa TX          : SUCCESS\n\n"
    "Free RAM         : %u bytes\n"
    "Uptime           : %02lu:%02lu:%02lu\n"
    "==================================\n",
    VEHICLE_ID_LABEL,
    (unsigned)gw6GetPriority(packet.flags),
    (packet.flags & FLAG_EMERGENCY) ? "YES" : "NO",
    (packet.flags & FLAG_GPS_VALID) ? "YES" : "NO",
    (packet.flags & FLAG_EMERGENCY) ? "ACTIVE" : "OFF",
    gw6DecodeCoord(packet.lat),
    gw6DecodeCoord(packet.lon),
    altitudeValid ? "valid" : "NO FIX",
    gw6DecodeSpeed(packet.speed),
    gw6DecodeHeading(packet.heading),
    headingValid ? "VALID" : "INVALID - IGNORE",
    (unsigned)fixType,
    (unsigned)siv,
    (unsigned long)hAccMm,
    precisionHigh ? "HIGH" : "degraded",
    (unsigned long)fixAgeMs, (unsigned long)MAX_FIX_AGE_MS,
    (unsigned long)packet.seq,
    (unsigned)currentTestLeg,
    (unsigned long)gpsEpoch, (unsigned)gpsMs,
    istTimeStr,
    (unsigned)packetBytes,
    includeCertThisPacket ? (certProvisioned ? "YES (SIGNED)" : "YES (UNPROVISIONED)") : "NO",
    ESP.getFreeHeap(),
    up / 3600, (up % 3600) / 60, up % 60
  );


#else
  Serial.printf("[TX] seq=%lu fix=%u hAcc=%lumm age=%lums v=%.1fkm/h hdg=%s\n",
                (unsigned long)packet.seq, (unsigned)fixType,
                (unsigned long)hAccMm, (unsigned long)fixAgeMs,
                gw6DecodeSpeed(packet.speed),
                headingValid ? "ok" : "invalid");
#endif

  pvtFresh = false;
}

/*
============================================================================
CHANGELOG V4 -> V5
============================================================================
FIX 1  Deleted the static-lock machine (old lines 609-641).
       Was: transmitted a frozen position with speed forced to 0 for at
       least 6 seconds after motion resumed.  Worth up to 67 m at 40 km/h,
       ~100 m at 60 km/h, and it zeroed the speed the ICU needs for ETA.
       Largest single error source in the file.

FIX 2  Deleted the EMA position filter (old lines 602-606).
       An EMA has steady-state lag v*T*(1-a)/a.  At a=0.75 and T=2000 ms
       that is 7.4 m at 40 km/h — a deterministic bias in the direction of
       travel, always toward the wrong side of a geofence boundary.  It was
       also a second, cruder filter stacked on the u-blox internal one.

FIX 3  GNSS is now configured over UBX at every boot: 115200 baud, UBX-only
       (NMEA off), Automotive dynamic model, 10 Hz.  V4 wired GPS_TX and
       never wrote to it, so the module ran in whatever state its backup RAM
       held — different per unit, different after a power cut.

FIX 4  Acceptance gate moved from HDOP to hAcc + fixType.  HDOP describes
       satellite geometry only and is blind to multipath, which is the
       dominant error exactly where this system operates.

FIX 5  Stale-fix gate tightened from 2000 ms to 250 ms, now possible because
       of the 10 Hz rate.  Removes up to 22 m at 40 km/h.

FIX 6  gps_epoch now timestamps THE FIX, not packet assembly.  V4's
       timestamp could be up to 2 s newer than the position it accompanied,
       silently.  This is the precondition for receiver-side dead reckoning.

FIX 7  heading_valid added on flags bit 4.  V4 sent invalid heading as
       0.00 deg = due North, indistinguishable from a real heading, which
       put vehicles on the wrong approach leg.  Uses a previously reserved
       bit, so the wire layout is unchanged and signatures still verify.
       flags bit 5 additionally carries a high-precision indication.

FIX 8  Dead-reckoning contract documented for the RDU/ICU side.  The 2 s
       cadence cannot shrink (duty cycle), so the remaining ~50 m of delay
       error must be removed by projecting the position forward at the
       receiver.  Fixes 1, 6 and 7 are what make that arithmetic valid.

FIX 9  (a) GPS UART RX buffer raised to 1024 B and the verbose diagnostic
       dump gated behind RANGE_TEST_BUILD — it blocked long enough to drop
       GNSS bytes.  (b) Arduino String removed from the serial command path
       and from bytesToHex; it heap-allocated on every character and
       fragmented the heap over long endurance runs.

UNCHANGED ON PURPOSE
       Wire format, TX_INTERVAL_MS = 2000, previousTX = now scheduling,
       transmit-only architecture, Ed25519 signing, sequence anti-rollback,
       LoRa radio parameters, duty-cycle audit, priority_class as advisory.

STILL OPEN — NOT FIXABLE IN THIS FILE
       - RDU trust anchor / certificate enrolment path is unresolved
       - ~37% duty cycle is a band and regulatory decision
       - GNSS spoofing: Ed25519 proves WHO sent it, not whether the position
         is true.  Needs RSSI plausibility at the RDU plus cross-check
         against the acoustic and visual layers.
       - No protocol version byte (see PROTOCOL_V2 block above)
============================================================================

============================================================================
CHANGELOG V5 -> V7   (WIRE FORMAT AND CERTIFICATE BOTH CHANGED — FLAG DAY)
============================================================================
V7-0   RESTORED LoRa.endPacket(). It was commented out in V5, so nothing was
       ever put on the air. Frames were still assembled, signed, and logged
       as "LoRa TX : SUCCESS". Re-check any V5 field data before trusting it.

--- ENTROPY (Open item 1) --------------------------------------------------
V7-1   gwGatherSeed() replaces Ed25519::generatePrivateKey().

       TWO things were wrong in V5 and fixing either alone fixes nothing.

       (a) esp_random() is not a TRNG on this board. The ESP32 hardware RNG
           is fed by RF noise or by the SAR ADC. This unit runs neither WiFi
           nor BT, so with no ADC entropy enabled it degrades to a PSEUDO-
           random sequence. bootloader_random_enable() is what switches the
           SAR ADC on for exactly this case. Safe here because the EVU uses
           only SPI/UART/GPIO; it would NOT be safe on the RDU, which drives
           an INMP441 over I2S.

       (b) Ed25519::generatePrivateKey() never called esp_random() anyway. It
           draws from rweather Crypto's own RNG object, which needs
           RNG.begin() and stirring. Adding esp_fill_random() beside it would
           have changed literally nothing.

       The fix removes that RNG from the trust path rather than seeding it.
       An Ed25519 private key IS 32 uniformly random bytes; clamping happens
       inside derivePublicKey(). This is only safe because Ed25519 SIGNING IS
       DETERMINISTIC (RFC 8032) — the per-signature nonce is hashed from the
       key and message, not drawn from an RNG. After key generation this
       firmware needs no randomness at all, which is a far smaller thing to
       get right than a live RNG.

       Four sources are mixed through SHA-256 (hardware RNG sampled over
       ~80 ms, cross-clock timing jitter, eFuse MAC, hardware RNG again), so
       the output is as strong as the BEST surviving source rather than as
       weak as the worst. The MAC is a per-device DIFFERENTIATOR, not
       entropy — it is public — and its job is to make a fleet-wide key
       collision impossible even if every real source fails.

V7-2   gwRngHealthCheck() runs before key generation and is FATAL on failure:
       stuck-at, repetition run, byte coverage, bit balance. It cannot prove
       the RNG is good — no test of a few kilobytes can — but it catches what
       actually fails on embedded hardware. A unit that will not boot is a
       support call; a unit that boots with a predictable key is a forgeable
       identity that nothing downstream can detect.

V7-3   RNGTEST <kb> dumps RAW generator output for dieharder or the NIST STS.
       The three-boards-different-keys check catches IDENTICAL keys but says
       nothing about LOW-ENTROPY ones. GATED BEHIND RANGE_TEST_BUILD: in a
       production build it is a free oracle on the generator that produced
       the identity key.

--- CERTIFICATES -----------------------------------------------------------
V7-4   NVS key "casig" (64 B bare signature) -> "cert" (108 B GwLeafCert).
       v0 certificates signed the ASCII string "AMB_02:<pubkey hex>" with the
       ROOT key and carried no version, issuer, serial or validity period.
       They are worthless under v1 and are deliberately NOT migrated.

V7-5   SETCERT validates before storing: length, key binding, structure. v0
       persisted any well-formed 64 bytes unchecked, so the first sign of a
       bad provisioning step was a roadside unit rejecting every frame with
       nothing in any log naming the cause.

V7-6   New commands PUBKEY / GETCERT / LOCKPROV, driven by provision_evu.py.
       No private key crosses the serial port in either direction, and no CA
       key of any tier exists on this device.

V7-7   LOCKPROV latches SETCERT closed until a provisioning jumper is fitted
       at boot. An EVU sits in a vehicle for years with a USB port on it.

V7-8   priority_class in the flags now comes from the CA-SIGNED vehicle_class
       in the stored certificate, not from a compile-time constant the device
       asserts about itself. This is the fix for spec defect S2-01: the ICU
       can act on a CA assertion where it could only log a self-assertion.

V7-9   An unprovisioned unit transmits NO certificate block. V5 would have
       sent 96 bytes of zeros, costing airtime to say nothing.

--- WIRE FORMAT (see GreenwaveWireV6.h for per-field justification) ---------
V7-10  payload 33 B -> 20 B, steady frame 97 -> 84 B, 640.0 -> 566.3 ms.
       cert 96 -> 108 B, cert frame 193 -> 192 B, 1180.7 -> 1156.1 ms.
       duty cycle 32.90% -> 29.30% at n=30, which is what the compaction
       itself buys. CERT_INTERVAL_STEADY was subsequently set to 5 for
       cold-start reliability, taking the shipped figure to 34.21%. See
       the note at the CERT_INTERVAL_STEADY definition.
       20 B is not "as small as possible" — it is the TOP of a LoRa airtime
       bucket. Frames of 81-84 B all cost 566.3 ms; byte 21 costs 24.576 ms.

V7-11  vehicle_id off the steady frame entirely. It now travels once inside
       the certificate, where the CA signature binds it to this device's key.
       A SECURITY change as much as a saving: a certified low-priority
       vehicle can no longer assert another vehicle's ID, because the field
       is not in the payload at all.

V7-12  proto_version added; quality byte (hAcc + fix age) added; lat/lon,
       speed, heading, time all re-encoded against gates already in this
       file; test_leg off the wire; FLAG_SIREN deleted as a duplicate of
       FLAG_EMERGENCY.

V7-13  printDutyCycle() derives airtime from sizeof() and the live LORA_*
       settings, and prints the remaining bucket headroom. TOA_STEADY_MS and
       TOA_CERT_MS deleted — hand-maintained constants that silently
       invalidated the one audit whose purpose is catching size changes.

STILL OPEN AFTER V7 — NOT FIXABLE IN THIS FILE
       - 29.3% duty cycle. Compaction took 3.6 points off. The remaining gap
         needs TRANSMISSION GATING (emergency-only + GNSS corridor), which
         moves the hourly average, or a band change to 865-867 MHz.
       - Private key still in PLAINTEXT NVS. Flash encryption WITHOUT secure
         boot does not help: an attacker flashes their own firmware and the
         hardware decrypts NVS for it. Release-mode flash encryption PLUS
         Secure Boot V2 PLUS disabled UART download, all irreversible eFuse
         burns, in the manufacturing flow from board one.
       - Revocation list not yet implemented at the RDU or ICU. With
         long-dated certificates it is the ONLY way to refuse a stolen unit.
       - GNSS spoofing: Ed25519 proves WHO sent it, not whether the position
         is true.
============================================================================
*/
