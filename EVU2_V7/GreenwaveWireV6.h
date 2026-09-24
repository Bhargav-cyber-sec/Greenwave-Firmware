// ============================================================================
// GreenwaveWireV6.h  —  compacted IF-1 wire format (EVU -> RDU)
//
// STATUS: proposal. Not flashed. Not verified against the RDU or ICU trees.
//
// Replaces TelemetryPayload (33 B) with TelemetryPayloadV6 (20 B) while ADDING
// everything the PROTOCOL_V2 block in EVU2.ino wanted — proto_version,
// horizontal accuracy and fix age — and KEEPING altitude at full int16.
//
//   steady frame   97 B -> 84 B     640.0 ms -> 566.3 ms   (-73.7 ms)
//   cert frame    193 B -> 192 B   1180.7 ms -> 1156.1 ms  (richer cert)
//   duty cycle (CERT_INTERVAL_STEADY = 30)   32.90% -> 29.30%
//
// WHY 20 BYTES AND NOT 18 OR 24 ----------------------------------------------
// LoRa payload airtime is quantised into 6-symbol steps (24.576 ms at
// SF9/CR4-6), each step 4.5 bytes wide. At SF9 the buckets are:
//     frame 94-97 B -> 640.0 ms      frame 81-84 B -> 566.3 ms
//     frame 90-93 B -> 615.4 ms      frame 76-79 B -> 541.7 ms
//     frame 85-88 B -> 590.8 ms
// 84 B (20 B payload + 64 B signature) sits exactly at the TOP of a bucket.
// Bytes saved below 20 are free to spend; byte 21 costs a full 24.576 ms.
// Design to the boundary, not to "as small as possible".
//
// This is why altitude_m survives at int16: dropping vehicle_id from the
// steady frame freed 6 bytes, and only 4 of them were needed to reach the
// boundary. The remaining 2 cost nothing.
//
// WHY vehicle_id IS NOT IN THE STEADY FRAME ----------------------------------
// It changes never and it is already bound into the certificate, which the CA
// signs over (vehicle_id || pubkey). Sending it 30 times per certificate is
// 60 wasted bytes per cycle.
//
// It now appears once, inside the certificate block. The RDU caches
// pubkey -> vehicle_id when the cert lands and attributes every subsequent
// steady frame by WHICH CACHED KEY VERIFIED THE SIGNATURE.
//
// Security note, and the real reason to do it: previously the RDU verified the
// signature under the certificate's key and separately read vehicle_id out of
// the payload. Unless it explicitly cross-checked the two, a legitimately
// certified low-priority vehicle could assert another vehicle's ID and the
// ICU's priority registry would honour it. Removing the field from the wire
// makes that attack unconstructible rather than merely guarded against.
//
// An RDU that has not yet cached the certificate cannot attribute a frame —
// but it cannot VERIFY one either, so the existing retro-verification buffer
// (spec 2.3) already covers exactly this window. No new failure mode.
//
// THIS IS A FLAG DAY ---------------------------------------------------------
// Every device reflashed together, ICU first. Copy this file verbatim into the
// RDU and ICU trees in the SAME commit and run verify_rdu_tree.py. There is no
// partial upgrade: proto_version exists so a mismatch is DIAGNOSABLE, not so
// it is survivable.
// ============================================================================

#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "GreenwaveCertV1.h"

#define GW_WIRE_VERSION 6

// ----------------------------------------------------------------------------
// THE STEADY FRAME PAYLOAD
// ----------------------------------------------------------------------------
// Every field size below is justified against a gate that ALREADY EXISTS in
// EVU2.ino, not against taste. If one of those gates is ever retuned, the
// corresponding field must be revisited — the justification is in the comment
// so that is possible.
struct __attribute__((packed)) TelemetryPayloadV6 {
  uint8_t  proto_version;   // = GW_WIRE_VERSION. First byte, always.
                            // EVU2.ino's PROTOCOL_V2 block is right: retrofitting
                            // a version byte later is strictly worse than paying
                            // for it now, while there are only three consumers.

  uint8_t  flags;           // see GW6_FLAG_* below. Now also carries
                            // priority_class, which EVU2.ino line 979 documents
                            // as advisory only and non-decisional.

  uint32_t seq;             // UNCHANGED, deliberately. This is the anti-replay
                            // mechanism. 32 bits at 0.5 Hz is 272 years; the
                            // 2 bytes a uint16 would save are not worth adding a
                            // wrap-handling window to the RDU's replay check.

  uint8_t  tow_ds[3];       // GPS time-of-week of THE FIX, 100 ms per LSB.
                            // 604800 s * 10 = 6,048,000 -> 23 bits, uint24.
                            // Replaces gps_epoch(4) + gps_ms(2).
                            //
                            // gps_ms was 16 bits holding TEN distinct values:
                            // setNavigationFrequency(10) means getMillisecond()
                            // is always a multiple of 100.
                            //
                            // The absolute epoch is dropped because the RDU has
                            // no wall clock and cannot use it (spec 3.3 step 6).
                            // The ICU, which does have a clock, reconstructs the
                            // absolute time from its own week number plus this.

  uint8_t  lat[3];          // int24, degrees * 1e5  ->  1.11 m per LSB
  uint8_t  lon[3];          // range +/-83.8 deg, which covers every landmass
                            // this system will be deployed on.
                            //
                            // MAX_HACC_MM is 10000 (10 m) and PRECISION_HACC_MM
                            // is 3500 (3.5 m). The old int32 * 1e7 encoded to
                            // 1.1 cm — 300x finer than the best fix this
                            // firmware will accept. Receiver-side dead reckoning
                            // (FIX 8) contributes far more error than 1.11 m.

  uint8_t  speed;           // 0.5 km/h per LSB, 0 - 127.5 km/h, clamped.
                            // MAX-M10S ground-speed accuracy is ~0.05 m/s
                            // (~0.18 km/h). The old 0.01 km/h resolution sat 18x
                            // BELOW the sensor's own noise floor.

  uint8_t  heading;         // 360/256 = 1.40625 deg per LSB.
                            // HEADING_MAX_ACC_DEG is 30.0 — this firmware
                            // accepts heading with up to thirty degrees of
                            // error. One byte is still 21x finer than the gate
                            // it has to pass.
                            // Meaningless unless GW6_FLAG_HEADING_VALID is set.

  int16_t  altitude_m;      // UNCHANGED from V5, metres above MSL.
                            // Retained at full precision because bytes 19-20 of
                            // this struct are free (see the bucket note above).
                            // Kept on the assumption that flyover-vs-surface
                            // discrimination matters at Indian intersections.
                            // If no consumer exists, delete it — but deleting it
                            // saves 0 ms, so there is no airtime argument either
                            // way. Decide on merit.

  uint8_t  quality;         // NEW. Replaces PROTOCOL_V2's hacc_cm + fix_age_ms
                            // (4 bytes) with 1, at resolution the consumer can
                            // actually use.
                            //   high nibble : hAcc in whole metres, 0-14,
                            //                 15 = "worse than 14 m"
                            //                 (MAX_HACC_MM is 10 m, so 15 only
                            //                  appears on a rejected fix)
                            //   low  nibble : fix age at TRANSMIT, 16 ms/step,
                            //                 0-15 -> 0-240 ms
                            //                 (MAX_FIX_AGE_MS is 250, so the
                            //                  range covers the whole gate)
                            //
                            // This is what makes the FIX 8 dead-reckoning
                            // contract implementable WITHOUT a shared clock:
                            // the receiver no longer needs t_now - gps_epoch, it
                            // reads the age directly off the wire.
};

static_assert(sizeof(TelemetryPayloadV6) == 20,
              "V6 payload must be exactly 20 B — 21 costs a full 24.576 ms step");

// ----------------------------------------------------------------------------
// THE CERTIFICATE BLOCK  (appended to the steady frame every Nth packet)
// ----------------------------------------------------------------------------
// 108 bytes. The format lives in GreenwaveCertV1.h, which the CA tool mirrors
// byte for byte; this file just names it so the airtime arithmetic below has
// something to measure.
//
// vehicle_id, expiry, serial number and issuer index are all INSIDE the signed
// certificate now. The EVU treats this block as an OPAQUE BLOB: it stores what
// the provisioning station gave it and relays it unmodified. It cannot mint
// one, cannot alter one without breaking the issuer signature, and holds no CA
// key of any kind. That is the whole point.
//
// Cert frame: 84 + 108 = 192 B = 1156.1 ms, against 1106.9 ms for the earlier
// 98-byte block. At CERT_INTERVAL_STEADY = 30 that is 1.6 ms amortised per
// packet — 0.09 percentage points of duty cycle — to gain expiry, serial
// numbers, revocation, two-tier trust and domain separation.
typedef GwLeafCert CertificateBlockV6;

static_assert(sizeof(CertificateBlockV6) == 108, "cert block must be 108 B");

// ---- flags ----
// bit 2 is reclaimed: EVU2.ino sets FLAG_EMERGENCY and FLAG_SIREN from the same
// sirenActive variable (lines 967 and 969). Two bits carrying one fact. That
// reclaimed bit is what lets priority_class move off its own byte.
#define GW6_FLAG_EMERGENCY       (1 << 0)
#define GW6_FLAG_GPS_VALID       (1 << 1)   // position is MEANINGLESS if clear
#define GW6_FLAG_ALT_VALID       (1 << 2)
#define GW6_FLAG_HEADING_VALID   (1 << 3)
#define GW6_FLAG_PRECISION_HIGH  (1 << 4)   // hAcc <= PRECISION_HACC_MM
#define GW6_PRIORITY_SHIFT       5          // bits 5-6, values 0-3
#define GW6_PRIORITY_MASK        0x03
// bit 7 spare

static inline uint8_t gw6GetPriority(uint8_t flags) {
  return (uint8_t)((flags >> GW6_PRIORITY_SHIFT) & GW6_PRIORITY_MASK);
}
static inline void gw6SetPriority(uint8_t *flags, uint8_t p) {
  *flags = (uint8_t)((*flags & (uint8_t)~(GW6_PRIORITY_MASK << GW6_PRIORITY_SHIFT))
                     | (uint8_t)((p & GW6_PRIORITY_MASK) << GW6_PRIORITY_SHIFT));
}

// ----------------------------------------------------------------------------
// int24 / uint24 — little-endian, matching the ESP32 and the rest of the struct
// ----------------------------------------------------------------------------
static inline void gw6PackI24(uint8_t *out, int32_t v) {
  out[0] = (uint8_t)( (uint32_t)v        & 0xFF);
  out[1] = (uint8_t)(((uint32_t)v >>  8) & 0xFF);
  out[2] = (uint8_t)(((uint32_t)v >> 16) & 0xFF);
}

static inline int32_t gw6UnpackI24(const uint8_t *in) {
  uint32_t v = (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16);
  if (v & 0x800000UL) v |= 0xFF000000UL;      // sign extend
  return (int32_t)v;
}

static inline void gw6PackU24(uint8_t *out, uint32_t v) {
  out[0] = (uint8_t)( v        & 0xFF);
  out[1] = (uint8_t)((v >>  8) & 0xFF);
  out[2] = (uint8_t)((v >> 16) & 0xFF);
}

static inline uint32_t gw6UnpackU24(const uint8_t *in) {
  return (uint32_t)in[0] | ((uint32_t)in[1] << 8) | ((uint32_t)in[2] << 16);
}

// ----------------------------------------------------------------------------
// Field codecs
//
// Round, never truncate. Truncation biases toward the equator in the southern
// and western hemispheres — a deterministic error in a fixed direction is
// exactly the kind of thing FIX 2 deleted the EMA filter for. Rounding is free.
//
// TX and RX MUST use these functions rather than open-coding the arithmetic.
// A scale factor written out twice is a scale factor that will eventually
// disagree with itself across three trees.
// ----------------------------------------------------------------------------
#define GW6_COORD_SCALE     100000.0        // 1e-5 deg = 1.11 m
#define GW6_SPEED_LSB_KMPH  0.5
#define GW6_HEADING_LSB_DEG (360.0 / 256.0)
#define GW6_TOW_LSB_MS      100UL
#define GW6_TOW_MODULO      6048000UL       // 604800 s at 100 ms per LSB

static inline void gw6EncodeCoord(uint8_t *out, double deg) {
  double s = deg * GW6_COORD_SCALE;
  if (s >  8388607.0) s =  8388607.0;
  if (s < -8388608.0) s = -8388608.0;
  gw6PackI24(out, (int32_t)llround(s));
}

static inline double gw6DecodeCoord(const uint8_t *in) {
  return (double)gw6UnpackI24(in) / GW6_COORD_SCALE;
}

static inline uint8_t gw6EncodeSpeed(double kmph) {
  if (kmph < 0.0)   kmph = 0.0;
  if (kmph > 127.5) kmph = 127.5;            // CLAMP. Wrapping would turn a
                                             // 130 km/h reading into 2.5 km/h.
  return (uint8_t)lround(kmph / GW6_SPEED_LSB_KMPH);
}

static inline double gw6DecodeSpeed(uint8_t v) {
  return (double)v * GW6_SPEED_LSB_KMPH;
}

static inline uint8_t gw6EncodeHeading(double deg) {
  while (deg <    0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  long v = lround(deg / GW6_HEADING_LSB_DEG);
  return (uint8_t)(v & 0xFF);                // 256 wraps to 0, which is correct
}

static inline double gw6DecodeHeading(uint8_t v) {
  return (double)v * GW6_HEADING_LSB_DEG;
}

// quality: hAcc metres in the high nibble, fix age in 16 ms steps in the low
static inline uint8_t gw6EncodeQuality(uint32_t hAccMm, uint32_t fixAgeMs) {
  uint32_t hi = hAccMm / 1000UL;
  if (hi > 15UL) hi = 15UL;
  uint32_t lo = fixAgeMs / 16UL;
  if (lo > 15UL) lo = 15UL;
  return (uint8_t)((hi << 4) | lo);
}

static inline uint32_t gw6DecodeHAccM(uint8_t q)    { return (uint32_t)(q >> 4); }
static inline uint32_t gw6DecodeFixAgeMs(uint8_t q) { return (uint32_t)(q & 0x0F) * 16UL; }

// GPS time-of-week, 100 ms per LSB
static inline void gw6EncodeTowMs(uint8_t *out, uint32_t towMs) {
  gw6PackU24(out, (towMs / GW6_TOW_LSB_MS) % GW6_TOW_MODULO);
}

static inline uint32_t gw6DecodeTowMs(const uint8_t *in) {
  return gw6UnpackU24(in) * GW6_TOW_LSB_MS;
}

// Seconds-of-week -> TOW milliseconds, for building from broken-down UTC.
// GPS week starts Sunday 00:00:00 UTC. dayOfWeek: 0 = Sunday.
static inline uint32_t gw6TowMsFromUtc(uint8_t dayOfWeek, uint8_t hour,
                                       uint8_t minute, uint8_t second,
                                       uint16_t millis_) {
  return ((uint32_t)dayOfWeek * 86400UL
        + (uint32_t)hour      * 3600UL
        + (uint32_t)minute    * 60UL
        + (uint32_t)second) * 1000UL + (uint32_t)millis_;
}

// ----------------------------------------------------------------------------
// AIRTIME — derived from sizeof(), never from a hand-maintained constant
//
// printDutyCycle() in EVU2.ino V5 read TOA_STEADY_MS / TOA_CERT_MS, which are
// #defines someone has to remember to update. The moment the struct changes,
// the boot-time duty-cycle audit goes quietly wrong — which is precisely the
// failure the audit exists to prevent. Compute it instead.
//
// Semtech SX1276/78 datasheet 4.1.1.7:
//   Tsym  = 2^SF / BW
//   Tpre  = (n_preamble + 4.25) * Tsym
//   n_pay = 8 + max(ceil((8PL - 4SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE)))
//                   * (CR + 4), 0)
// DE (low data rate optimise) is 0 for SF9 at BW125; it is only mandatory at
// SF11/SF12.
//
// Verified against the two measured figures in the existing tree:
//   PL = 97  -> 640.0 ms   (matches TOA_STEADY_MS = 640)
//   PL = 193 -> 1180.7 ms  (matches TOA_CERT_MS   = 1181)
// ----------------------------------------------------------------------------
static inline double gwLoRaAirtimeMs(uint32_t payloadBytes,
                                     uint8_t  sf             = 9,
                                     uint32_t bwHz           = 125000UL,
                                     uint8_t  crDenom        = 6,   // 5..8 = CR4/5..4/8
                                     uint8_t  preamble       = 12,
                                     bool     crcOn          = true,
                                     bool     implicitHdr    = false,
                                     bool     lowDataRateOpt = false) {
  const double tSym = (double)(1UL << sf) / (double)bwHz;
  const double tPre = ((double)preamble + 4.25) * tSym;

  const int de  = lowDataRateOpt ? 1 : 0;
  const int ih  = implicitHdr    ? 1 : 0;
  const int crc = crcOn          ? 1 : 0;
  const int cr  = (int)crDenom - 4;

  const double num = 8.0 * (double)payloadBytes - 4.0 * (double)sf
                   + 28.0 + 16.0 * (double)crc - 20.0 * (double)ih;
  const double den = 4.0 * ((double)sf - 2.0 * (double)de);

  double steps = ceil(num / den);
  if (steps < 0.0) steps = 0.0;

  const double nPayload = 8.0 + steps * (double)(cr + 4);
  return (tPre + nPayload * tSym) * 1000.0;
}

// How many bytes can be added to this frame before the airtime steps up.
// Call this before adding a field: if it returns 3, the next three bytes are
// free and the fourth costs 24.576 ms. Makes "adding a frame field costs
// airtime" (spec Part 6 rule 7) a number instead of a warning.
static inline uint32_t gwFreeBytesInAirtimeBucket(uint32_t payloadBytes,
                                                  uint8_t  sf      = 9,
                                                  uint8_t  crDenom = 6,
                                                  uint8_t  preamble = 12) {
  const double here = gwLoRaAirtimeMs(payloadBytes, sf, 125000UL, crDenom, preamble);
  uint32_t n = 0;
  while (n < 16 &&
         gwLoRaAirtimeMs(payloadBytes + n + 1, sf, 125000UL, crDenom, preamble)
           <= here + 0.01) {
    n++;
  }
  return n;
}
