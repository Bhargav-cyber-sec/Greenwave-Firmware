// ============================================================================
// GreenwaveCertV1.h — two-tier certificate format, shared by EVU / RDU / ICU
//
// STATUS: proposal. Not flashed. Copy VERBATIM into all three trees.
//
// Replaces the v0 certificate, which was a bare 64-byte Ed25519 signature over
// the ASCII string  "AMB_02:<pubkey hex>"  — no version, no issuer, no serial,
// no validity period, no domain separation, and an unconstrained vehicle_id
// whose ':' delimiter could be injected.
//
// ----------------------------------------------------------------------------
// TRUST CHAIN
// ----------------------------------------------------------------------------
//   ROOT CA        air-gapped. Key on removable media or a hardware token.
//     |            Signs ISSUER certificates ONLY, a few times per decade.
//     |            NEVER signs an EVU.
//     v
//   ISSUER CA      depot provisioning laptop. Signs EVU leaf certificates.
//     |            Compromise costs ONE issuer, killed by pushing an updated
//     |            issuer table to six RDUs. Root stays intact.
//     v
//   EVU LEAF       in the vehicle. Signs its own beacons at 0.5 Hz.
//                  Private key generated ON DEVICE, never leaves it.
//
// NO CA PRIVATE KEY OF ANY TIER EXISTS ON AN EVU, RDU OR ICU. An EVU that
// could issue certificates would be a total-compromise device: it is
// operator-accessible and its flash is unencrypted, so one stolen unit would
// mint identities for the entire fleet.
//
// ----------------------------------------------------------------------------
// WHY THERE IS NO SHORT EXPIRY, AND WHAT CARRIES THE WEIGHT INSTEAD
// ----------------------------------------------------------------------------
// Operational requirement: a working EVU must never need to be touched. The
// only acceptable reason to open one is that it is already being serviced. So
// certificates are issued LONG-DATED (default 2050) rather than rotated.
//
// This is also forced by the architecture. The RDU has no wall clock; its only
// time source is a field asserted by the entity being authenticated, which is
// circular (spec 3.3 step 6). Expiry can therefore only ever be enforced at
// the ICU. The RDU passes not_after through and does not judge it.
//
// `not_after_days` still exists deliberately. Shortening a long-dated
// deployment later then becomes a policy change, not a wire-format flag day.
//
// WHAT ACTUALLY PROTECTS YOU IS `serial`.
// With no meaningful expiry, revocation is the ONLY way to refuse a stolen
// unit. A unit pulled from a scrapped ambulance has unencrypted flash and
// gives up its private key in about ten minutes. Without a serial number there
// is no way to NAME that unit and therefore no way to ever say no to it.
// Every RDU and ICU MUST implement gwCertIsRevoked(). A deployment that skips
// the revocation list has no answer to a stolen EVU, permanently.
//
// ----------------------------------------------------------------------------
// DOMAIN SEPARATION
// ----------------------------------------------------------------------------
// Signature preimages are CONTEXT || 0x00 || TBS, with a different context per
// certificate type. This stops a signature issued in one role being presented
// in another — an issuer cert replayed as a leaf cert, or a future
// firmware-signing signature accepted as an identity. The context is NOT
// transmitted; both sides construct it. Zero bytes of airtime.
//
// The 0x00 terminator is load-bearing. Without it, two contexts where one is a
// prefix of the other can produce colliding preimages.
//
// ----------------------------------------------------------------------------
// AIRTIME
// ----------------------------------------------------------------------------
//   V6 cert block          98 B -> frame 182 B -> 1106.9 ms
//   V7 cert block (this)  108 B -> frame 192 B -> 1156.1 ms
// At CERT_INTERVAL_STEADY = 30 that is +1.6 ms amortised, duty cycle
// 29.21% -> 29.30%.
//
// 108 B is the TOP of a LoRa airtime bucket: cert blocks of 105-108 B all cost
// 1156.1 ms. Every field below is therefore already paid for, and there is NO
// headroom left — a 109th byte costs a full 24.576 ms step.
// ============================================================================

#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define GW_CERT_VERSION  1

// ---- domain separation contexts (constructed locally, never transmitted) ----
#define GW_CTX_LEAF    "greenwave-evu-cert-v1"
#define GW_CTX_ISSUER  "greenwave-issuer-cert-v1"
// RESERVED — do not reuse either context above for these purposes:
//   "greenwave-firmware-v1"  firmware signing, if it is ever added
//   "greenwave-crl-v1"       signed revocation lists

// ---- compact date encoding ----
// Days since 2020-01-01. uint16 spans 179 years, to 2199.
#define GW_DATE_EPOCH_UNIX  1577836800UL   // 2020-01-01T00:00:00Z
#define GW_DATE_NEVER       0xFFFF
// GW_DATE_NEVER is accepted but NOT recommended. It makes the serial number the
// single mechanism standing between you and a stolen unit. A 2050 date is
// operationally identical across any vehicle's service life and keeps a second
// backstop.

// ============================================================================
// EVU LEAF CERTIFICATE — 108 bytes on the wire
// ============================================================================
// Fixed-width little-endian binary. Python side packs the TBS as
//   struct.pack('<BBHIHBB32s', ...)
// which produces exactly this layout. Field order is load-bearing: the
// signature covers these bytes in this order, so reordering invalidates every
// certificate ever issued.
struct __attribute__((packed)) GwLeafCertTBS {   // 44 B — the signed bytes
  uint8_t  cert_version;     // = GW_CERT_VERSION
  uint8_t  issuer_id;        // which issuer signed this. 1-254.
                             // 0 = invalid, 255 = reserved for the root.
  uint16_t vehicle_id;       // numeric fleet ID, matches the V6 wire format.
                             // 0 = unassigned, always rejected.
  uint32_t serial;           // unique per issuer, monotonic, never reused.
                             // THE REVOCATION HANDLE — see the note above.
  uint16_t not_after_days;   // days since 2020-01-01, or GW_DATE_NEVER.
                             // Enforced at the ICU only; the RDU has no clock.
  uint8_t  vehicle_class;    // CA-AUTHORISED priority, 0-3.
                             //
                             // This is the fix for spec defect S2-01. The
                             // priority_class the EVU puts in its telemetry
                             // flags is advisory because the vehicle asserts it
                             // about itself. THIS field is asserted by the CA,
                             // so the ICU can act on it directly. A
                             // disagreement between the two remains a useful
                             // security signal: a unit claiming an importance
                             // it was not granted.
  uint8_t  capabilities;     // reserved bitfield, MUST be 0 in v1.
                             // Intended for corridor or region restrictions.
                             // Verifiers MUST REJECT unknown bits rather than
                             // ignore them — ignoring means a future
                             // restriction can be stripped and not noticed.
  uint8_t  pubkey[32];       // the EVU's Ed25519 public key
};

struct __attribute__((packed)) GwLeafCert {      // 108 B — what goes on the air
  GwLeafCertTBS tbs;
  uint8_t       sig[64];     // Ed25519(issuer_priv, GW_CTX_LEAF || 0x00 || tbs)
};

static_assert(sizeof(GwLeafCertTBS) == 44, "leaf TBS must be 44 B");
static_assert(sizeof(GwLeafCert) == 108,
              "leaf cert must be 108 B — the top of a LoRa airtime bucket. "
              "A 109th byte costs a full 24.576 ms step.");

// ============================================================================
// ISSUER CERTIFICATE — 104 bytes, NEVER on the IF-1 wire
// ============================================================================
// Lives in RDU and ICU firmware, or in their NVS, added at maintenance.
//
// Why these are SIGNED OBJECTS rather than bare public keys compiled in: RDU
// flash is unencrypted and the units are unattended and physically reachable.
// A bare key table can be edited in place with an SOIC-8 clip. A table of
// root-signed certificates means an attacker must also forge a ROOT signature
// to insert their own issuer.
//
// It does not defeat wholesale firmware replacement — only secure boot does
// that — but it closes the NVS-editing path, and it lets a new issuer be
// distributed WITHOUT reflashing six roadside units.
struct __attribute__((packed)) GwIssuerCertTBS {  // 40 B
  uint8_t  cert_version;     // = GW_CERT_VERSION
  uint8_t  issuer_id;        // 1-254, matches GwLeafCertTBS::issuer_id
  uint16_t not_after_days;
  uint32_t serial;           // so an issuer can itself be revoked
  uint8_t  pubkey[32];       // this issuer's Ed25519 public key
};

struct __attribute__((packed)) GwIssuerCert {     // 104 B
  GwIssuerCertTBS tbs;
  uint8_t         sig[64];   // Ed25519(root_priv, GW_CTX_ISSUER || 0x00 || tbs)
};

static_assert(sizeof(GwIssuerCertTBS) == 40, "issuer TBS must be 40 B");
static_assert(sizeof(GwIssuerCert) == 104, "issuer cert must be 104 B");

// ============================================================================
// SIGNATURE PREIMAGE CONSTRUCTION
// ============================================================================
// Both sides MUST build preimages with these functions. A context string
// written out by hand in two trees is a context string that will eventually
// disagree, and the symptom is universal signature failure with nothing in any
// log naming the cause — exactly the failure Part 6 rule 1 exists to prevent.
#define GW_PREIMAGE_MAX 96

static inline size_t gwBuildLeafPreimage(uint8_t *out, size_t outLen,
                                         const GwLeafCertTBS *tbs) {
  const size_t ctxLen = sizeof(GW_CTX_LEAF);        // includes the NUL
  const size_t total  = ctxLen + sizeof(GwLeafCertTBS);
  if (outLen < total) return 0;
  memcpy(out, GW_CTX_LEAF, ctxLen);
  memcpy(out + ctxLen, tbs, sizeof(GwLeafCertTBS));
  return total;
}

static inline size_t gwBuildIssuerPreimage(uint8_t *out, size_t outLen,
                                           const GwIssuerCertTBS *tbs) {
  const size_t ctxLen = sizeof(GW_CTX_ISSUER);
  const size_t total  = ctxLen + sizeof(GwIssuerCertTBS);
  if (outLen < total) return 0;
  memcpy(out, GW_CTX_ISSUER, ctxLen);
  memcpy(out + ctxLen, tbs, sizeof(GwIssuerCertTBS));
  return total;
}

// ============================================================================
// STATUS CODES
// ============================================================================
typedef enum {
  GW_CERT_OK = 0,
  GW_CERT_BAD_VERSION,
  GW_CERT_BAD_ISSUER,
  GW_CERT_BAD_VEHICLE,
  GW_CERT_BAD_SERIAL,
  GW_CERT_BAD_CLASS,
  GW_CERT_UNKNOWN_CAPABILITY,
  GW_CERT_UNTRUSTED_ISSUER,
  GW_CERT_REVOKED,
  GW_CERT_BAD_SIGNATURE,
  GW_CERT_EXPIRED,            // ICU only
  GW_CERT_KEY_MISMATCH        // EVU self-check at provisioning time
} GwCertStatus;

static inline const char *gwCertStatusName(GwCertStatus s) {
  switch (s) {
    case GW_CERT_OK:                 return "OK";
    case GW_CERT_BAD_VERSION:        return "BAD_VERSION";
    case GW_CERT_BAD_ISSUER:         return "BAD_ISSUER";
    case GW_CERT_BAD_VEHICLE:        return "BAD_VEHICLE";
    case GW_CERT_BAD_SERIAL:         return "BAD_SERIAL";
    case GW_CERT_BAD_CLASS:          return "BAD_CLASS";
    case GW_CERT_UNKNOWN_CAPABILITY: return "UNKNOWN_CAPABILITY";
    case GW_CERT_UNTRUSTED_ISSUER:   return "UNTRUSTED_ISSUER";
    case GW_CERT_REVOKED:            return "REVOKED";
    case GW_CERT_BAD_SIGNATURE:      return "BAD_SIGNATURE";
    case GW_CERT_EXPIRED:            return "EXPIRED";
    case GW_CERT_KEY_MISMATCH:       return "KEY_MISMATCH";
  }
  return "UNKNOWN";
}

// ============================================================================
// STRUCTURAL VALIDATION — no crypto. Call BEFORE verifying any signature.
// ============================================================================
// Cheapest checks first, mirroring the ICU's 8-step order. Under a flood of
// malformed certificates the system then spends integer comparisons rather
// than Ed25519 verifications, which are orders of magnitude more expensive.
static inline GwCertStatus gwCertCheckStructure(const GwLeafCertTBS *t) {
  if (t->cert_version != GW_CERT_VERSION)         return GW_CERT_BAD_VERSION;
  if (t->issuer_id == 0 || t->issuer_id == 255)   return GW_CERT_BAD_ISSUER;
  if (t->vehicle_id == 0)                         return GW_CERT_BAD_VEHICLE;
  if (t->serial == 0)                             return GW_CERT_BAD_SERIAL;
  if (t->vehicle_class > 3)                       return GW_CERT_BAD_CLASS;
  if (t->capabilities != 0)                       return GW_CERT_UNKNOWN_CAPABILITY;
  return GW_CERT_OK;
}

// ICU ONLY. Never call this on an RDU — its only time source is a field
// asserted by the entity being authenticated, which is circular.
static inline GwCertStatus gwCertCheckValidity(const GwLeafCertTBS *t,
                                               uint32_t nowUnix) {
  if (t->not_after_days == GW_DATE_NEVER) return GW_CERT_OK;
  const uint32_t notAfter =
      GW_DATE_EPOCH_UNIX + (uint32_t)t->not_after_days * 86400UL;
  return (nowUnix > notAfter) ? GW_CERT_EXPIRED : GW_CERT_OK;
}

// ============================================================================
// REVOCATION — MANDATORY on every RDU and ICU
// ============================================================================
// With long-dated certificates this is the only mechanism that can ever refuse
// a stolen unit. 5 bytes per entry; 200 revoked units is 1 KB, which fits in
// NVS with room to spare and can be updated at maintenance without reflashing
// provided the list itself is signed under a "greenwave-crl-v1" context.
//
// An empty list is a valid STARTING state. It is not a valid permanent state
// for a deployed fleet.
struct __attribute__((packed)) GwRevokedEntry {
  uint8_t  issuer_id;
  uint32_t serial;
};

static_assert(sizeof(GwRevokedEntry) == 5, "revocation entry must be 5 B");

static inline bool gwCertIsRevoked(const GwLeafCertTBS *t,
                                   const GwRevokedEntry *list, size_t count) {
  for (size_t i = 0; i < count; i++) {
    if (list[i].issuer_id == t->issuer_id && list[i].serial == t->serial) {
      return true;
    }
  }
  return false;
}

// ============================================================================
// TRUST STORE — what an RDU or ICU holds
// ============================================================================
// Populate `issuers` from export-rdu-header output at build time, or from
// signed issuer certificates in NVS at maintenance. EVERY entry MUST have been
// verified under ROOT_CA_PUBLIC_KEY at boot before it is used. Log loudly on
// failure: an issuer certificate that does not verify means either flash
// corruption or tampering with an unattended roadside unit, and both are
// findings.
#define GW_MAX_ISSUERS  8

struct GwTrustStore {
  uint8_t               rootPublicKey[32];
  GwIssuerCertTBS       issuers[GW_MAX_ISSUERS];   // verified copies only
  uint8_t               issuerCount;
  const GwRevokedEntry *crl;
  size_t                crlCount;
};

static inline const GwIssuerCertTBS *gwFindIssuer(const GwTrustStore *ts,
                                                  uint8_t issuerId) {
  for (uint8_t i = 0; i < ts->issuerCount; i++) {
    if (ts->issuers[i].issuer_id == issuerId) return &ts->issuers[i];
  }
  return NULL;
}

// ============================================================================
// FULL VERIFICATION — reference order for the RDU
// ============================================================================
// The Ed25519 call differs between trees (the EVU uses rweather Crypto; the
// RDU and ICU have their own), so this is expressed as a callback. Implement
// it in EXACTLY this order:
//
//   1  structure         cheap integer field tests
//   2  issuer lookup     unknown issuer => UNTRUSTED_ISSUER, no crypto spent
//   3  revocation        a revoked serial never reaches a signature check
//   4  preimage          GW_CTX_LEAF || 0x00 || tbs
//   5  Ed25519 verify    under the ISSUER's key, not the root's
//   6  cache             pubkey -> {vehicle_id, vehicle_class, issuer, serial}
//   7  ICU only          gwCertCheckValidity()
//
// NEVER log key material, a shared secret, or a key fingerprint (Part 6 rule
// 6) — a fingerprint is a free confirmation oracle for offline brute-forcing.
// DO log issuer_id and serial. Those are public identifiers and they are
// precisely what an operator needs in order to revoke a unit.
typedef bool (*GwEd25519VerifyFn)(const uint8_t sig[64],
                                  const uint8_t *msg, size_t msgLen,
                                  const uint8_t pubkey[32]);

static inline GwCertStatus gwVerifyLeafCert(const GwLeafCert *cert,
                                            const GwTrustStore *ts,
                                            GwEd25519VerifyFn verifyFn) {
  GwCertStatus st = gwCertCheckStructure(&cert->tbs);
  if (st != GW_CERT_OK) return st;

  const GwIssuerCertTBS *iss = gwFindIssuer(ts, cert->tbs.issuer_id);
  if (iss == NULL) return GW_CERT_UNTRUSTED_ISSUER;

  if (gwCertIsRevoked(&cert->tbs, ts->crl, ts->crlCount)) return GW_CERT_REVOKED;

  uint8_t pre[GW_PREIMAGE_MAX];
  size_t  preLen = gwBuildLeafPreimage(pre, sizeof(pre), &cert->tbs);
  if (preLen == 0) return GW_CERT_BAD_SIGNATURE;

  if (!verifyFn(cert->sig, pre, preLen, iss->pubkey)) return GW_CERT_BAD_SIGNATURE;
  return GW_CERT_OK;
}

// Verify one issuer certificate under the root key. Run at BOOT for every
// entry before loading it into the trust store.
static inline GwCertStatus gwVerifyIssuerCert(const GwIssuerCert *cert,
                                              const uint8_t rootPublicKey[32],
                                              GwEd25519VerifyFn verifyFn) {
  if (cert->tbs.cert_version != GW_CERT_VERSION)       return GW_CERT_BAD_VERSION;
  if (cert->tbs.issuer_id == 0 || cert->tbs.issuer_id == 255)
                                                       return GW_CERT_BAD_ISSUER;
  if (cert->tbs.serial == 0)                           return GW_CERT_BAD_SERIAL;

  uint8_t pre[GW_PREIMAGE_MAX];
  size_t  preLen = gwBuildIssuerPreimage(pre, sizeof(pre), &cert->tbs);
  if (preLen == 0) return GW_CERT_BAD_SIGNATURE;

  if (!verifyFn(cert->sig, pre, preLen, rootPublicKey)) return GW_CERT_BAD_SIGNATURE;
  return GW_CERT_OK;
}

// ============================================================================
// EVU-SIDE PROVISIONING SELF-CHECK
// ============================================================================
// The EVU cannot be given a CA private key, but it CAN be given the PUBLIC
// keys — those leak nothing by definition. Doing so lets the device refuse a
// bad certificate at the serial port instead of silently transmitting one that
// every RDU rejects with nothing in any log naming the cause.
//
// This catches the single most common provisioning error by a wide margin:
// pasting unit A's certificate into unit B.
static inline GwCertStatus gwCertMatchesOwnKey(const GwLeafCert *cert,
                                               const uint8_t ownPublicKey[32]) {
  if (memcmp(cert->tbs.pubkey, ownPublicKey, 32) != 0) return GW_CERT_KEY_MISMATCH;
  return GW_CERT_OK;
}
