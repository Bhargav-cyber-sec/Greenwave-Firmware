#ifndef GREENWAVE_CRYPTO_H
#define GREENWAVE_CRYPTO_H

/***********************************************************************
 * GREENWAVE IF-2 AUTHENTICATION  --  v4
 *
 * Implements: "GreenWave EVP - RDU-ICU Implementation SOP, Phase 2
 *              Part 1: RDU -> ICU Secure Link"
 *
 *   SOP 3.1  identity / keypairs
 *   SOP 3.2  X25519 ECDH + HKDF-SHA256 session key derivation
 *   SOP 3.3  per-packet tag  (HMAC-SHA256 truncated -- see WHY below)
 *   SOP 3.4  time-based re-key, session_epoch
 *   SOP 3.5  revocation at session establishment
 *   SOP 5.3  epoch-overlap acceptance window
 *
 * ===================================================================
 * WHAT THIS REPLACES, AND WHY
 * ===================================================================
 *
 * v3 used a static table, GW_NODE_KEYS[6][16], in this header. Because
 * this header is compiled into BOTH LaneNode.ino and ICU.ino, every
 * lane node's firmware image contained ALL SIX node keys. Dump the
 * flash of any one roadside enclosure and you could impersonate every
 * node at that intersection -- including forging the two-node
 * corroboration that production mode depends on.
 *
 * That is the actual defect this change closes (SOP 3.2). Note it is
 * NOT an airtime win: the SOP's "-48 to -56 bytes" figure is measured
 * against a v4-architecture design that used a 64-byte Ed25519
 * signature per packet, which this codebase never implemented. Against
 * the shipped 8-byte CMAC, v4 costs +4 bytes/frame (+25 ms airtime).
 * Do not repeat the airtime claim internally; it is inverted here.
 *
 * Now: each RDU holds only ITS OWN private key. The ICU holds its own
 * private key plus the RDUs' public keys. There is no table of other
 * devices' secrets anywhere in the system.
 *
 * ===================================================================
 * WHY HMAC-SHA256 AND NOT ChaCha20-Poly1305   (SOP 3.3 / SOP 7 item 1)
 * ===================================================================
 *
 * SOP 3.3 names ChaCha20-Poly1305 as primary and truncated HMAC-SHA256
 * as an acceptable alternative, and SOP 7 leaves the choice to the
 * team. HMAC was chosen deliberately:
 *
 *  1. NO NONCE. Poly1305 requires a nonce that must never repeat under
 *     one key; a repeat is key recovery -- total, silent, permanent. A
 *     safe nonce is constructible here (session_epoch || counter) but it
 *     rests on two independently NVS-persisted monotonic counters both
 *     surviving every brownout, forever. HMAC has no nonce, so nonce
 *     reuse is not merely unlikely, it is not a thing that exists.
 *
 *  2. FAILURE SEVERITY IS ASYMMETRIC. If the counter wraps or NVS
 *     corrupts: under HMAC, replay detection degrades -- logged,
 *     counted, recoverable. Under Poly1305, the key is recoverable by
 *     an attacker and nothing in the logs would ever show it.
 *
 *  3. NO CONFIDENTIALITY NEEDED. SOP 3.3 states this hop's contents are
 *     not confidentiality-sensitive. ChaCha20-Poly1305's advantage is
 *     "encryption for free later" -- taking on a catastrophic failure
 *     mode today for an option that may never be exercised.
 *
 *  4. ESP32-S3 has hardware SHA acceleration (SOP 3.3 concedes this),
 *     and mbedtls_md_hmac is a 3-call API.
 *
 * Tag is 8 bytes (SOP 3.3 permits 8-16), matching the outgoing CMAC tag
 * so per-frame airtime does not grow on the tag account.
 *
 * ===================================================================
 * FORWARD SECRECY -- READ THIS BEFORE QUOTING SOP 3.4
 * ===================================================================
 *
 * This is static-static ECDH: both keypairs are long-term, so the raw
 * shared secret is FIXED FOREVER. session_epoch only varies the HKDF
 * salt. Therefore:
 *
 *   Rotation bounds the lifetime of a leaked SESSION key.
 *   It does NOT protect against a compromised DEVICE key.
 *
 * An attacker holding an RDU's private key derives every session key
 * for that node, past and future. There is no forward secrecy. This is
 * an accepted trade -- the ICU is receive-only with no downlink, so
 * there is no channel for an ephemeral handshake -- but SOP 3.4's
 * phrase "bounded useful lifetime" overstates it, and anyone reviewing
 * this should know the real boundary.
 ***********************************************************************/

#include <Arduino.h>
#include <string.h>
#include <Preferences.h>

#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/bignum.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/md.h"
#include "esp_system.h"
#include "esp_random.h"

#include "GreenwaveTypes.h"

// ---------------------------------------------------------------- sizes
/***********************************************************************
 * FAILURE REPORTING
 *
 * gwX25519()/gwDeriveSessionKey() previously returned a bare bool and
 * discarded the mbedTLS return code, so a derivation failure produced
 * "[SESS] derive=FAILED" and nothing actionable -- the same silent-failure
 * pattern this project has already been bitten by three times. Every
 * mbedTLS call now records its step and error code.
 ***********************************************************************/
static int         gwLastErr  = 0;
static const char *gwLastStep = "none";

static inline void gwClearErr(void) { gwLastErr = 0; gwLastStep = "none"; }

// Latched at the first derivation failure so the reason is still reportable
// long after the boot banner has scrolled off the serial monitor. Diagnostic
// state must outlive the terminal buffer.
static uint32_t    gwOverlapHits = 0;   // SOP 5.3 window used (benign, expected at rotation)
static int         gwFailErr  = 0;
static const char *gwFailStep = "none";
static bool        gwFailSelfTestOk = false;
#define GW_FAIL(step, rc)  do { gwLastStep = (step); gwLastErr = (rc); goto done; } while (0)

// Human-readable hint for the codes that actually occur here.
static inline const char *gwErrHint(int rc) {
    switch (rc) {
        case -0x4F80: return "ECP_FEATURE_UNAVAILABLE: Curve25519 not compiled into mbedTLS";
        case -0x4F00: return "ECP_BAD_INPUT_DATA: bad key length or malformed point";
        case -0x4E80: return "ECP_INVALID_KEY: scalar not clamped, or invalid public point";
        case -0x4D00: return "ECP_ALLOC_FAILED: out of heap";
        case -0x0010: return "MPI_ALLOC_FAILED: out of heap";
        default:      return "see mbedtls/error.h";
    }
}

#define GW_TAG_LEN        8     // truncated HMAC-SHA256 (SOP 3.3: 8-16)
#define GW_SESSION_KEY_LEN 32   // HKDF-SHA256 output
#define GW_X25519_LEN     32    // scalar and u-coordinate are both 32B
#define GW_HKDF_INFO      "rdu-icu-session-v1"   // SOP 3.2, verbatim

// Re-key cadence, SOP 3.4. Time-based, not packet-count -- SOP 3.4
// explicitly recommends time-based for this hop.
#define GW_REKEY_INTERVAL_MS  60000UL

/***********************************************************************
 * SOP 5.1 / 5.3 -- CERTIFICATE OVER THE AIR
 *
 * Disabled by default. With RDU public keys pre-provisioned at the ICU
 * (six fixed nodes per intersection), session_epoch rides in every
 * frame header, so the ICU derives the key for whatever epoch it sees
 * on any steady-state frame. There is no session-establishment frame at
 * all and no on-air cost.
 *
 * Enabling this adds a 118-byte / 763 ms frame every 60 s -- the
 * largest thing this system would ever transmit, 1.8x a normal event,
 * on a link where transmit time is directly ambulance-deafness, resent
 * forever because there is no downlink to confirm receipt.
 *
 * Turn it on only if you need field-swappable RDUs without reflashing
 * the ICU. Revocation (SOP 3.5) works identically either way -- it is a
 * lookup on node identity, not on certificate presence.
 ***********************************************************************/
#define GW_CERT_ON_AIR 0

// ==================================================================
// KEY MATERIAL
//
// PLACEHOLDER -- REPLACE BEFORE DEPLOYMENT.
//
// Generate with a real CSPRNG. These values are in a file that has been
// in a chat log and a git history; treat them as public. Long term the
// private half belongs in the ATECC608A (CMP-07) or ESP32-S3 eFuse with
// flash encryption enabled -- see FUTURE_WORK.md 1.1, which is still an
// open hardware decision.
//
// PROVISIONING NOTE: SOP 3.1 asserts each RDU already has an Ed25519
// keypair from Phase 1 and that no provisioning change is needed. That
// is NOT true of this codebase -- LaneNode.ino held only
// ROOT_CA_PUBLIC_KEY, with no private key and no RDU certificate. The
// RDU identity did not exist and is created here.
//
// Because the identity is new, these are NATIVE X25519 keypairs. We do
// NOT do the Ed25519->X25519 birational transform SOP 3.1 describes.
// That transform (u = (1+y)/(1-y) mod 2^255-19) would be hand-rolled
// field inversion with no vetted test vectors in this build, and its
// only purpose was avoiding re-enrollment -- which we are doing anyway.
// Deleting the requirement deletes the risk.
// ==================================================================

/***********************************************************************
 * ROLE-GATED COMPILATION
 *
 * GW_ROLE_LANE_NODE / GW_ROLE_ICU are defined by the sketch BEFORE this
 * header is included (the hook added in v3.6 for exactly this purpose).
 *
 * This matters more than it looks. Without the guards, both private keys
 * compile into both images: every lane node would carry the ICU's private
 * key, and the ICU would carry an RDU private key. Opening one roadside
 * enclosure would then yield the ICU identity and let an attacker
 * impersonate ANY node -- reintroducing precisely the blast radius that
 * v3's GW_NODE_KEYS table had and that this whole change exists to remove.
 *
 * A key that is not compiled in cannot be extracted from the image.
 ***********************************************************************/
#if !defined(GW_ROLE_LANE_NODE) && !defined(GW_ROLE_ICU)
#error "Define GW_ROLE_LANE_NODE or GW_ROLE_ICU before including GreenwaveCrypto.h"
#endif
#if defined(GW_ROLE_LANE_NODE) && defined(GW_ROLE_ICU)
#error "Define exactly one of GW_ROLE_LANE_NODE / GW_ROLE_ICU, not both"
#endif

/***********************************************************************
 * v4.5 -- PROVISIONED KEY SWITCH
 *
 * 0 = compile the PLACEHOLDER keys below (current bench behaviour).
 * 1 = compile keys from GreenwaveKeys.h (per-device, from
 *     gw_provision_keys.py).
 *
 * THE PLACEHOLDER KEYS ARE THE RFC 7748 SECTION 6.1 TEST VECTORS.
 * They are the published Alice/Bob keypairs from an IETF standard. They
 * form a mathematically consistent pair, which is exactly why the link
 * works and nothing complains -- and why anyone who can read that RFC can
 * derive your session key for any epoch and forge any IF-2 frame.
 *
 * The bug in v4 was not that the placeholders were present. It was that
 * they were SILENT: nothing at compile time or boot time objected, so a
 * fully working bench looked indistinguishable from a secured one. That
 * silence is fixed below; the key VALUES are deliberately unchanged so an
 * already-flashed bench keeps working.
 *
 * TO GO SECURE: set this to 1, drop the matching per-device
 * GreenwaveKeys.h into each sketch folder (they are DIFFERENT files with
 * the same name -- the one case where the two folders must NOT match),
 * and reflash both ends together.
 ***********************************************************************/
#define GW_USE_PROVISIONED_KEYS 1

#if GW_USE_PROVISIONED_KEYS
  #include "GreenwaveKeys.h"
  #ifndef GW_KEY_SET_ID
    #error "GreenwaveKeys.h did not define GW_KEY_SET_ID - wrong or missing key file"
  #endif
#else
  #warning "GreenWave: building with RFC 7748 PLACEHOLDER keys. IF-2 authentication provides NO security. Set GW_USE_PROVISIONED_KEYS to 1 before deployment."
  #define GW_KEY_SET_ID "PLACEHOLDER-RFC7748"
#endif

#if !GW_USE_PROVISIONED_KEYS
#ifdef GW_ROLE_LANE_NODE
// ---- RDU side: this node's own private key. ----------------------
// Each node is flashed with ITS OWN key here. Nothing else. This block
// is NOT compiled into the ICU image.
static const uint8_t GW_RDU_PRIVATE_KEY[GW_X25519_LEN] = {
    // PLACEHOLDER -- REPLACE BEFORE DEPLOYMENT (unique per node)
    0x77,0x07,0x6d,0x0a,0x73,0x18,0xa5,0x7d,0x3c,0x16,0xc1,0x72,0x51,0xb2,0x66,0x45,
    0xdf,0x4c,0x2f,0x87,0xeb,0xc0,0x99,0x2a,0xb1,0x77,0xfb,0xa5,0x1d,0xb9,0x2c,0x2a
};

// ---- ICU public key, known to every RDU (SOP 3.1). ---------------
// Public. Safe in a lane-node image.
static const uint8_t GW_ICU_PUBLIC_KEY_LN[GW_X25519_LEN] = {
    // PLACEHOLDER -- must match GW_ICU_PRIVATE_KEY on the ICU. Derive with
    // gwDerivePublicKey() at provisioning; do not compute at runtime.
    0xde,0x9e,0xdb,0x7d,0x7b,0x7d,0xc1,0xb4,0xd3,0x5b,0x61,0xc2,0xec,0xe4,0x35,0x37,
    0x3f,0x83,0x43,0xc8,0x5b,0x78,0x67,0x4d,0xad,0xfc,0x7e,0x14,0x6f,0x88,0x2b,0x4f
};
#define GW_ICU_PUBLIC_KEY GW_ICU_PUBLIC_KEY_LN
#endif // GW_ROLE_LANE_NODE

#ifdef GW_ROLE_ICU
// ---- ICU side: the ICU's own private key. ------------------------
// SOP 3.2 step 2: one static keypair per ICU, not rotated. NOT compiled
// into any lane-node image.
static const uint8_t GW_ICU_PRIVATE_KEY[GW_X25519_LEN] = {
    // PLACEHOLDER -- REPLACE BEFORE DEPLOYMENT
    0x5d,0xab,0x08,0x7e,0x62,0x4a,0x8a,0x4b,0x79,0xe1,0x7f,0x8b,0x83,0x80,0x0e,0xe6,
    0x6f,0x3b,0xb1,0x29,0x26,0x18,0xb6,0xfd,0x1c,0x2f,0x8b,0x27,0xff,0x88,0xe0,0xeb
};

// ---- RDU public keys, pre-provisioned at the ICU (see GW_CERT_ON_AIR).
// Index = (lane_id - 1) * 2 + (node_id - 1).
//
// NOTE: unlike v3's GW_NODE_KEYS, these are PUBLIC keys. A dumped ICU
// image leaks nothing that lets an attacker forge a node. That is the
// whole point of the change.
static const uint8_t GW_RDU_PUBLIC_KEYS[6][GW_X25519_LEN] = {
    // L1N1 -- PLACEHOLDER, REPLACE BEFORE DEPLOYMENT
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6a},
    // L1N2 -- PLACEHOLDER
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6b},
    // L2N1 -- PLACEHOLDER
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6c},
    // L2N2 -- PLACEHOLDER
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6d},
    // L3N1 -- PLACEHOLDER
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6e},
    // L3N2 -- PLACEHOLDER
    {0x85,0x20,0xf0,0x09,0x89,0x30,0xa7,0x54,0x74,0x8b,0x7d,0xdc,0xb4,0x3e,0xf7,0x5a,
     0x0d,0xbf,0x3a,0x0d,0x26,0x38,0x1a,0xf4,0xeb,0xa4,0xa9,0x8e,0xaa,0x9b,0x4e,0x6f},
};
#endif // GW_ROLE_ICU
#endif // !GW_USE_PROVISIONED_KEYS

/***********************************************************************
 * SOP 3.5 -- REVOCATION
 *
 * "a revoked RDU's public key simply fails at the session-establishment
 * step instead of ever producing a report the ICU has to evaluate."
 *
 * Here, session establishment IS key derivation, so revocation is
 * checked in gwIcuGetSession() before any ECDH is performed. A revoked
 * node never gets a session, so its frames can never produce a valid
 * tag, so validateLane() never sees them.
 *
 * Bitmask indexed the same way as GW_RDU_PUBLIC_KEYS. Bit set = revoked.
 * SOP 3.5 calls for this to be database-backed at the ICU; until the
 * backend exists this is the local expression of that decision, and it
 * is the hook the backend will write into.
 ***********************************************************************/
static uint8_t gwRevokedMask = 0x00;   // no node revoked by default

static inline void gwRevokeNode(uint8_t lane, uint8_t node);
static inline bool gwIsRevoked(uint8_t lane, uint8_t node);

// ==================================================================
// INDEXING
// ==================================================================
static inline int gwKeyIndex(uint8_t lane, uint8_t node) {
    if (lane < 1 || lane > 3 || node < 1 || node > 2) return -1;
    return (lane - 1) * 2 + (node - 1);
}

static inline bool gwIsRevoked(uint8_t lane, uint8_t node) {
    int k = gwKeyIndex(lane, node);
    if (k < 0) return true;                      // unknown node: treat as revoked
    return (gwRevokedMask >> k) & 0x01;
}

static inline void gwRevokeNode(uint8_t lane, uint8_t node) {
    int k = gwKeyIndex(lane, node);
    if (k >= 0) gwRevokedMask |= (uint8_t)(1u << k);
}

static inline void gwUnrevokeNode(uint8_t lane, uint8_t node) {
    int k = gwKeyIndex(lane, node);
    if (k >= 0) gwRevokedMask &= (uint8_t)~(1u << k);
}

// ==================================================================
// X25519 ECDH  (SOP 3.2)
//
// mbedtls_ecdh_compute_shared() on MBEDTLS_ECP_DP_CURVE25519. Both
// sides compute the same secret from their own private key and the
// other's public key; nothing is ever transmitted.
//
// Byte order: RFC 7748 specifies little-endian for X25519 wire format;
// mbedtls_mpi is big-endian internally. We reverse on the way in and
// on the way out so what is stored in the key arrays above and what
// goes on the wire is standard little-endian X25519.
// ==================================================================
/***********************************************************************
 * gwX25519 -- scalar multiplication on Curve25519
 *
 * PUBLIC API ONLY. An earlier revision reached into mbedtls_ecp_point's
 * members directly (Q.X, Q.Z). That compiled against mbedTLS 2.x but
 * fails on 3.x, which renamed every public struct member to private_*
 * precisely to stop callers doing this. Do NOT "fix" a future breakage
 * here by defining MBEDTLS_ALLOW_PRIVATE_ACCESS -- that just re-arms the
 * same trap for the next mbedTLS bump. Use the accessor functions.
 *
 * BYTE ORDER. X25519 is little-endian end to end (RFC 7748), and the
 * functions used here are the little-endian variants:
 *   mbedtls_ecp_point_read_binary() takes the Montgomery u-coordinate
 *     as 32 LE bytes and sets Z internally.
 *   mbedtls_mpi_read_binary_le() / _write_binary_le() handle the scalar
 *     and the shared secret.
 * So the key arrays, the wire format and mbedTLS all agree and there is
 * no byte reversal anywhere. The manual reversal in the previous
 * revision existed only because it used the big-endian read_binary().
 *
 * CLAMPING. RFC 7748 requires the scalar to have bits 0,1,2 cleared and
 * bit 254 set. mbedtls_ecp_check_privkey() ENFORCES this and returns
 * MBEDTLS_ERR_ECP_INVALID_KEY otherwise, so an unclamped key fails at
 * runtime, not at provisioning. Clamping here is defensive: it is
 * idempotent (clamping an already-clamped key changes nothing) and
 * X25519(clamp(k),P) == X25519(k,P) by definition, so it cannot change
 * a result -- it only removes a dependency on whatever the key
 * generator happened to emit.
 ***********************************************************************/
/*
 * RNG for mbedtls_ecp_mul's coordinate blinding.
 *
 * The first cut passed f_rng = NULL. mbedTLS 2.x documented that as "no
 * randomization needed"; parts of 3.x and the ESP32 hardware-accelerated
 * ECP port are stricter and can reject it outright. Passing a real RNG
 * removes that as a variable AND is better practice regardless -- blinding
 * costs microseconds and defends the scalar against timing/power analysis.
 *
 * esp_random() is the hardware TRNG. It is properly seeded once the RF
 * subsystem is up, which it is here because LoRa is initialised first.
 */
static int gwRng(void *ctx, unsigned char *buf, size_t len) {
    (void)ctx;
    esp_fill_random(buf, len);
    return 0;
}

static bool gwX25519(const uint8_t privLE[32], const uint8_t pubLE[32],
                     uint8_t sharedLE[32]) {
    mbedtls_ecp_group grp;
    mbedtls_mpi d, z;
    mbedtls_ecp_point Q;
    uint8_t clamped[32];
    bool ok = false;
    int rc = 0;

    mbedtls_ecp_group_init(&grp);
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);
    mbedtls_ecp_point_init(&Q);

    rc = mbedtls_ecp_group_load(&grp, MBEDTLS_ECP_DP_CURVE25519);
    if (rc != 0) GW_FAIL("ecp_group_load(CURVE25519)", rc);

    memcpy(clamped, privLE, 32);
    clamped[0]  &= 248;
    clamped[31] &= 127;
    clamped[31] |= 64;

    rc = mbedtls_mpi_read_binary_le(&d, clamped, 32);
    if (rc != 0) GW_FAIL("mpi_read_binary_le(scalar)", rc);

    rc = mbedtls_ecp_point_read_binary(&grp, &Q, pubLE, 32);
    if (rc != 0) GW_FAIL("ecp_point_read_binary(peer_pub)", rc);

    rc = mbedtls_ecdh_compute_shared(&grp, &z, &Q, &d, gwRng, NULL);
    if (rc != 0) GW_FAIL("ecdh_compute_shared", rc);

    rc = mbedtls_mpi_write_binary_le(&z, sharedLE, 32);
    if (rc != 0) GW_FAIL("mpi_write_binary_le(shared)", rc);

    ok = true;

done:
    // Wipe the clamped scalar copy. mbedtls_*_free zeroises its own buffers.
    memset(clamped, 0, sizeof(clamped));
    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&z);
    mbedtls_mpi_free(&d);
    mbedtls_ecp_group_free(&grp);
    if (!ok) memset(sharedLE, 0, 32);
    return ok;
}

// Constant-time compare. An early-exit memcmp leaks, via timing, how
// many leading tag bytes were correct, which lets an attacker forge a
// tag one byte at a time -- roughly 2048 attempts instead of 2^64.
// This property was correct in the v3 CMAC path and is preserved here.
// DO NOT "optimise" this back into a memcmp.
static bool gwConstTimeEqual(const uint8_t *a, const uint8_t *b, size_t n) {
    uint8_t diff = 0;
    for (size_t i = 0; i < n; i++) diff |= (uint8_t)(a[i] ^ b[i]);
    return diff == 0;
}

/***********************************************************************
 * gwCryptoSelfTest -- RFC 7748 section 6.1 known-answer test.
 *
 * Runs the published Alice/Bob X25519 exchange through the exact same
 * code path the firmware uses. This separates the two failure modes that
 * otherwise look identical:
 *
 *   FAIL  -> mbedTLS itself cannot do Curve25519 in this build. Nothing
 *            about your provisioned keys matters yet. Check
 *            CONFIG_MBEDTLS_ECP_DP_CURVE25519_ENABLED.
 *   PASS  -> the primitive works; a derive failure after this is about
 *            the key material or HKDF, not the curve.
 *
 * Costs one scalar multiplication at boot. Worth it.
 ***********************************************************************/
static bool gwCryptoSelfTest(void) {
    static const uint8_t alice_priv[32] = {
        0x77,0x07,0x6d,0x0a,0x73,0x18,0xa5,0x7d,0x3c,0x16,0xc1,0x72,0x51,0xb2,0x66,0x45,
        0xdf,0x4c,0x2f,0x87,0xeb,0xc0,0x99,0x2a,0xb1,0x77,0xfb,0xa5,0x1d,0xb9,0x2c,0x2a };
    static const uint8_t bob_pub[32] = {
        0xde,0x9e,0xdb,0x7d,0x7b,0x7d,0xc1,0xb4,0xd3,0x5b,0x61,0xc2,0xec,0xe4,0x35,0x37,
        0x3f,0x83,0x43,0xc8,0x5b,0x78,0x67,0x4d,0xad,0xfc,0x7e,0x14,0x6f,0x88,0x2b,0x4f };
    static const uint8_t expect[32] = {
        0x4a,0x5d,0x9d,0x5b,0xa4,0xce,0x2d,0xe1,0x72,0x8e,0x3b,0xf4,0x80,0x35,0x0f,0x25,
        0xe0,0x7e,0x21,0xc9,0x47,0xd1,0x9e,0x33,0x76,0xf0,0x9b,0x3c,0x1e,0x16,0x17,0x42 };

    uint8_t got[32];
    gwClearErr();
    if (!gwX25519(alice_priv, bob_pub, got)) {
        Serial.printf("[SELFTEST] X25519 FAILED at %s rc=-0x%04X (%s)\n",
                      gwLastStep, (unsigned)(-gwLastErr), gwErrHint(gwLastErr));
        return false;
    }
    bool match = gwConstTimeEqual(got, expect, 32);
    memset(got, 0, sizeof(got));
    Serial.printf("[SELFTEST] X25519 RFC7748 vector: %s\n", match ? "PASS" : "MISMATCH");
    if (!match) {
        Serial.println("[SELFTEST] mbedTLS computed a wrong result - byte order or curve config");
    }
    return match;
}

// Provisioning helper: derive the public key for a given private key by
// scalar-multiplying the curve basepoint. Call this ONCE at
// provisioning to fill in the public-key tables above. Not used at
// runtime -- and note it prints nothing; the caller decides what to do
// with the result, and the result is a PUBLIC key.
static bool gwDerivePublicKey(const uint8_t privLE[32], uint8_t pubLE[32]) {
    static const uint8_t basepoint[32] = { 9 };   // rest zero
    return gwX25519(privLE, basepoint, pubLE);
}

// ==================================================================
// SESSION KEY DERIVATION  (SOP 3.2)
//
//   Shared_Secret = X25519_ECDH(own_private, peer_public)
//   Session_Key   = HKDF-SHA256(
//                      ikm  = Shared_Secret,
//                      salt = RDU_ID || session_epoch,
//                      info = "rdu-icu-session-v1")
//
// RDU_ID here is the 4-byte tuple {intersection_id, lane_id, node_id}
// already carried in every FrameHeader, so it is exactly the identity
// the ICU routes on -- no second naming scheme.
// ==================================================================
static bool gwDeriveSessionKey(const uint8_t peerPubLE[32],
                               const uint8_t ownPrivLE[32],
                               uint16_t intersectionId,
                               uint8_t lane, uint8_t node,
                               uint32_t sessionEpoch,
                               uint8_t sessionKeyOut[GW_SESSION_KEY_LEN]) {
    uint8_t shared[32];
    uint8_t salt[8];
    bool ok = false;

    if (!gwX25519(ownPrivLE, peerPubLE, shared)) return false;   // gwLastErr set

    // salt = RDU_ID (4B) || session_epoch (4B), little-endian, fixed width
    salt[0] = (uint8_t)(intersectionId & 0xFF);
    salt[1] = (uint8_t)(intersectionId >> 8);
    salt[2] = lane;
    salt[3] = node;
    salt[4] = (uint8_t)(sessionEpoch & 0xFF);
    salt[5] = (uint8_t)((sessionEpoch >> 8) & 0xFF);
    salt[6] = (uint8_t)((sessionEpoch >> 16) & 0xFF);
    salt[7] = (uint8_t)((sessionEpoch >> 24) & 0xFF);

    {
        const mbedtls_md_info_t *md =
            mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        if (md == NULL) {
            gwLastStep = "md_info_from_type(SHA256)";
            gwLastErr  = -1;
        } else {
            int rc = mbedtls_hkdf(md,
                                  salt, sizeof(salt),
                                  shared, sizeof(shared),
                                  (const uint8_t *)GW_HKDF_INFO,
                                  strlen(GW_HKDF_INFO),
                                  sessionKeyOut, GW_SESSION_KEY_LEN);
            if (rc != 0) { gwLastStep = "mbedtls_hkdf"; gwLastErr = rc; }
            ok = (rc == 0);
        }
    }

    memset(shared, 0, sizeof(shared));
    memset(salt, 0, sizeof(salt));
    if (!ok) memset(sessionKeyOut, 0, GW_SESSION_KEY_LEN);
    return ok;
}

// ==================================================================
// PER-PACKET TAG  (SOP 3.3)
//
// tag = HMAC-SHA256(Session_Key, frame_without_tag)[0 .. GW_TAG_LEN-1]
//
// The MAC covers the ENTIRE frame including the header. Deliberate: if
// the header were outside the MAC, an attacker could take a genuine
// frame from L1N1 and relabel it L2N2, defeating the two-node
// corroboration rule using nothing but real traffic. It also binds
// session_epoch, so a frame cannot be re-attributed to a different
// epoch to dodge rotation.
// ==================================================================
static bool gwComputeTag(const uint8_t sessionKey[GW_SESSION_KEY_LEN],
                         const uint8_t *frame, size_t frameLen,
                         uint8_t *tagOut) {
    if (frame == NULL || tagOut == NULL) return false;
    if (frameLen <= GW_TAG_LEN) return false;      // length check before any read

    const mbedtls_md_info_t *md = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md == NULL) return false;

    uint8_t full[32];
    int rc = mbedtls_md_hmac(md,
                             sessionKey, GW_SESSION_KEY_LEN,
                             frame, frameLen - GW_TAG_LEN,
                             full);
    if (rc != 0) { memset(full, 0, sizeof(full)); return false; }

    memcpy(tagOut, full, GW_TAG_LEN);
    memset(full, 0, sizeof(full));
    return true;
}

// ==================================================================
// RDU SIDE  --  session state and signing
// ==================================================================

// Reasons a verify can fail, so the ICU can count each one separately
// (no silent failure paths).
enum GwVerifyResult : uint8_t {
    GW_OK              = 0,
    GW_ERR_LENGTH      = 1,
    GW_ERR_UNKNOWN_NODE= 2,
    GW_ERR_REVOKED     = 3,
    GW_ERR_NO_SESSION  = 4,   // key derivation failed
    GW_ERR_EPOCH       = 5,   // epoch outside the acceptance window
    GW_ERR_TAG         = 6
};

#ifdef GW_ROLE_LANE_NODE
class GwRduSession {
public:
    // SOP 3.4. session_epoch must be monotonic across reboots for the
    // same reason the packet counter is: if it restarts at zero, an old
    // session key becomes valid again and rotation buys nothing. Held in
    // NVS, one write per rotation (60 s) -- ~525k writes/year, well
    // inside ESP32 NVS wear-levelling for a single u32 entry.
    void begin(uint16_t intersectionId, uint8_t lane, uint8_t node,
               const char *ns = "gw-sess") {
        _iid = intersectionId; _lane = lane; _node = node;

        _prefs.begin(ns, false);
        _epoch = _prefs.getULong("epoch", 0) + 1;
        _prefs.putULong("epoch", _epoch);

        _lastRekeyMs = millis();

        // v4.5: state key provenance at every boot. A device running
        // published test-vector keys must never be mistaken for a secured
        // one by anybody reading a serial log.
#if GW_USE_PROVISIONED_KEYS
        Serial.printf("[KEYS] provisioned set '%s'\n", GW_KEY_SET_ID);
#else
        Serial.println("[KEYS] ****************************************************");
        Serial.println("[KEYS] *** RFC 7748 PLACEHOLDER KEYS - NO SECURITY      ***");
        Serial.println("[KEYS] *** IF-2 tags are forgeable by anyone. Bench only.***");
        Serial.println("[KEYS] *** Set GW_USE_PROVISIONED_KEYS=1 to secure.     ***");
        Serial.println("[KEYS] ****************************************************");
#endif

        // Prove the primitive works before blaming the keys.
        _selfTestOk = gwCryptoSelfTest();

        gwClearErr();
        _valid = deriveCurrent();

        // Logs the epoch only. Never the key, never the shared secret,
        // never a fingerprint of either -- a fingerprint is a free
        // confirmation oracle for anyone brute-forcing offline.
        Serial.printf("[SESS] epoch=%lu derive=%s\n",
                      (unsigned long)_epoch, _valid ? "OK" : "FAILED");
        if (!_valid) {
            gwFailStep = gwLastStep;          // latch for later reporting
            gwFailErr  = gwLastErr;
            gwFailSelfTestOk = _selfTestOk;
            Serial.printf("[SESS] FAILED at step '%s' rc=-0x%04X\n",
                          gwLastStep, (unsigned)(-gwLastErr));
            Serial.printf("[SESS] hint: %s\n", gwErrHint(gwLastErr));
            if (!_selfTestOk) {
                Serial.println("[SESS] self-test also failed -> this is an mbedTLS build");
                Serial.println("[SESS] problem, NOT your provisioned keys. Enable");
                Serial.println("[SESS] CONFIG_MBEDTLS_ECP_DP_CURVE25519_ENABLED.");
            } else {
                Serial.println("[SESS] self-test PASSED -> curve works; suspect key material");
                Serial.println("[SESS] (GW_ICU_PUBLIC_KEY / GW_RDU_PRIVATE_KEY) or HKDF config.");
            }
            Serial.println("[SESS] FATAL: no session key - frames cannot be authenticated");
        }
    }

    // Call once per loop from transportTask. Rotation is checked, not
    // performed inline in the transmit path, so an ECDH never lands
    // inside a scheduled transmit window.
    void tick() {
        if (!_valid) return;
        if (millis() - _lastRekeyMs < GW_REKEY_INTERVAL_MS) return;

        _lastRekeyMs = millis();
        _epoch++;
        _prefs.putULong("epoch", _epoch);
        if (deriveCurrent()) {
            Serial.printf("[SESS] re-key -> epoch=%lu\n", (unsigned long)_epoch);
        } else {
            _valid = false;
            Serial.println("[SESS] FATAL: re-key derivation failed");
        }
    }

    uint32_t epoch() const { return _epoch; }
    bool     valid() const { return _valid; }
    const uint8_t *key() const { return _key; }

private:
    bool deriveCurrent() {
        return gwDeriveSessionKey(GW_ICU_PUBLIC_KEY, GW_RDU_PRIVATE_KEY,
                                  _iid, _lane, _node, _epoch, _key);
    }

    Preferences _prefs;
    bool     _selfTestOk = false;
    uint8_t  _key[GW_SESSION_KEY_LEN] = {0};
    uint32_t _epoch = 0;
    unsigned long _lastRekeyMs = 0;
    uint16_t _iid = 0;
    uint8_t  _lane = 0, _node = 0;
    bool     _valid = false;
};

extern GwRduSession gwSession;   // defined in LaneNode.ino

// Reprint the latched failure. Called from the rate-limited abort path so
// the diagnosis is available without capturing the boot log.
static inline void gwPrintSessionFailure(void) {
    Serial.printf("[SESS] latched failure: step='%s' rc=-0x%04X (%s)\n",
                  gwFailStep, (unsigned)(-gwFailErr), gwErrHint(gwFailErr));
    Serial.printf("[SESS] X25519 self-test at boot: %s\n",
                  gwFailSelfTestOk ? "PASSED (curve OK - suspect key material/HKDF)"
                                   : "FAILED (mbedTLS build problem, not your keys)");
}

// Transmitter side. Signature unchanged from v3 so radiateToICU() does
// not move. Stamps session_epoch into the header, then tags -- in that
// order, because the tag covers the epoch.
static inline bool gwSignFrame(uint8_t lane, uint8_t node,
                               uint8_t *frame, size_t frameLen) {
    (void)lane; (void)node;                 // identity comes from gwSession
    if (frame == NULL || frameLen <= GW_TAG_LEN) return false;
    if (frameLen < sizeof(FrameHeader)) return false;
    if (!gwSession.valid()) return false;

    uint32_t ep = gwSession.epoch();
    memcpy(frame + GW_EPOCH_OFFSET, &ep, sizeof(ep));

    return gwComputeTag(gwSession.key(), frame, frameLen,
                        frame + frameLen - GW_TAG_LEN);
}
#endif // GW_ROLE_LANE_NODE

// ==================================================================
// ICU SIDE  --  per-node session cache with epoch overlap
//
// SOP 5.3: "both sides track a brief overlap window (accept either the
// current or immediately-prior epoch's tag) to tolerate in-flight
// packets during the cutover."
//
// The ICU is receive-only, so it does not run its own rotation clock --
// it follows whatever epoch the RDU asserts in the header, and keeps
// the previous epoch's key alive so a frame that was already on the air
// when the RDU rotated is not dropped.
// ==================================================================
#ifdef GW_ROLE_ICU
class GwIcuSessionCache {
public:
    void begin() {
        for (int i = 0; i < 6; i++) { _s[i].have = false; _s[i].havePrev = false; }
    }

    // Returns GW_OK and fills keyOut if a usable key exists for this
    // (lane,node,epoch). Derives on demand and slides the overlap window
    // forward when the RDU advances its epoch.
    GwVerifyResult keyFor(uint8_t lane, uint8_t node, uint32_t epoch,
                          const uint8_t **keyOut) {
        int k = gwKeyIndex(lane, node);
        if (k < 0) return GW_ERR_UNKNOWN_NODE;

        // SOP 3.5: revocation gates session establishment itself. A
        // revoked node never gets a key, so it can never produce a
        // verifiable frame, so it never reaches the report decision.
        if (gwIsRevoked(lane, node)) return GW_ERR_REVOKED;

        Slot &s = _s[k];

        if (s.have && s.epoch == epoch)          { *keyOut = s.key;     return GW_OK; }
        if (s.havePrev && s.prevEpoch == epoch)  {
            // SOP 5.3 overlap window actually used: a frame was in flight
            // when the node rotated. Counted so it is visible whether this
            // path is exercised at all, and how often.
            gwOverlapHits++;
            *keyOut = s.prevKey; return GW_OK;
        }

        // Epoch must move FORWARD. An older epoch than the one we last
        // accepted is a replay of a retired session, not a cutover.
        if (s.have && epoch < s.epoch) return GW_ERR_EPOCH;

        uint8_t fresh[GW_SESSION_KEY_LEN];
        if (!gwDeriveSessionKey(GW_RDU_PUBLIC_KEYS[k], GW_ICU_PRIVATE_KEY,
                                _iid, lane, node, epoch, fresh)) {
            memset(fresh, 0, sizeof(fresh));
            return GW_ERR_NO_SESSION;
        }

        if (s.have) {                             // slide the overlap window
            memcpy(s.prevKey, s.key, GW_SESSION_KEY_LEN);
            s.prevEpoch = s.epoch;
            s.havePrev  = true;
            // v4.5: SOP 3.4/5.3 rotations were previously invisible at the
            // ICU. This is the only place that proves re-keying actually
            // happened and that the overlap window slid rather than the
            // node simply restarting.
            Serial.printf("[SESS] L%uN%u epoch %lu -> %lu (prior epoch kept for overlap)\n",
                          (unsigned)lane, (unsigned)node,
                          (unsigned long)s.prevEpoch, (unsigned long)epoch);
        } else {
            Serial.printf("[SESS] L%uN%u first session, epoch=%lu\n",
                          (unsigned)lane, (unsigned)node, (unsigned long)epoch);
        }
        memcpy(s.key, fresh, GW_SESSION_KEY_LEN);
        s.epoch = epoch;
        s.have  = true;
        memset(fresh, 0, sizeof(fresh));

        *keyOut = s.key;
        return GW_OK;
    }

    void setIntersection(uint16_t iid) { _iid = iid; }

private:
    struct Slot {
        uint8_t  key[GW_SESSION_KEY_LEN];
        uint8_t  prevKey[GW_SESSION_KEY_LEN];
        uint32_t epoch = 0, prevEpoch = 0;
        bool have = false, havePrev = false;
    };
    Slot _s[6];
    uint16_t _iid = 0;
};

extern GwIcuSessionCache gwIcuSessions;   // defined in ICU.ino

// Receiver side. Returns a specific GwVerifyResult so every rejection
// path can be counted and logged separately at the call site.
//
// All length validation happens before any read of frame contents --
// this input is attacker-controlled.
static inline GwVerifyResult gwVerifyFrameEx(uint8_t lane, uint8_t node,
                                             uint32_t epoch,
                                             const uint8_t *frame,
                                             size_t frameLen) {
    if (frame == NULL) return GW_ERR_LENGTH;
    if (frameLen < sizeof(FrameHeader) + GW_TAG_LEN) return GW_ERR_LENGTH;
    if (frameLen > GW_MAX_FRAME_LEN) return GW_ERR_LENGTH;

    const uint8_t *key = NULL;
    GwVerifyResult r = gwIcuSessions.keyFor(lane, node, epoch, &key);
    if (r != GW_OK) return r;

    uint8_t expected[GW_TAG_LEN];
    if (!gwComputeTag(key, frame, frameLen, expected)) return GW_ERR_TAG;

    bool eq = gwConstTimeEqual(expected, frame + frameLen - GW_TAG_LEN, GW_TAG_LEN);
    memset(expected, 0, sizeof(expected));
    return eq ? GW_OK : GW_ERR_TAG;
}

#endif // GW_ROLE_ICU

static inline const char *gwVerifyResultName(GwVerifyResult r) {
    switch (r) {
        case GW_OK:               return "OK";
        case GW_ERR_LENGTH:       return "LENGTH";
        case GW_ERR_UNKNOWN_NODE: return "UNKNOWN_NODE";
        case GW_ERR_REVOKED:      return "REVOKED";
        case GW_ERR_NO_SESSION:   return "NO_SESSION";
        case GW_ERR_EPOCH:        return "EPOCH";
        case GW_ERR_TAG:          return "BAD_TAG";
        default:                  return "?";
    }
}

/***********************************************************************
 * MONOTONIC PACKET COUNTER  --  UNCHANGED FROM v3
 *
 * counter = (epoch << 16) | seq. Monotonic across power cycles; only
 * the boot epoch is written to flash, once per boot. This is separate
 * from session_epoch above and serves a different purpose: session_epoch
 * selects the key, counter defeats replay within a key.
 *
 * Kept verbatim -- it was verified across five TC-LN-001 runs and the
 * v3.2 transmit-time stamping fix depends on its behaviour.
 ***********************************************************************/
class GwCounter {
public:
    void begin(const char *nvsNamespace = "gw-ctr") {
        _prefs.begin(nvsNamespace, false);
        _epoch = _prefs.getUShort("epoch", 0) + 1;
        _prefs.putUShort("epoch", _epoch);
        _seq = 0;
        Serial.printf("[CTR] boot epoch=%u (counter base=%lu)\n",
                      (unsigned)_epoch, (unsigned long)((uint32_t)_epoch << 16));
    }

    uint32_t next() {
        if (_seq >= 0xFFFE) {          // wrap guard: burn a new epoch
            _epoch++;
            _prefs.putUShort("epoch", _epoch);
            _seq = 0;
            Serial.printf("[CTR] seq wrapped, new epoch=%u\n", (unsigned)_epoch);
        }
        return ((uint32_t)_epoch << 16) | (_seq++);
    }

private:
    Preferences _prefs;
    uint16_t _epoch = 0;
    uint16_t _seq   = 0;
};

#endif // GREENWAVE_CRYPTO_H
