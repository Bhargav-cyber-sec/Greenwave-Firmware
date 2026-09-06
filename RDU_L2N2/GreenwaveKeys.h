/***********************************************************************
 * GreenWave IF-2 -- UNPROVISIONED KEY MATERIAL   ** PLACEHOLDER **
 *
 * Device : RDU L2N2 (intersection 1)
 * ICU public-key table index : 3
 *
 * THIS FILE CONTAINS NO PRIVATE KEY AND MUST BE REPLACED BEFORE THIS
 * NODE CAN AUTHENTICATE.
 *
 * ------------------------------------------------------------------
 * HANDOVER NOTE -- EMBEDDED INTERN -> SECURITY INTERN
 * ------------------------------------------------------------------
 * The ICU already holds the PUBLIC key for this node at index 3 of
 * GW_RDU_PUBLIC_KEYS in ICU/GreenwaveKeys.h, from key set 7e200d93:
 *
 *     30 71 52 ad 2e 29 b5 71 59 d3 6e a8 3b f4 5e f7
 *     37 6c 91 28 1d 51 4f 36 5d 12 1c 78 c9 b9 80 1b
 *
 * The matching PRIVATE key was never present in the firmware tree and
 * is not reproducible from the public half. It must come from whatever
 * ran gw_provision_keys.py on 2026-08-05, or the whole set must be
 * regenerated.
 *
 * REGENERATE THE WHOLE SET, NOT FOUR KEYS. The header of every existing
 * key file records that key set 7e200d93 was published in a chat log and
 * a git history and "must be treated as public". Partial regeneration is
 * also impossible in principle: the ICU's public-key table and the six
 * private keys are ONE SET, so the ICU is reflashed either way.
 *
 * WHATEVER IS GENERATED, VERIFY IT BEFORE FLASHING. Derive each private
 * key's public half and check it against the ICU table at index
 * (lane-1)*2 + (node-1). An index off by one produces 100% BAD TAG from
 * one node and nothing else in any log -- it is invisible with one node
 * and fatal with two.
 * ------------------------------------------------------------------
 *
 * This file deliberately FAILS TO COMPILE. A placeholder that builds is
 * a placeholder that ships. To build this folder for a bench test that
 * does not need authentication, define GW_BENCH_UNPROVISIONED -- the
 * node will then boot, print an unmissable warning banner, and be
 * rejected by the ICU on every frame, which is the correct behaviour.
 ***********************************************************************/

#ifndef GREENWAVE_KEYS_H
#define GREENWAVE_KEYS_H

#define GW_KEY_SET_ID "UNPROVISIONED"
#define GW_KEY_INTERSECTION 1

#ifndef GW_ROLE_LANE_NODE
#error "GreenwaveKeys.h for a lane node included in a non-lane-node build"
#endif

#define GW_KEY_LANE_ID 2
#define GW_KEY_NODE_ID 2

#ifndef GW_BENCH_UNPROVISIONED
#error "RDU_L2N2/GreenwaveKeys.h is a PLACEHOLDER. Provision this node before building. See the handover note at the top of this file."
#endif

/* Placeholder. Authenticates nothing. */
static const uint8_t GW_RDU_PRIVATE_KEY[32] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};

/* The ICU's PUBLIC key from set 7e200d93. Safe in this image, and correct
 * only for as long as the ICU still runs that set. */
static const uint8_t GW_ICU_PUBLIC_KEY_LN[32] = {
    0xe2,0xbe,0x53,0x29,0x6a,0xf8,0x94,0xd8,0x6f,0x61,0x43,0x04,0x41,0x37,0x18,0x51,
    0xc9,0x43,0xb9,0xf1,0x49,0xe4,0x0f,0x98,0xa9,0xcb,0xc9,0x66,0x9b,0x93,0xc1,0x03
};
#define GW_ICU_PUBLIC_KEY GW_ICU_PUBLIC_KEY_LN

#endif /* GREENWAVE_KEYS_H */
