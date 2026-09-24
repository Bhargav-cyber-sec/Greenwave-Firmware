/***********************************************************************
 * GreenWave IF-2 v4 -- PROVISIONED KEY MATERIAL
 *
 * GENERATED FILE -- DO NOT EDIT BY HAND.
 * Generated : 2026-09-10 13:33:08Z
 * Generator : gw_provision_keys.py
 * Device    : RDU L3N2 (intersection 1)
 * Set ID    : 87554f73
 *
 * CONTAINS A PRIVATE KEY. Do not commit to version control.
 * Do not copy to any other device. File mode is 0600 by design.
 *
 * Include this file from GreenwaveCrypto.h, or paste its arrays over the
 * PLACEHOLDER blocks there. Whichever you choose, the placeholder values
 * shipped with the header MUST be gone before this device leaves the
 * bench -- they have been published in a chat log and a git history and
 * must be treated as public.
 ***********************************************************************/

#ifndef GREENWAVE_KEYS_H
#define GREENWAVE_KEYS_H

#define GW_KEY_SET_ID "87554f73"
#define GW_KEY_INTERSECTION 1

#ifndef GW_ROLE_LANE_NODE
#error "GreenwaveKeys.h for a lane node included in a non-lane-node build"
#endif

#define GW_KEY_LANE_ID 3
#define GW_KEY_NODE_ID 2

/* This node's own private key. No other node's secret appears in this file,
 * and neither does the ICU's private key. That is the entire v4 change:
 * opening this enclosure compromises this node and nothing else. */
static const uint8_t GW_RDU_PRIVATE_KEY[32] = {
    0xa0,0xbe,0x32,0xa8,0x48,0x95,0x9f,0x91,0xd7,0xbf,0x48,0xc6,0xd1,0xc6,0x75,0xc4,
    0xad,0xf5,0x2e,0xfb,0x0d,0x81,0xc7,0x83,0x88,0xef,0xb4,0xda,0x60,0x14,0xef,0x65
};

/* The ICU's PUBLIC key. Safe in this image. */
static const uint8_t GW_ICU_PUBLIC_KEY_LN[32] = {
    0xfb,0xd7,0x2c,0xbc,0xc5,0xe9,0x3e,0x53,0x7b,0xb7,0xdb,0x93,0x12,0xfd,0x60,0xf4,
    0x6a,0x6c,0x6d,0xe2,0x72,0xcc,0x37,0x1e,0x65,0x8c,0xf0,0x8f,0x8b,0xd6,0x2c,0x29
};
#define GW_ICU_PUBLIC_KEY GW_ICU_PUBLIC_KEY_LN

#endif /* GREENWAVE_KEYS_H */
