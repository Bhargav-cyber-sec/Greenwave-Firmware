/***********************************************************************
 * GreenWave IF-2 v4 -- PROVISIONED KEY MATERIAL
 *
 * GENERATED FILE -- DO NOT EDIT BY HAND.
 * Generated : 2026-09-10 13:33:08Z
 * Generator : gw_provision_keys.py
 * Device    : RDU L1N1 (intersection 1)
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

#define GW_KEY_LANE_ID 1
#define GW_KEY_NODE_ID 1

/* This node's own private key. No other node's secret appears in this file,
 * and neither does the ICU's private key. That is the entire v4 change:
 * opening this enclosure compromises this node and nothing else. */
static const uint8_t GW_RDU_PRIVATE_KEY[32] = {
    0x78,0x4e,0x3c,0xbd,0xb5,0x26,0x85,0x3b,0x91,0xc4,0xfb,0x3c,0x44,0x84,0x4d,0xe1,
    0xe1,0x50,0x0c,0x59,0x22,0x06,0x79,0x44,0x27,0x0c,0x39,0xf9,0xde,0x4f,0x52,0x46
};

/* The ICU's PUBLIC key. Safe in this image. */
static const uint8_t GW_ICU_PUBLIC_KEY_LN[32] = {
    0xfb,0xd7,0x2c,0xbc,0xc5,0xe9,0x3e,0x53,0x7b,0xb7,0xdb,0x93,0x12,0xfd,0x60,0xf4,
    0x6a,0x6c,0x6d,0xe2,0x72,0xcc,0x37,0x1e,0x65,0x8c,0xf0,0x8f,0x8b,0xd6,0x2c,0x29
};
#define GW_ICU_PUBLIC_KEY GW_ICU_PUBLIC_KEY_LN

#endif /* GREENWAVE_KEYS_H */
