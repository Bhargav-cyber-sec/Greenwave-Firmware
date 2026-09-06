/***********************************************************************
 * GreenWave IF-2 v4 -- PROVISIONED KEY MATERIAL
 *
 * GENERATED FILE -- DO NOT EDIT BY HAND.
 * Generated : 2026-08-05 11:40:17Z
 * Generator : gw_provision_keys.py
 * Device    : RDU L1N2 (intersection 1)
 * Set ID    : 7e200d93
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

#define GW_KEY_SET_ID "7e200d93"
#define GW_KEY_INTERSECTION 1

#ifndef GW_ROLE_LANE_NODE
#error "GreenwaveKeys.h for a lane node included in a non-lane-node build"
#endif

#define GW_KEY_LANE_ID 1
#define GW_KEY_NODE_ID 2

/* This node's own private key. No other node's secret appears in this file,
 * and neither does the ICU's private key. That is the entire v4 change:
 * opening this enclosure compromises this node and nothing else. */
static const uint8_t GW_RDU_PRIVATE_KEY[32] = {
    0x00,0x90,0x4c,0xde,0xac,0xd9,0x2d,0xe5,0x5f,0x52,0x68,0xad,0xd0,0xe8,0x48,0x98,
    0xf5,0xc1,0xc1,0x4b,0x3f,0x4e,0x30,0x17,0x60,0x8d,0x3b,0xd3,0x05,0xf3,0xda,0x7b
};

/* The ICU's PUBLIC key. Safe in this image. */
static const uint8_t GW_ICU_PUBLIC_KEY_LN[32] = {
    0xe2,0xbe,0x53,0x29,0x6a,0xf8,0x94,0xd8,0x6f,0x61,0x43,0x04,0x41,0x37,0x18,0x51,
    0xc9,0x43,0xb9,0xf1,0x49,0xe4,0x0f,0x98,0xa9,0xcb,0xc9,0x66,0x9b,0x93,0xc1,0x03
};
#define GW_ICU_PUBLIC_KEY GW_ICU_PUBLIC_KEY_LN

#endif /* GREENWAVE_KEYS_H */
