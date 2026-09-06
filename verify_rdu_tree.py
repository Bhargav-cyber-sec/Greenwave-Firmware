#!/usr/bin/env python3
"""
Pre-flash invariant checker for the six RDU folders.

Run this after EVERY firmware change and before EVERY flashing session.
It enforces the one rule that keeps six folders maintainable:

    everything is identical except GreenwaveKeys.h

and the one rule that keeps them safe:

    GreenwaveKeys.h must DIFFER in all six.

Usage:  python3 verify_rdu_tree.py [Firmware/]
Exit 0 = safe to flash. Exit 1 = do not flash.
"""

import hashlib
import os
import re
import sys

NODES = [(l, n) for l in (1, 2, 3) for n in (1, 2)]

IDENTICAL = [
    "GreenwaveTypes.h", "GreenwaveCrypto.h",
    "greenwave_siren_dscnn_int8.tflite",
    "gw_model.cpp", "gw_model.h", "gw_model_data.h",
    "gw_frontend.c", "gw_frontend.h", "gw_frontend_tables.h",
    "gw_fft.h", "gw_fft_kiss.c",
    "kiss_fft.c", "kiss_fft.h", "kiss_fftr.c", "kiss_fftr.h",
    "kiss_fft_log.h", "_kiss_fft_guts.h",
]

fails = []
warns = []


def md5(p):
    return hashlib.md5(open(p, "rb").read()).hexdigest()


def main(root):
    dirs = {(l, n): os.path.join(root, f"RDU_L{l}N{n}") for l, n in NODES}

    for k, d in dirs.items():
        if not os.path.isdir(d):
            fails.append(f"missing folder {d}")
    if fails:
        return

    # --- 1. the .ino must be byte-identical in all six ------------------
    inos = {}
    for (l, n), d in dirs.items():
        p = os.path.join(d, f"RDU_L{l}N{n}.ino")
        if not os.path.isfile(p):
            fails.append(f"missing {p} (Arduino requires folder name == .ino name)")
            continue
        inos[(l, n)] = md5(p)
    if len(set(inos.values())) > 1:
        fails.append("RDU_*.ino DIFFER across folders -- identity must live only "
                     "in GreenwaveKeys.h")
        for k, v in inos.items():
            fails.append(f"    L{k[0]}N{k[1]}  {v}")
    elif inos:
        print(f"[OK] RDU_*.ino identical in all six   md5 {list(inos.values())[0]}")

    # --- 2. shared support files identical ------------------------------
    for f in IDENTICAL:
        h = {}
        for (l, n), d in dirs.items():
            p = os.path.join(d, f)
            if not os.path.isfile(p):
                fails.append(f"missing {p}")
                continue
            h[(l, n)] = md5(p)
        if len(set(h.values())) > 1:
            fails.append(f"{f} DIFFERS across folders")
    if not any(f.endswith("DIFFERS across folders") for f in fails):
        print(f"[OK] {len(IDENTICAL)} shared support files identical in all six")

    # --- 3. key files must all DIFFER -----------------------------------
    # This is the line that catches the dangerous mistake: the same folder
    # flashed twice, i.e. two boxes carrying one identity.
    keys = {}
    for (l, n), d in dirs.items():
        p = os.path.join(d, "GreenwaveKeys.h")
        if not os.path.isfile(p):
            fails.append(f"missing {p}")
            continue
        keys[(l, n)] = md5(p)
    if len(set(keys.values())) != len(keys):
        seen = {}
        for k, v in keys.items():
            seen.setdefault(v, []).append(k)
        for v, ks in seen.items():
            if len(ks) > 1:
                fails.append("IDENTICAL GreenwaveKeys.h in: " +
                             ", ".join(f"L{a}N{b}" for a, b in ks))
    else:
        print("[OK] GreenwaveKeys.h differs in all six")

    # --- 4. lane/node macros must match the folder name -----------------
    for (l, n), d in dirs.items():
        p = os.path.join(d, "GreenwaveKeys.h")
        if not os.path.isfile(p):
            continue
        t = open(p, encoding="utf-8", errors="replace").read()
        if f"#define GW_KEY_LANE_ID {l}" not in t:
            fails.append(f"L{l}N{n}: GW_KEY_LANE_ID does not say {l}")
        if f"#define GW_KEY_NODE_ID {n}" not in t:
            fails.append(f"L{l}N{n}: GW_KEY_NODE_ID does not say {n}")
    print("[OK] lane/node macros match folder names")

    # --- 5. provisioning state -----------------------------------------
    unprov = []
    for (l, n), d in dirs.items():
        t = open(os.path.join(d, "GreenwaveKeys.h"), encoding="utf-8",
                 errors="replace").read()
        if "UNPROVISIONED" in t or re.search(r"PRIVATE_KEY\[32\]\s*=\s*\{[\s0x,]*?\}",
                                             t) and t.count("0x00,") >= 30:
            unprov.append(f"L{l}N{n}")
    if unprov:
        warns.append("UNPROVISIONED (placeholder keys, cannot authenticate): "
                     + ", ".join(unprov))

    # --- 6. private keys must be distinct where they exist --------------
    privs = {}
    for (l, n), d in dirs.items():
        t = open(os.path.join(d, "GreenwaveKeys.h"), encoding="utf-8",
                 errors="replace").read()
        i = t.find("GW_RDU_PRIVATE_KEY")
        if i < 0:
            continue
        j = t.find("}", i)
        b = bytes(int(x, 16) for x in re.findall(r"0x[0-9a-fA-F]{2}", t[i:j]))
        if len(b) == 32 and any(b):
            privs[(l, n)] = b.hex()
    if len(set(privs.values())) != len(privs):
        fails.append("TWO NODES SHARE A PRIVATE KEY -- this is the "
                     "'same folder flashed twice' failure, in the source tree")
    elif privs:
        print(f"[OK] {len(privs)} real private key(s), all distinct")

    # --- 7. key/ICU-table pairing ---------------------------------------
    icu = os.path.join(os.path.dirname(root.rstrip("/")) or ".", "ICU",
                       "GreenwaveKeys.h")
    if not os.path.isfile(icu):
        icu = os.path.join(root, "..", "ICU", "GreenwaveKeys.h")
    if os.path.isfile(icu):
        try:
            from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey
            from cryptography.hazmat.primitives import serialization as s
        except ImportError:
            warns.append("cryptography not installed -- key/ICU pairing NOT checked. "
                         "pip install cryptography")
        else:
            t = open(icu, encoding="utf-8", errors="replace").read()
            i = t.index("GW_RDU_PUBLIC_KEYS")
            j = t.index("};", i)
            blob = bytes(int(x, 16) for x in re.findall(r"0x[0-9a-fA-F]{2}", t[i:j]))
            table = [blob[k * 32:(k + 1) * 32] for k in range(6)]
            for (l, n), hx in privs.items():
                idx = (l - 1) * 2 + (n - 1)
                pub = X25519PrivateKey.from_private_bytes(
                    bytes.fromhex(hx)).public_key().public_bytes(
                    s.Encoding.Raw, s.PublicFormat.Raw)
                if pub != table[idx]:
                    fails.append(f"L{l}N{n}: private key does NOT match ICU table "
                                 f"index {idx} -- this is the off-by-one that "
                                 f"produces 100% BAD TAG")
                else:
                    print(f"[OK] L{l}N{n} key pairs with ICU table index {idx}")
    else:
        warns.append("ICU/GreenwaveKeys.h not found -- key/ICU pairing NOT checked")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
    print()
    for w in warns:
        print("[WARN] " + w)
    for f in fails:
        print("[FAIL] " + f)
    if fails:
        print("\nDO NOT FLASH.")
        sys.exit(1)
    print("\nTree invariants hold." + ("  (see warnings above)" if warns else ""))
