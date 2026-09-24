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

import glob
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


# =====================================================================
# v7 CROSS-ROLE CHECKS
#
# Everything above verifies the six RDU folders against each other. The
# checks below verify things that span ROLES -- RDU vs ICU vs EVU -- and
# are the reason this file is extended rather than replaced by a new
# tool: the failure mode is identical (one copy edited, the others not)
# and a second script is a second thing to forget to run.
# =====================================================================

def _strip_comments(s):
    s = re.sub(r"/\*.*?\*/", "", s, flags=re.S)
    s = re.sub(r"//.*", "", s)
    return s


def _parse_struct(text, name):
    """Return [(type, field)] for a packed struct, or None.

    Compares the DECLARED LAYOUT, not the source text. Comments,
    whitespace and blank lines differ between the three copies today and
    are not layout; types, order and array extents are.
    """
    m = re.search(r"struct\s+__attribute__\(\(packed\)\)\s+" + name +
                  r"\s*\{(.*?)\}\s*;", text, re.S)
    if not m:
        return None
    fields = []
    for decl in _strip_comments(m.group(1)).split(";"):
        decl = " ".join(decl.split())
        if not decl:
            continue
        parts = decl.split()
        ftype = " ".join(parts[:-1])
        fname = parts[-1]
        fields.append((ftype, fname))
    return fields


# Sizes used only to report a byte offset in the error message. The
# check itself compares declarations, so an unknown type is not fatal.
_SZ = {"char": 1, "uint8_t": 1, "int8_t": 1, "uint16_t": 2, "int16_t": 2,
       "uint32_t": 4, "int32_t": 4, "uint64_t": 8, "int64_t": 8,
       "float": 4, "double": 8}


def _offsets(fields):
    off, out = 0, []
    for ftype, fname in fields:
        base = fname.split("[")[0]
        n = 1
        mm = re.search(r"\[(\d+)\]", fname)
        if mm:
            n = int(mm.group(1))
        sz = _SZ.get(ftype)
        out.append((off, ftype, base))
        off = None if (off is None or sz is None) else off + sz * n
    return out


def check_constants_used(root):
    """Every constant declared for the banner must actually be READ by it.

    Added after a real defect in the v7 Issue-3 patch: the heartbeat
    banner's literal 40000 was changed to %lu and the matching argument
    was never added. EXPECTED_ICU_HB_TIMEOUT_MS was defined, documented,
    cross-checked against the ICU by this very script -- and never read.
    The banner printed whatever happened to be in the register.

    That is worse than the literal it replaced: a wrong literal is at
    least wrong reproducibly. And every other check in this file passed,
    because they all verified the constant's VALUE and none verified that
    anything used it.

    Narrow on purpose. A general printf-arity parser was tried first and
    mis-located the format string whenever an argument contained a quoted
    ternary, producing ten false failures on correct code. A tool that
    cries wolf is worse than no tool: people learn to pass it with -x.
    So this checks the one class of constant that exists solely to be
    printed, where "defined but unread" is unambiguously a bug.
    """
    rdu = os.path.join(root, "RDU_L1N1", "RDU_L1N1.ino")
    if not os.path.isfile(rdu):
        warns.append("constants-used check skipped (RDU sketch missing)")
        return
    src = open(rdu, encoding="utf-8", errors="replace").read()
    # Strip comments: a constant mentioned only in prose is still unread.
    code = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
    code = re.sub(r"//.*", "", code)

    names = re.findall(r"#define\s+(EXPECTED_ICU_\w+)", code)
    if not names:
        fails.append("no EXPECTED_ICU_* constants found -- the boot banner is "
                     "printing ICU values as literals again")
        return
    for n in names:
        uses = len(re.findall(r"\b" + n + r"\b", code)) - 1   # minus the #define
        if uses < 1:
            fails.append(f"{n} is defined but never read. The banner has a "
                         f"conversion with no argument, or the constant was "
                         f"orphaned. See RDU boot banner.")
    if not any(n in f for f in fails for n in names):
        print(f"[OK] {len(names)} EXPECTED_ICU_* constant(s) defined and read")


def check_telemetry_payload(root):
    """ISSUE 1: one canonical TelemetryPayload, three copies.

    EVU2.ino declares its own copy and does not include GreenwaveTypes.h
    (its own comment says so). The signature covers exactly these bytes,
    so a one-field drift between the signer and either verifier produces
    universal signature failure with nothing in any log naming the cause.
    """
    NAME = "TelemetryPayload"
    CANONICAL = os.path.join(root, "RDU_L1N1", "GreenwaveTypes.h")

    copies = [
        ("RDU  (canonical)", CANONICAL),
        ("ICU ", os.path.join(root, "ICU", "GreenwaveTypes.h")),
        ("EVU ", os.path.join(root, "EVU2", "EVU2.ino")),
    ]

    parsed = {}
    for label, path in copies:
        if not os.path.isfile(path):
            fails.append(f"{NAME}: cannot find {path}")
            return
        f = _parse_struct(open(path, encoding="utf-8", errors="replace").read(), NAME)
        if f is None:
            fails.append(f"{NAME}: no packed struct of that name in {path}")
            return
        parsed[label] = (f, path)

    ref_label, (ref, ref_path) = "RDU  (canonical)", parsed["RDU  (canonical)"]
    ok = True
    for label, (f, path) in parsed.items():
        if label == ref_label:
            continue
        if f == ref:
            continue
        ok = False
        fails.append(f"{NAME} DIFFERS: {label} ({path})")
        fails.append(f"    canonical is {ref_path}")
        offs = _offsets(ref)
        for i in range(max(len(ref), len(f))):
            a = ref[i] if i < len(ref) else ("<missing>", "")
            b = f[i] if i < len(f) else ("<missing>", "")
            if a != b:
                at = offs[i][0] if i < len(offs) and offs[i][0] is not None else "?"
                fails.append(f"    field {i} (byte offset {at}): "
                             f"canonical `{a[0]} {a[1]}`  vs  {label.strip()} `{b[0]} {b[1]}`")
        fails.append("    The EVU signs these bytes and the RDU verifies them. A "
                     "mismatch here fails EVERY signature with no log line "
                     "naming the cause. Fix the copy, do not change the canonical.")
    if ok:
        print(f"[OK] {NAME} identical across RDU / ICU / EVU ({len(ref)} fields)")


def check_wire_types(root):
    """GreenwaveTypes.h is not byte-identical between ICU and RDU (the ICU
    copy carries extra commentary). Compare the WIRE CONTENT instead."""
    a = os.path.join(root, "RDU_L1N1", "GreenwaveTypes.h")
    b = os.path.join(root, "ICU", "GreenwaveTypes.h")
    if not (os.path.isfile(a) and os.path.isfile(b)):
        warns.append("GreenwaveTypes.h cross-role check skipped (file missing)")
        return
    ta, tb = (open(x, encoding="utf-8", errors="replace").read() for x in (a, b))

    da = dict(re.findall(r"#define\s+(GW_\w+|EVF_\w+|HBF_\w+)\s+([^\n/]+)", ta))
    db = dict(re.findall(r"#define\s+(GW_\w+|EVF_\w+|HBF_\w+)\s+([^\n/]+)", tb))
    bad = [k for k in set(da) & set(db) if da[k].strip() != db[k].strip()]
    only_a, only_b = sorted(set(da) - set(db)), sorted(set(db) - set(da))
    for k in bad:
        fails.append(f"GreenwaveTypes.h: {k} = {da[k].strip()} (RDU) vs "
                     f"{db[k].strip()} (ICU)")
    for k in only_a:
        fails.append(f"GreenwaveTypes.h: {k} defined in RDU copy only")
    for k in only_b:
        fails.append(f"GreenwaveTypes.h: {k} defined in ICU copy only")
    if not (bad or only_a or only_b):
        print(f"[OK] GreenwaveTypes.h wire constants agree RDU/ICU ({len(da)} defines)")


def check_paired_constants(root):
    """ISSUE 3: constants one role documents about another.

    The RDU's boot banner announces the ICU values it was built against.
    Nothing compiles across two sketch folders, so nothing caught the
    banner going stale when the ICU moved 40000 -> 60000. This is that
    check.
    """
    icu = os.path.join(root, "ICU", "ICU.ino")
    rdu = os.path.join(root, "RDU_L1N1", "RDU_L1N1.ino")
    if not (os.path.isfile(icu) and os.path.isfile(rdu)):
        warns.append("paired-constant check skipped (file missing)")
        return
    ti, tr = (open(x, encoding="utf-8", errors="replace").read() for x in (icu, rdu))

    PAIRS = [("HEARTBEAT_TIMEOUT_MS", "EXPECTED_ICU_HB_TIMEOUT_MS"),
             ("EVENT_TIMEOUT_MS", "EXPECTED_ICU_EVENT_TIMEOUT_MS")]

    for icu_name, rdu_name in PAIRS:
        mi = re.search(r"#define\s+" + icu_name + r"\s+(\d+)", ti)
        mr = re.search(r"#define\s+" + rdu_name + r"\s+(\d+)", tr)
        if not mi:
            fails.append(f"{icu_name} not found in ICU.ino")
            continue
        if not mr:
            fails.append(f"{rdu_name} not found in the RDU sketch -- the banner "
                         f"is printing a literal again")
            continue
        if int(mi.group(1)) != int(mr.group(1)):
            fails.append(f"PAIRED CONSTANT DRIFT: ICU {icu_name}={mi.group(1)} "
                         f"but RDU {rdu_name}={mr.group(1)}. The RDU boot banner "
                         f"will announce a value the ICU is not using.")
        else:
            print(f"[OK] {icu_name} = {mi.group(1)} agrees with RDU {rdu_name}")

    # Free-text mentions of a stale number are how this drifted the first
    # time: the #define moved and the prose next to it did not.
    #
    # Re-read HEARTBEAT_TIMEOUT_MS here rather than reusing `mi` from the
    # loop above -- that variable holds whichever pair ran last, so the
    # comparison silently used EVENT_TIMEOUT_MS. Exactly the class of
    # defect this function exists to catch, in the function that catches it.
    m_hb = re.search(r"#define\s+HEARTBEAT_TIMEOUT_MS\s+(\d+)", ti)
    if not m_hb:
        return
    for m in re.finditer(r"HEARTBEAT_TIMEOUT_MS[^\n]{0,60}?(\d{4,6})", tr):
        v = int(m.group(1))
        if v != 0 and str(v) != m_hb.group(1):
            warns.append(f"RDU sketch mentions HEARTBEAT_TIMEOUT_MS near the "
                         f"literal {v}; the ICU value is {m_hb.group(1)}. "
                         f"Comment may be stale.")


def check_bypass_cap_present(root):
    """The host test in tests/ mirrors the cap. Confirm the real one exists
    and still runs BEFORE the near-zone clamp -- reordering is the change a
    reviewer misses and the test cannot see."""
    p = os.path.join(root, "ICU", "ICU_Decision.ino")
    if not os.path.isfile(p):
        warns.append("bypass-cap check skipped (ICU_Decision.ino missing)")
        return
    t = open(p, encoding="utf-8", errors="replace").read()
    if "gwEvuLaneUnenforced" not in t:
        fails.append("ICU_Decision.ino no longer applies the v7 geofence cap "
                     "(gwEvuLaneUnenforced not called) -- a bypassed report can "
                     "reach COMMIT again")
        return
    i_cap = t.find("gwEvuLaneUnenforced")
    i_near = t.find("d.nearZone = true")
    i_rec = t.find("gwEvuIsReceding(lane))")
    if not (i_rec < i_cap < i_near):
        fails.append("v7 geofence cap is out of order in gwBuildDemand(). It must "
                     "run AFTER the receding cap and BEFORE the NEAR_ZONE clamp; "
                     "see tests/test_bypass_cap.cpp case 7.")
    else:
        print("[OK] v7 geofence cap present and correctly ordered")


def _fn_span(src, sig):
    """Byte span of a function body, by brace matching from its signature."""
    i = src.find(sig)
    if i < 0:
        return None
    j = src.find("{", i)
    if j < 0:
        return None
    d, k = 0, j
    while k < len(src):
        if src[k] == "{":
            d += 1
        elif src[k] == "}":
            d -= 1
            if d == 0:
                return (i, k)
        k += 1
    return None


def check_rx_pipeline_wired(root):
    """v7.1: the receive pipeline must be WIRED IN, not merely present.

    Written at the same time as the pipeline, not after an incident with it.
    The precedent is EXPECTED_ICU_HB_TIMEOUT_MS: defined, documented,
    cross-checked by this very script -- and never read, because every check
    verified the constant's VALUE and none verified that anything USED it.
    README section 8 lists that failure shape twice.

    A receive substrate is a much larger thing to get into that state. If
    ICU_RxPipeline.h were present and correct but not included, or included
    but never called, the ICU would silently keep running the v7 sequential
    path while the tree looked solved. So: check that something uses it.

    Also checks the two invariants a future reader is most likely to break
    while "fixing" this code -- one owner for the radio, and no lock around
    validation.
    """
    icu = os.path.join(root, "ICU", "ICU.ino")
    hdr = os.path.join(root, "ICU", "ICU_RxPipeline.h")
    if not (os.path.isfile(icu) and os.path.isfile(hdr)):
        fails.append("ICU.ino or ICU_RxPipeline.h missing -- the v7.1 receive "
                     "pipeline is gone")
        return

    src = open(icu, encoding="utf-8", errors="replace").read()
    code = _strip_comments(src)
    # #include <LoRa.h> is not a use of the radio. Blank include lines out
    # rather than deleting them, so byte offsets stay meaningful in the
    # diagnostics below.
    inc_free = re.sub(r"^\s*#\s*include.*$",
                      lambda m: " " * len(m.group(0)), code, flags=re.M)

    # 1. the header is actually included
    if '#include "ICU_RxPipeline.h"' not in code:
        fails.append("ICU.ino does not include ICU_RxPipeline.h. The receive "
                     "pipeline substrate is present but unwired -- the ICU is "
                     "silently back on the v7 sequential path. See README 7.4.")
        return

    # 2/3. defined AND called. Definition alone is what went wrong last time.
    for name, caller in (("gwStartRxPipeline", "setup"),
                         ("gwServiceValidated", "loop"),
                         ("gwDrainPipeLog", "loop")):
        if ("void " + name) not in code:
            fails.append(f"{name}() is not defined in ICU.ino")
            continue
        span = _fn_span(code, "void " + caller + "()")
        if span is None:
            fails.append(f"cannot locate {caller}() in ICU.ino")
            continue
        if name not in code[span[0]:span[1]]:
            fails.append(f"{name}() is defined but NEVER CALLED from {caller}(). "
                         f"The receive pipeline is built and not used -- the "
                         f"'defined, documented, never read' defect again.")

    # 4. the old sequential path is really gone
    if re.search(r"\bpollLoRa\s*\(", code):
        fails.append("pollLoRa() is still called. The v7 sequential receive "
                     "path and the v7.1 pipeline cannot both own the radio.")

    # 5. ONE OWNER FOR THE RADIO. Every LoRa.* use must sit inside either
    #    initCentralLoRa() (setup, before the task starts) or gwRxTask().
    #    Two readers of one SX127x FIFO is a silent corruption, not an error.
    allowed = []
    for sig in ("void initCentralLoRa()", "static void gwRxTask(void *)"):
        sp = _fn_span(code, sig)
        if sp is None:
            fails.append(f"cannot locate {sig} in ICU.ino")
        else:
            allowed.append(sp)
    for m in re.finditer(r"\bLoRa\s*\.", inc_free):
        if not any(a <= m.start() <= b for a, b in allowed):
            fails.append("LoRa.* is touched outside initCentralLoRa()/gwRxTask() "
                         f"at offset {m.start()}. The radio must have exactly "
                         f"one owner.")
            break
    for f in glob.glob(os.path.join(root, "ICU", "*.ino")):
        if os.path.basename(f) == "ICU.ino":
            continue
        other = re.sub(r"^\s*#\s*include.*$", "", _strip_comments(
                open(f, encoding="utf-8", errors="replace").read()), flags=re.M)
        if re.search(r"\bLoRa\s*\.", other):
            fails.append(f"{os.path.basename(f)} touches LoRa.* -- only "
                         f"gwRxTask() may.")

    # 6. per-node security state must still be claimed, and the claim must
    #    NOT be held across nothing (i.e. the drain must be inside it).
    dn = _fn_span(code, "static void gwWorkerTask(void *arg)")
    if dn is None:
        fails.append("gwWorkerTask() not found")
    else:
        body = code[dn[0]:dn[1]]
        if "gwClaimNode" not in body or "gwReleaseNode" not in body:
            fails.append("gwWorkerTask() no longer claims/releases the node. "
                         "Per-node ownership is what makes duplicate "
                         "suppression correct WITHOUT a global lock; without "
                         "it two workers can race one RDU's replay counter.")
        if body.find("gwClaimNode") > body.find("gwDrainNode"):
            fails.append("gwWorkerTask() drains a node before claiming it.")

    # 7. a global mutex around validation is the thing the brief forbids and
    #    the thing a future 'fix' will reach for first.
    val = _fn_span(code, "static void gwValidateSlot(int k, const GwRxSlot *s)")
    if val:
        body = code[val[0]:val[1]]
        for bad in ("portENTER_CRITICAL", "xSemaphoreTake", "std::lock_guard"):
            if bad in body:
                fails.append(f"gwValidateSlot() contains {bad} -- cryptographic "
                             f"validation has been put back behind a lock. "
                             f"Per-node claims exist so it does not need one.")

    t = os.path.join(root, "tests", "test_icu_rx_pipeline.cpp")
    if not os.path.isfile(t):
        warns.append("tests/test_icu_rx_pipeline.cpp missing -- the pipeline "
                     "has no host test")

    if not any("pipeline" in f or "gwRxTask" in f or "LoRa" in f or
               "gwWorkerTask" in f or "gwValidateSlot" in f or
               "gwStartRxPipeline" in f or "gwServiceValidated" in f or
               "gwDrainPipeLog" in f for f in fails):
        print("[OK] v7.1 receive pipeline wired in, radio has one owner, "
              "per-node claims intact")


if __name__ == "__main__":
    root = sys.argv[1] if len(sys.argv) > 1 else "."
    main(root)
    print()
    check_telemetry_payload(root)
    check_wire_types(root)
    check_paired_constants(root)
    check_bypass_cap_present(root)
    check_constants_used(root)
    check_rx_pipeline_wired(root)
    print()
    for w in warns:
        print("[WARN] " + w)
    for f in fails:
        print("[FAIL] " + f)
    if fails:
        print("\nDO NOT FLASH.")
        sys.exit(1)
    print("\nTree invariants hold." + ("  (see warnings above)" if warns else ""))
