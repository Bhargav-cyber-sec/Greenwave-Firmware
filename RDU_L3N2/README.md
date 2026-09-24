# RDU_L3N2

**Lane 3 (RIGHT) · node 2 · NEAR (inner — last node before the stop line)**
ICU public-key table index **5** = `(lane-1)*2 + (node-1)`

---

## What this folder is

One of six identical sketches. `RDU_L3N2.ino` is **byte-identical** to the
other five — see the root `README.md` §3. Everything that makes this node
different from the others is in `GreenwaveKeys.h`.

**Do not edit `RDU_L3N2.ino` here.** Edit `../RDU_canonical.ino.MASTER`, fan
out to all six, then run `../verify_rdu_tree.py`.

## Keys — **PLACEHOLDER — this folder will not compile**

`GreenwaveKeys.h` here is a placeholder that deliberately `#error`s. A placeholder
that builds is a placeholder that ships.

The ICU already holds this node's **public** key at table index 5. The matching
private key was never in this tree and cannot be derived from the public half. The
placeholder file quotes the public key it must pair with.

To compile this folder for a bench check that does not need authentication, define
`GW_BENCH_UNPROVISIONED`. The node boots, prints an unmissable warning banner, and
is rejected by the ICU on every frame — which is correct behaviour, not a
workaround.

## Transmit slot

Six nodes share one 434.5 MHz channel. This node transmits only inside its own
phase of the EVU period; it never waits, because a delay applied after the
window gate has cleared a frame moves that frame out of the window it was
cleared for.

| | |
|---|---|
| Slot index | **5** = `(lane-1)*2 + (node-1)` |
| EVU period parity | **1** (odd sequence numbers) |
| Position in window | **2** |
| Phase | **1280-1500 ms** after EVU frame start |
| Heartbeat offset | **8330 ms** |
| Transmit opportunity | once per 4000 ms (2 EVU periods) |

The boot banner prints all of these. **No two of the six boxes may print the
same line** — two matching means the same folder was flashed twice.

## Commissioning

1. **Flash** this folder to one physical box.
2. **Label the box immediately:** `L3N2 — lane 3 NEAR`.
   Six identical boards is exactly how a node ends up on the wrong post.
3. **Read the boot banner.** Confirm `lane=3 node=2`, slot 5, and a config
   hash that differs from every node flashed so far.
4. **Set the distance on site:** `dist <metres>`.
   The two nodes on this approach must differ, and by **≥85 m**, or direction
   inference degrades. Equal distances disable the approach — the ICU says so
   explicitly.
5. Site geometry is set **once, on the ICU**, not here.

## Files

| | |
|---|---|
| `RDU_L3N2.ino` | identical in all six |
| `GreenwaveKeys.h` | **unique to this node** — contains a private key, never commit |
| `GreenwaveTypes.h`, `GreenwaveCrypto.h` | identical in all six |
| `gw_model*`, `gw_frontend*`, `kiss_fft*`, `*.tflite` | siren classifier, identical in all six |

## If this node fails

| Symptom | Almost certainly |
|---|---|
| 100% `BAD TAG` from this node only | wrong key file — verify against ICU index 5 before reflashing |
| `BAD TAG` plus replay rejections, two boxes fighting | same folder flashed twice |
| `IDENTICAL DISTANCES — cannot order nodes` | both nodes on lane 3 have the same `dist` |
| `if2=` below 70% | channel contention — re-measure all six before changing anything |
| Compile error naming `GW_KEY_*` | wrong, missing, or unprovisioned key file |

All provisioning errors, not code errors. The firmware detects every one.
