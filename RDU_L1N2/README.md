# RDU_L1N2

**Lane 1 (LEFT) · node 2 · NEAR (inner — last node before the stop line)**
ICU public-key table index **1** = `(lane-1)*2 + (node-1)`

---

## What this folder is

One of six identical sketches. `RDU_L1N2.ino` is **byte-identical** to the
other five — see the root `README.md` §3. Everything that makes this node
different from the others is in `GreenwaveKeys.h`.

**Do not edit `RDU_L1N2.ino` here.** Edit `../RDU_canonical.ino.MASTER`, fan
out to all six, then run `../verify_rdu_tree.py`.

## Keys — **REAL** — verified against ICU table index 1

This folder has a real, verified key from set `7e200d93`.

**That set is public.** It was transmitted in plain text during development. It is
fine for bench work and must be regenerated before deployment — together with the
other five nodes and the ICU, since they are one set.

## Transmit slot

Six nodes share one 434.5 MHz channel. This node transmits only inside its own
phase of the EVU period; it never waits, because a delay applied after the
window gate has cleared a frame moves that frame out of the window it was
cleared for.

| | |
|---|---|
| Slot index | **1** = `(lane-1)*2 + (node-1)` |
| EVU period parity | **1** (odd sequence numbers) |
| Position in window | **0** |
| Phase | **840-1060 ms** after EVU frame start |
| Heartbeat offset | **1666 ms** |
| Transmit opportunity | once per 4000 ms (2 EVU periods) |

The boot banner prints all of these. **No two of the six boxes may print the
same line** — two matching means the same folder was flashed twice.

## Commissioning

1. **Flash** this folder to one physical box.
2. **Label the box immediately:** `L1N2 — lane 1 NEAR`.
   Six identical boards is exactly how a node ends up on the wrong post.
3. **Read the boot banner.** Confirm `lane=1 node=2`, slot 1, and a config
   hash that differs from every node flashed so far.
4. **Set the distance on site:** `dist <metres>`.
   The two nodes on this approach must differ, and by **≥85 m**, or direction
   inference degrades. Equal distances disable the approach — the ICU says so
   explicitly.
5. Site geometry is set **once, on the ICU**, not here.

## Files

| | |
|---|---|
| `RDU_L1N2.ino` | identical in all six |
| `GreenwaveKeys.h` | **unique to this node** — contains a private key, never commit |
| `GreenwaveTypes.h`, `GreenwaveCrypto.h` | identical in all six |
| `gw_model*`, `gw_frontend*`, `kiss_fft*`, `*.tflite` | siren classifier, identical in all six |

## If this node fails

| Symptom | Almost certainly |
|---|---|
| 100% `BAD TAG` from this node only | wrong key file — verify against ICU index 1 before reflashing |
| `BAD TAG` plus replay rejections, two boxes fighting | same folder flashed twice |
| `IDENTICAL DISTANCES — cannot order nodes` | both nodes on lane 1 have the same `dist` |
| `if2=` below 70% | channel contention — re-measure all six before changing anything |
| Compile error naming `GW_KEY_*` | wrong, missing, or unprovisioned key file |

All provisioning errors, not code errors. The firmware detects every one.
