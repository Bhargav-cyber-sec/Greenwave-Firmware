# RDU three-lane scaling — handoff

**From:** embedded / IoT (firmware scaling)
**To:** security (key provisioning)
**Base:** `Allcodes_25thaug`
**Status:** firmware complete for six nodes. Four nodes have **placeholder keys** and will not build until provisioned.

---

## 1. What was delivered

Six Arduino folders, `RDU_L1N1` … `RDU_L3N2`.

```
Firmware/
├── ICU/            unchanged
├── ERC/            unchanged
├── EVU2/           unchanged
├── RDU_L1N1/  keys REAL          RDU_L2N1/  keys PLACEHOLDER
├── RDU_L1N2/  keys REAL          RDU_L2N2/  keys PLACEHOLDER
│                                 RDU_L3N1/  keys PLACEHOLDER
│                                 RDU_L3N2/  keys PLACEHOLDER
└── verify_rdu_tree.py
```

**The `.ino` is byte-identical in all six folders** (`md5 e650833343a90d98048be1f0e38e2261`). So are all 17 shared support files. **Only `GreenwaveKeys.h` differs.** `verify_rdu_tree.py` enforces this; run it before every flashing session.

The file is named per folder (`RDU_L1N1.ino`, `RDU_L2N1.ino`, …) only because Arduino requires the folder name and the `.ino` name to match. That is an IDE constraint, not six different programs. **Do not edit one folder's `.ino`** — edit the canonical file, copy it to all six, re-run the verifier.

### Naming change from the 25 Aug tree

`RDU_L1N1/RDU1.ino` and `RDU_L1N2/RDU2.ino` did not match their folder names, so neither folder actually opened in the Arduino IDE as-is. Both renamed.

---

## 2. What changed in the firmware

Identity was already derived from the key file (`#define LANE_ID GW_KEY_LANE_ID`), so that part was done. Everything below is new.

### 2.1 Six-node transmit slotting (the main change)

The old scheme keyed on `NODE_ID` alone: a heartbeat offset of half an interval, and a 250 ms `vTaskDelay` on node 2 inside `radiateToICU()`. Three things broke at six nodes.

**(a) It cannot see the lane.** `NODE_ID` is 1 or 2 regardless of approach, so all three lane-N1 nodes fired together and all three lane-N2 nodes fired together.

**(b) The stagger was applied after the window check — a defect at two nodes, today.** `transportTask` calls `icuTransmitWindowOpen()`, gets clearance, calls `radiateToICU()`, which then sleeps 250 ms. The usable window is 795 ms wide. A frame cleared at `elapsed = 1600 ms` starts at 1850 ms, inside the guard band. The stagger invalidated the check that permitted it. Widening the stagger to 500 ms, as the scaling note proposed, would put the frame at 2100 ms — on top of the next EVU frame, every time.

**(c) Three lanes create a new contention source.** FIX 1 splits relay duty by `NODE_ID` parity against the EVU sequence number, **not by lane**. All six nodes hear the same EVU beacon, so on sequence *N* all three lane-N1 nodes independently decide "my turn". At two nodes that was one transmitter; at six it is three.

**Also rejected:** the 100 ms slot spacing in §8.2 of the scaling note. Slot *spacing* must exceed frame *airtime*; total span is irrelevant. At 100 ms spacing and 165 ms frames every adjacent pair overlaps by 65 ms. And six slots cannot fit one window at any spacing: 795 ms usable ÷ 220 ms per slot ≈ 3.

**What replaced it: phase, not delay.** A node transmits only while `millis()` lies inside its assigned phase of the EVU period. If the phase has passed it waits for the next one. Clearance and separation are now the same test, so nothing can be delayed out of a window it was cleared for.

Six slots fold across two EVU periods:

| Node | slot | parity | phase after EVU frame start |
|---|---|---|---|
| L1N1 | 0 | 0 | 840–1060 ms |
| L1N2 | 1 | 1 | 840–1060 ms |
| L2N1 | 2 | 0 | 1060–1280 ms |
| L2N2 | 3 | 1 | 1060–1280 ms |
| L3N1 | 4 | 0 | 1280–1500 ms |
| L3N2 | 5 | 1 | 1280–1500 ms |

The fold matches FIX 1: node-1s and node-2s already alternate periods, so this follows the traffic that exists rather than imposing a second rhythm on top. It also resolves (c) — the three simultaneous relayers land in three different slots.

Period parity comes from **the EVU's sequence number**, not a locally counted period index. A node that missed two frames would count periods from a different anchor than its neighbour and silently pick the wrong slot.

### 2.2 Supporting changes

| | |
|---|---|
| `lastEVUSeq` / `lastEVUSeqValid` | New. Set only on live frames, never retro replays — same rule as `lastEVUTxStartMillis`. |
| `OutFrame.fallingEdge` | New. Captured at queue time, not read from the file-scope flag at transmit time — the frame sits in a queue and the flag may by then describe a different event. |
| LBT bounded | `3 × random(5,50)` (up to 150 ms, 68% of a slot) → `2 × random(5,40)`. |
| Heartbeat bypass narrowed | Was `icuTransmitWindowOpen(...) || hbOverdue`, which let an overdue heartbeat transmit **on top of the ambulance**. Now `hbOverdue` bypasses the slot only; the EVU guard is never bypassed. |
| `static_assert`s | Lane/node range, intersection match, slot range, and slot-fits-in-window. All compile-time. |
| Boot banner | Prints slot, phase, and provisioning state. |

---

## 3. Your part

### 3.1 The four placeholder key files

Each of `RDU_L2N1`, `RDU_L2N2`, `RDU_L3N1`, `RDU_L3N2` has a `GreenwaveKeys.h` that **deliberately fails to compile**. A placeholder that builds is a placeholder that ships.

The ICU already holds all six **public** keys (set `7e200d93`, index `(lane-1)*2 + (node-1)`). Each placeholder file quotes the public key it must pair with. The matching **private** keys were never in the firmware tree and cannot be derived from the public halves.

### 3.2 Recommendation: regenerate the whole set

Every existing key file header records that set `7e200d93` was published in a chat log and a git history and "must be treated as public." Two further reasons:

- Partial regeneration is impossible in principle — the ICU's table and the six private keys are one set, so the ICU is reflashed either way.
- L1N1 and L1N2 are already compromised by that disclosure, so recovering the four missing privates would still leave a public key set in service.

Reflash **all seven images**: six nodes plus the ICU.

### 3.3 Verify before flashing — this is the one that bites

For every node, derive the private key's public half and check it against the ICU table at index `(lane-1)*2 + (node-1)`.

`verify_rdu_tree.py` does this automatically for any node with a real key:

```bash
pip install cryptography
python3 verify_rdu_tree.py Firmware/
```

Expected once you're done — no warnings, six pairings:

```
[OK] RDU_*.ino identical in all six
[OK] GreenwaveKeys.h differs in all six
[OK] 6 real private key(s), all distinct
[OK] L1N1 key pairs with ICU table index 0
...
[OK] L3N2 key pairs with ICU table index 5
```

An index off by one produces **100% `BAD TAG` from one node and nothing else in any log**. It is invisible with one node and fatal with two. Two folders sharing a key produces the same symptom plus replay rejections, with two boxes fighting over one identity.

### 3.4 Bench builds before you have keys

Define `GW_BENCH_UNPROVISIONED` to build an unprovisioned folder. The node boots, prints an unmissable warning banner, and is rejected by the ICU on every frame — which is the correct behaviour, not a workaround. Use it to confirm the firmware compiles and the slot banner is right; nothing else.

---

## 4. What I could not verify — please check

1. **`ICU_LORA_SF` in `ICU.ino` must be 7.** The RDU sends at SF7 on IF-2. A node at SF7 and an ICU at SF9 are completely deaf to each other, with no error message. I did not read the ICU radio config.
2. **`HEARTBEAT_TIMEOUT_MS` in `ICU.ino`** is derived from the RDU's heartbeat sizing. I did not change heartbeat timing, so it should still hold — but the slot scheme changes worst-case spacing and it is worth recomputing rather than assuming.
3. **The ICU's `if2=` statistics** are the measurement that says whether any of this worked. Threshold to watch: below ~70% on any node means the slots need re-sizing.

---

## 5. Known residuals — not fixed, deliberately

1. **Certificate periods.** Every 5th EVU frame occupies 1181 ms, so the window opens at 1381 ms and only position 0 fits. Positions 1 and 2 (lanes 2 and 3) lose one opportunity in five. Acceptable at a 10 s heartbeat; re-check if the heartbeat is shortened.
2. **Falling-edge latency** rises to ~4 s worst case (one skipped period). The ICU holds `loraDetected` for 20 s, so this is inside tolerance — but measure it, don't assume.
3. **Duty cycle is unchanged.** Slotting removes *collisions*, not *occupancy*. Whether ~4% per node on 434.5 MHz is licence-exempt under Indian WPC rules is an open question nobody on this project has answered. It needs answering before the pilot.
4. **This scheme is full at six nodes.** Three positions × two parities, zero spare. A four-way intersection does not fit, and neither does adding a camera node. It is the right way to run the three-approach prototype and a dead end beyond it — the long-term answer is a wired IF-2 (RS-485 or PoE) to a cabinet that already has mains power.

---

## 6. Commissioning order (unchanged from the plan, with one correction)

| Phase | |
|---|---|
| A | Flash L1N1 + L1N2 with the **new** firmware. Confirm approach USABLE, both HEALTHY, `if2 > 80%`. |
| B | Provision the four new nodes. Run the verifier. |
| C | Reflash the ICU + all six nodes with the new key set. |
| D | Add lane 2. Set distances (must differ by ≥85 m within an approach) and bearing. Watch `if2=`. |
| E | Add lane 3. Measure `if2=` on all six. |
| F | Multi-approach `simevu` testing. |
| G | MQTT — **only now**. |

**Correction to the earlier plan:** it had the slot change applied conditionally at a later phase, "if degradation appears." That is backwards. Defect 2.1(b) is live at two nodes today, so a Phase A baseline taken on the old firmware is measuring a bug. The slots go in first, while the change can still be attributed to two nodes.

Per node at install: flash → **label the box immediately** (`L2N1 — lane 2 FAR`) → read the boot banner → `dist <m>` on site. Six identical boards is exactly how a node ends up on the wrong post. Every one of the six boot banners must show a different slot line and a different config hash; two matching means the same folder was flashed twice.
