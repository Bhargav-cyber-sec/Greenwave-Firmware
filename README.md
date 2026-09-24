# Priority One — Firmware

**Build:** v7.1 · three-way intersection · six roadside nodes
**Base:** `Allcodes_25thaug`
**Status:** RDU firmware complete. Four nodes carry placeholder keys and will not compile until provisioned.

---

## Read this first

**Nothing in this tree is ready to flash to a road.** Four of the six nodes have no private key. Every node is built with the geofence bypassed and its position unsurveyed. See §6.

**Run the verifier before every flashing session:**

```bash
pip install cryptography
python3 verify_rdu_tree.py .
```

Exit 0 means the tree is internally consistent. Exit 0 does *not* mean the tree is deployable.

**And run the host tests, which need no hardware:**

```bash
cd tests
g++ -std=c++11 -Wall -o test_bypass_cap test_bypass_cap.cpp && ./test_bypass_cap
g++ -std=gnu++17 -O2 -pthread -DGW_HOST_TEST -I../ICU \
    test_icu_rx_pipeline.cpp -o test_icu_rx_pipeline && ./test_icu_rx_pipeline
```

The second is a concurrency test. Run it under `-fsanitize=thread` as well —
see `tests/README.md`. A concurrency test that has only been run without TSan
has not really been run.

---

## 1. What the system is

Four components. This tree contains three of them; the dashboard is a separate repository.

| | Runs on | Role |
|---|---|---|
| **EVU** (`EVU2/`) | vehicle | Signs and broadcasts position, speed, heading, emergency flag |
| **RDU** (`RDU_L*N*/`) | roadside post | Verifies the vehicle signature, detects sirens with an on-device model, relays authenticated reports to the ICU |
| **ICU** (`ICU/`) | signal cabinet | Tracks vehicles, arbitrates between approaches, drives preemption |
| **ERC** (`ERC/`) | operator console | Displays alerts, independent timeout backstop |

Two radio links, deliberately different:

- **IF-1** — vehicle → roadside. 433.0 MHz, SF9. Long range, few transmitters.
- **IF-2** — roadside → cabinet. 434.5 MHz, SF7. Short range, six transmitters sharing one channel.

The RDU has **one radio** shared between both. Every millisecond it spends transmitting on IF-2 is a millisecond it is deaf to the ambulance on IF-1. Most of the scheduling complexity in `RDU_*.ino` exists for that reason.

---

## 2. Layout

```
Firmware/
├── verify_rdu_tree.py            run before every flash
├── HANDOFF_TO_SECURITY.md        provisioning brief
├── RDU_canonical.ino.MASTER      edit THIS, then fan out (§4)
├── tests/
│   ├── test_bypass_cap.cpp       host-compiled, no hardware
│   ├── test_icu_rx_pipeline.cpp  host-compiled, REAL THREADS (v7.1)
│   └── README.md
├── ICU/          ICU.ino + 7 partials + 6 headers
│                 ICU_RxPipeline.h is the v7.1 receive substrate
├── ERC/          console
├── EVU2/         vehicle unit
└── RDU_L1N1/ … RDU_L3N2/         six identical sketches, six different keys
```

---

## 3. The one rule

> **`RDU_L*N*.ino` is byte-identical in all six folders. Only `GreenwaveKeys.h` differs.**

The `.ino` is named per folder because Arduino requires the folder name and sketch name to match. That is an IDE constraint. It is one program, not six.

Node identity — lane, node number, private key — comes only from `GreenwaveKeys.h`. Distance to the stop line comes from NVS (`dist <m>` on site). Site geometry comes from NVS on the ICU (`site` commands).

**Why not six separately edited sketches.** Identity used to live in the `.ino`, and regenerating two node builds silently lost the `NODE_DISTANCE_SURVEYED` flag twice. The key file is the one place identity cannot be wrong without the ICU noticing, because a mismatched key fails authentication immediately and loudly.

**Why not one image with six keys.** Opening one roadside enclosure and dumping its flash would then compromise every node at the intersection, including approaches the attacker never touched.

---

## 4. Making a firmware change

```bash
# 1. edit the master, never a folder copy
vim RDU_canonical.ino.MASTER

# 2. fan out
for d in RDU_L*/; do n=$(basename $d); cp RDU_canonical.ino.MASTER $d/$n.ino; done

# 3. verify
python3 verify_rdu_tree.py .
```

Editing one folder's copy directly is the mistake this layout exists to prevent, and step 3 catches it.

---

## 5. What the verifier checks

| Check | Catches |
|---|---|
| `.ino` identical ×6 | someone edited one folder |
| 17 support files identical ×6 | a partial copy |
| `GreenwaveKeys.h` **differs** ×6 | **the same folder flashed twice** |
| lane/node macros match folder name | mislabelled key file |
| private keys distinct | two boxes sharing one identity |
| key ↔ ICU table index | **off-by-one → 100% `BAD TAG`, nothing else in any log** |
| `TelemetryPayload` identical RDU/ICU/EVU | signature failure with no diagnosable cause |
| `GreenwaveTypes.h` wire constants agree | protocol drift between roles |
| ICU ↔ RDU paired constants | boot banner announcing a value the ICU isn't using |
| geofence cap present and ordered | the v7 security fix silently removed |
| `EXPECTED_ICU_*` defined **and read** | a banner conversion with no argument |
| RX pipeline **wired in**, not merely present | the v7.1 substrate present but unwired (§7.4) |
| `LoRa.*` touched only by `gwRxTask()` | a second reader of a FIFO that has one |
| node claim present in `gwWorkerTask()` | two workers racing one RDU's replay counter |
| no critical section inside `gwValidateSlot()` | HMAC put back behind a global lock |

Every one of these was written after the corresponding mistake was actually made. See §8.

---

## 6. Before this touches a road

Four blockers, in order.

**1. Four nodes have no private key.** `RDU_L2N1`, `RDU_L2N2`, `RDU_L3N1`, `RDU_L3N2` ship a `GreenwaveKeys.h` that deliberately `#error`s. The ICU already holds all six *public* keys; the matching privates were never in this tree. See `HANDOFF_TO_SECURITY.md`.

**2. The existing key set is public.** Set `7e200d93` was transmitted in plain text during development. Regenerate all seven images — six nodes plus the ICU. There is no partial regeneration: the ICU's table and the six privates are one set.

**3. The geofence is bypassed on every node.** `GEOFENCE_BYPASSED_FOR_SOP true`. As of v7 the ICU knows this and caps such reports to PREPARE (§7). The fix is on the node, not the ICU: survey the posts, set `LANE_NODE_POSITION_KNOWN true` and `GEOFENCE_BYPASSED_FOR_SOP false`.

**4. Duty cycle is unresolved.** Six nodes on 434.5 MHz at roughly 4% each. Whether that is licence-exempt under Indian WPC rules is not answered anywhere in this project. It needs answering, and nobody has.

---

## 7. v7 / v7.1 changes

Three v7 fixes, plus the v7.1 receive rework. Full reasoning is in the comments
at each site; this is the index.

### 7.1 Geofence bypass is now enforced, not merely logged

`EVF_GEOFENCE_BYPASS` reached the ICU and was printed. Nothing read it, so a bench report participated in preemption exactly like a production one.

Worse, the flag it sat next to was false. The RDU computed
`accepted = GEOFENCE_BYPASSED_FOR_SOP ? true : smartGeofenceDecision(...)`
and passed that same variable as `geofencePass`. With the bypass on, `smartGeofenceDecision()` was **never called** and `EVF_GEOFENCE_PASS` was set on every frame anyway. The ICU was told the geofence passed about a check that did not run.

**RDU:** one variable split into two — `accepted` (admission, unchanged) and `geofenceResult` (assessment, now always evaluated). New `EVF_GEOFENCE_ENFORCED` on a previously-unused flag bit, set only when the geofence was genuinely acting as a gate.

**ICU:** provenance recorded on `EvuTrack`, sticky for the track's life, exposed through `gwEvuLaneUnenforced()` and used to cap the stage at PREPARE in `gwBuildDemand()` — the same mechanism the receding check already uses.

Reading the three bits:

| ENFORCED | PASS | meaning |
|---|---|---|
| ✅ | ✅ | geofence ran as a gate, vehicle cleared it — **fully trusted** |
| ✅ | ❌ | cannot occur; the RDU would have dropped the packet |
| ❌ | ✅ | would have passed, but nothing rode on it (bypassed, or unsurveyed and failing open) |
| ❌ | ❌ | admitted **only** because the geofence was bypassed |

**This changes what the current build does, and that is the point.** Every node here is unenforced, so every EVU demand now caps to PREPARE. That is not a regression — it is the first time the bench configuration has been visible in the decision layer rather than only in a log line. `ALLOW_BYPASSED_COMMIT 1` restores the old behaviour explicitly and loudly, for bench runs that need COMMIT.

### 7.2 `TelemetryPayload` drift

Three hand-maintained copies: `EVU2.ino`, `RDU_L1N1/GreenwaveTypes.h`, `ICU/GreenwaveTypes.h`. All three currently agree. The EVU **signs exactly these bytes** and the RDU verifies them, so a one-field drift fails every signature with nothing in any log naming the cause.

Structure unchanged. `verify_rdu_tree.py` now compares the declared layout — types, order, array extents — across all three and names the diverging field and its byte offset.

*Long term:* a shared header is the right answer but is a build-system change. Arduino resolves includes per sketch folder, so it needs a library directory; and handing EVU2 the full `GreenwaveTypes.h` would drag ICU-facing types into the vehicle image, against the separation `ICU_EvuTypes.h` exists to preserve. The right shape is a small `GreenwaveWire.h` holding only `TelemetryPayload` and the `EVF_*` bits. The check above removes the urgency.

### 7.3 Stale `HEARTBEAT_TIMEOUT_MS`

The RDU documented 40000; the ICU runs 60000. **60000 is correct and is unchanged.** The ICU raised it alongside `NODE_FAILED_MS` because the RDU heartbeat doubled to 10 s in the duty-cycle work, making worst-case spacing 17.5 s — at 40000 the health layer would mark a node offline while the geometry layer still called it merely SUSPECT.

Fixed the RDU comment, and a runtime banner that was printing `40000` as a hardcoded string literal. Both ICU values are now constants (`EXPECTED_ICU_HB_TIMEOUT_MS`, `EXPECTED_ICU_EVENT_TIMEOUT_MS`) that the verifier cross-checks against `ICU.ino`.

---

### 7.4 v7.1 — the ICU receive path is no longer serialised behind the decision loop

**The reported symptom was real and the diagnosis in the review was not.** The
review said concurrent RDUs serialise behind HMAC validation, giving roughly
linear latency growth. Traced against the actual v7 code, that is not what
happens, and fixing the HMAC would have bought nothing.

**Why the HMAC is not the bottleneck.** Six RDUs share ONE 434.5 MHz channel at
SF7. Airtime, computed from the radio parameters this build sets:

| Frame | Size | Airtime |
|---|---|---|
| `AcousticEventFrame` | 30 B | 86.3 ms |
| `HeartbeatFrame` | 42 B | 104.7 ms |
| `LoRaEventFrame` | 56 B | 129.3 ms |

Two RDUs *cannot* deliver frames to this receiver at the same instant.
Overlapping transmissions collide and both are lost. So the ICU is handed at
most one frame per ~86 ms. Against that, HMAC-SHA256 over ≤56 bytes is four
SHA256 compressions — tens of microseconds, about 0.03% of the airtime of the
frame it authenticates. Six workers hashing in parallel would save nothing, and
could not run in parallel anyway.

**What the bottleneck actually was.** `pollLoRa()` held the radio for the whole
of validation *and* the decision-and-logging tail:

- `[RX2]` + `[LORA RX]` for one accepted event frame: **~31 ms of blocking UART
  inside the receive path**
- `printRejectSummary()` every 30 s: **≥240 ms**

The SX127x in continuous RX does not queue. A frame that completes while the
FIFO holds an unread one is **destroyed**. A 240 ms printout costs about two
frames outright, and that node's next re-assert is `ACOUSTIC_REASSERT_MS` (8 s)
away. That is the "more RDUs, more latency" effect — measured in **seconds of
lost re-assert**, not microseconds of queued cryptography.

**The fix.** Three stages, split at the two boundaries that matter:

```
LoRa RX  ──►  gwRxTask()            one owner of the radio. Copies bytes out,
   │                                does the four header checks that need no
   │                                per-node state, returns to the radio.
   │                                Never blocks, prints, or computes a MAC.
   ▼
per-node SPSC ring ×6               immutable frame copy, depth 4, no heap
   │
   ▼
gwWorkerTask() ×2                   claims ONE node by CAS, drains its ring:
   │                                replay check → keyFor → HMAC → advance
   │                                counter. Different nodes concurrent; the
   ▼                                same node never.
validated-event ring (MPSC)         spinlock around a memcpy, no crypto inside
   │
   ▼
gwServiceValidated() in loop()      the v7 handlers, unchanged, single-threaded
   │
   ▼
decision / tracks / ERC / Serial    untouched
```

**No global mutex, and per-node claims are why.** `gwNodeOwner[k]` is CASed
0→w+1. The worker holding node *k* is the only reader or writer of that node's
`lastCounter`, `counterStarted`, reject counters and session `Slot`. Two copies
of one counter land in *the same* ring in arrival order and are popped by *the
same* owner, so the first advances `lastCounter` and the second compares equal
and is suppressed. The §11 race is not prevented, it is made unrepresentable —
and nothing is held across the HMAC to achieve it. A worker that finds a node
claimed does not wait; it moves to the next node, so head-of-line blocking is
impossible and there is nothing to deadlock.

Ownership is now written down per field: every `NodeState` member is marked
`[W]` (worker-owned *security* state) or `[L]` (loop-owned *detection* state).
The split follows a real boundary, which is why it is clean.

**Why two workers, not six.** Not "because there are six RDUs". One worker
suffices for the HMAC. The second exists for exactly one operation: X25519 +
HKDF inside `keyFor()` on an epoch change — milliseconds, the only thing in the
path long enough to matter. With one worker that would stall a *different*
node's queued frame, which is the head-of-line blocking the change exists to
remove. A third adds nothing: worker stacks are sized by mbedTLS ECDH (8 kB
each), so six would be 48 kB of DRAM to parallelise an operation the air
delivers at most 8 times a second. Two is 16 kB and bounds the pile-up to one
in-progress derivation. `GW_VAL_WORKERS` is the dial; raise it only if a
measured `[PERF] wait=` starts tracking `key=`.

**Polled RX, not DIO0 interrupt.** arduino-LoRa's `onReceive()` runs
`handleDio0Rise()`, which does SPI in ISR context — a known ESP32 crash source.
"Intermittent crash in the ambulance-detection path" is not a trade worth one
millisecond. One tick of polling latency against 86–129 ms of airtime is ~1%.

**Two premises in the review that the code does not support.** The ICU never
transmits on LoRa — it is receive-only (`GreenwaveCrypto.h` says so, and the
only call is `LoRa.receive()`). There is no IF-2 ACK to arbitrate; the only ACK
is the operator's `ACKNOWLEDGED` over UART2 from the ERC. And no protocol
change was needed: frame layouts, `TelemetryPayload`, counter and tag offsets
and the identity scheme are all untouched.

**New observability.** `[PIPE]` reports every drop path (per-node ring drops and
high-water marks, validated-event drops, log drops) and whether the RX task is
still beating — a wedged RX task is a *new* failure mode this change
introduces, and an ICU that goes deaf while `loop()` keeps feeding the watchdog
would otherwise look perfectly healthy. `[PERF]` reports measured min/mean/max
for each stage: queue, wait, key, hmac, hand, total.

**Guarding the new machinery.** `ICU_RxPipeline.h` is new in v7.1, and a
substrate that is present, correct and *unwired* would fail in exactly the shape
§8 already describes twice — defined, documented, cross-checked, never read. So
`check_rx_pipeline_wired()` fails the tree if the header is not included, if the
entry points are defined but never called, if anything but `gwRxTask()` touches
`LoRa.*`, if the node claim disappears from `gwWorkerTask()`, or if a critical
section reappears inside `gwValidateSlot()`. All five have been negative-tested
by deliberately introducing each regression and confirming the check fires.

That last one is the important one. A future reader who sees concurrent access
to shared security state will reach for a mutex — it is the obvious fix, and it
is the one thing this architecture must not have. The check says so in the diff,
at the moment they try.

## 8. Mistakes this tree has already made

Kept because each one is now a check, and because the next person will be tempted by the same shortcut.

| | |
|---|---|
| Placeholder keys generated by incrementing a byte | index 0 worked *by accident*, every other node failed 100% `BAD TAG`. Invisible with one node, fatal with two |
| Identity in the `.ino` | `NODE_DISTANCE_SURVEYED` silently lost twice |
| `HEARTBEAT_MAX_DEFER_MS` corrected, banner literal not | v3.6 banner announced 12000 on a node built with 6000 — in the line whose whole job is catching that |
| Same defect again in v7 | changed the literal to `%lu` and forgot the argument. Constant defined, documented, cross-checked, **never read**. Every other check passed because they all verified the *value* and none verified that anything used it |
| ETA from instantaneous GPS speed | number visibly bounced on the console; an officer watching a number bounce learns to distrust it |
| A stagger applied after the window gate cleared the frame | moved the frame out of the window that cleared it |

The last two lines are the same lesson twice: **a value can be correct and still be wrong at the point it is consumed.**
