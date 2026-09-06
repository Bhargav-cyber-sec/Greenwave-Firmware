# Phase 4 — RDU firmware

## What changed

| Change | Why |
|---|---|
| `NODE_DISTANCE_TO_STOPLINE_M` | Node reports its own surveyed distance. ICU sorts → FAR/NEAR. |
| `NODE_APPROACH_ID`, `config_hash` | Catches a node flashed with the wrong lane's config |
| `mic_noise_floor`, `loop_liveness` | Detects a dead microphone and a hung inference task |
| `event_id`, `age_ms` on acoustic frames | Lets the ICU measure the FAR→NEAR time delta correctly |
| `heading_deg` on relay frames | Was being read and thrown away. Needed for approach association and divergence. |
| Airtime constants recomputed | Frames grew. Old values under-reserved the channel. |
| Boot configuration banner | Makes a mis-flashed node visible in five seconds |

---

## THE ONE THING THAT MATTERS

**All six RDUs run identical firmware.** They differ only by four `#define`
lines near the top of `RDU.ino`.

Flashing the wrong build to the wrong enclosure is the failure this phase
is built to prevent, because nothing about a running node makes it
visible — every field is individually plausible, so a mis-flashed node
reports confidently and accurately about the wrong piece of road.

Check the boot banner on every node after flashing.

---

## Settings per node

Edit these four lines, flash, label the enclosure, move to the next.

### Approach 1 (ERC "LEFT")

**Node 1 — outer, 250 m**
```c
#define LANE_ID 1
#define NODE_ID 1
#define NODE_DISTANCE_TO_STOPLINE_M  250
#define NODE_DISTANCE_SURVEYED  true
```

**Node 2 — inner, 150 m**
```c
#define LANE_ID 1
#define NODE_ID 2
#define NODE_DISTANCE_TO_STOPLINE_M  150
#define NODE_DISTANCE_SURVEYED  true
```

### Approach 2 (ERC "BOTTOM")
```c
#define LANE_ID 2
#define NODE_ID 1
#define NODE_DISTANCE_TO_STOPLINE_M  250
#define NODE_DISTANCE_SURVEYED  true
```
```c
#define LANE_ID 2
#define NODE_ID 2
#define NODE_DISTANCE_TO_STOPLINE_M  150
#define NODE_DISTANCE_SURVEYED  true
```

### Approach 3 (ERC "RIGHT")
```c
#define LANE_ID 3
#define NODE_ID 1
#define NODE_DISTANCE_TO_STOPLINE_M  250
#define NODE_DISTANCE_SURVEYED  true
```
```c
#define LANE_ID 3
#define NODE_ID 2
#define NODE_DISTANCE_TO_STOPLINE_M  150
#define NODE_DISTANCE_SURVEYED  true
```

---

## Rules

**1. Two nodes on one approach must never share a distance.**
The ICU cannot order them, marks the approach MISCONFIGURED, and disables
automatic action on it. That is the correct response to geometry it was
not given — but it means that approach does nothing.

**2. `NODE_DISTANCE_SURVEYED` stays `false` until you actually measure.**
A plausible number nobody surveyed passes every range check and quietly
becomes the basis for direction inference. False makes the node report
`GW_DISTANCE_UNSET` instead, so it fails loudly rather than confidently.

For bench testing on a table, `true` with fake distances is fine — the
point is that the two numbers differ and you know which box is which.

**3. Node ID does not decide FAR or NEAR.** Only distance does. You can
give node 2 the larger distance and it becomes RDU_FAR. That is the whole
design: swap two enclosures and the ICU follows the swap.

---

## Boot banner

Every node prints this. Read it before you close the enclosure.

```
================ NODE CONFIGURATION ================
  intersection=1  lane=1  node=1  approach=1
  distance to stop line : 250 m  [SURVEYED]
  config hash           : 0x3F2A91C4
  proto version         : 5   frames HB=42B ACO=30B EVT=58B
  airtime reserved      : HB=375ms ACO=305ms EVT=475ms
  mic self-test band    : 0.00030 .. 0.20000 rms
  role (FAR/NEAR)       : decided by the ICU, not here
====================================================
```

Check: **lane**, **node**, **distance**, and that `[SURVEYED]` appears.

If it says `NOT SURVEYED - ICU WILL REJECT`, you flashed a build with
`NODE_DISTANCE_SURVEYED false`.

**The config hash differs for every node.** Two nodes showing the same
hash means you flashed the same build twice.

---

## Flashing order

Do these one at a time and label each enclosure as it comes off the bench.

1. `GreenwaveTypes.h` must already be the Phase 1 version in `RDU/`
2. Edit the four lines → Verify → Upload → read banner → label the box
3. Repeat for all six

Start with **lane 1 only** (two nodes). Get one approach working before
building the other four.

---

## What to test

**Test 1 — banner.** Flash both lane-1 nodes. Confirm banner values,
confirm the two config hashes differ.

**Test 2 — ICU sees both.** Power both nodes and the ICU. The ICU should
receive heartbeats from `L1N1` and `L1N2`. It will not yet display roles
— that logic is Phase 5.

**Test 3 — microphone self-test.** Leave a node running two minutes, then
cover its microphone completely. The noise floor should fall. This is the
fault that was previously invisible: a node whose radio and battery are
healthy and whose microphone is dead heartbeats perfectly forever.

**Test 4 — siren episode.** Play a siren near one node:
```
[SIREN DETECTED] CONF=0.94 SCORE=3 ...
[SIREN SUSTAINED] ...
[SIREN CLEARED] event=1 duration=8420ms
```
Play it again — `event=2`. Each continuous detection gets its own ID.

---

## Known gap, on purpose

`HBF_TIME_VALID` is never set. These nodes have no disciplined clock, so
`age_ms` is only meaningful **relative to one node**, not across nodes.

Left honest rather than claiming a time source we do not have. Phase 5's
correlation logic must account for it, and it will — but it needs to know
the limitation exists rather than discovering it in the field.
