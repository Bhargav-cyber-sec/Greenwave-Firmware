# tests/

Host-compiled. No ESP32, no Arduino toolchain.

```bash
g++ -std=c++11 -Wall -o test_bypass_cap test_bypass_cap.cpp && ./test_bypass_cap

g++ -std=gnu++17 -O2 -pthread -DGW_HOST_TEST -I../ICU \
    test_icu_rx_pipeline.cpp -o test_icu_rx_pipeline && ./test_icu_rx_pipeline
```

Both exit 0 on success. Run the second one under sanitizers too — it is a
concurrency test, and a concurrency test that has only been run without
TSan has not really been run:

```bash
g++ -std=gnu++17 -g -O1 -pthread -fsanitize=address,undefined -DGW_HOST_TEST \
    -I../ICU test_icu_rx_pipeline.cpp -o /tmp/t && /tmp/t
g++ -std=gnu++17 -g -O1 -pthread -fsanitize=thread -DGW_HOST_TEST \
    -I../ICU test_icu_rx_pipeline.cpp -o /tmp/t && /tmp/t
```

## `test_bypass_cap.cpp` — v7 geofence-enforcement cap

11 cases covering the stage rules in `gwBuildDemand()`.

| | Proves |
|---|---|
| 1, 2, 9 | **normal production reports behave exactly as before** — single node, two nodes, and DEGRADED evidence |
| 3, 4 | a bypassed report caps to PREPARE and cannot COMMIT |
| 5 | an *unsurveyed* node caps too — it sets `PASS` while failing open, so any rule keyed on `PASS` alone would have trusted it |
| 6 | trust is sticky: a later production packet does not restore a tainted track |
| 7 | the cap runs **before** the NEAR_ZONE clamp |
| 8 | the receding cap and the bypass cap compose; neither overwrites the other |
| 10 | `ALLOW_BYPASSED_COMMIT 1` reproduces pre-v7 behaviour exactly |
| 11 | the bypass flag stays readable for logging regardless of the cap |

## What this does not prove

`gwBuildDemand()` cannot compile on a host — it pulls in the whole ICU sketch.
This file **reproduces** the rule sequence with the same operators in the same
order. It proves the rule and the ordering are right. It does **not** prove the
copy in `ICU_Decision.ino` matches.

That gap is closed from the other side: `verify_rdu_tree.py` greps
`ICU_Decision.ino` and fails if the cap is missing, or if it is not positioned
between the receding cap and the NEAR_ZONE clamp.

Two mechanisms, neither sufficient alone. Read both when changing the cap.

## Case 7 is the one to keep

Reordering is the change a reviewer misses in a diff and a truth-table test
cannot see. If the cap moved after the NEAR_ZONE clamp, a bypassed vehicle 30 m
from the stop line would be marked maximally urgent on the strength of a
distance derived from the very report whose position is not trusted. Every other
case still passes. Only case 7 fails.

## `test_icu_rx_pipeline.cpp` — v7.1 receive/validation concurrency

31 assertions across 13 cases, driven by **real threads** against the **real**
`ICU/ICU_RxPipeline.h`. The rings, the memory orderings and the claim protocol
under test are the ones the firmware compiles.

| Case | Proves |
|---|---|
| 1 | one RDU, one request → one accepted transaction |
| 2 | the same counter twice, sequentially → one accepted, one *benign duplicate* |
| 3 | the same counter twice **simultaneously**, 4 workers contending → still one accepted |
| 4 | two different RDUs progress independently; each advances only its own counter |
| 5 | one node's epoch rotation leaves the other node's session state untouched |
| 5b | SOP 5.3 overlap window: a frame from the prior epoch still verifies |
| 6 | two packets from one RDU keep their order; in-order arrival never reads as replay |
| 7 | a forged tag is rejected **and does not advance the replay counter** |
| 8 | a *lower* counter is a REPLAY, not a duplicate — the v3.1 semantic split survives |
| 9 | a revoked node is refused at session establishment, before any tag check |
| 10 | a short frame is rejected on length before any cryptography runs |
| 11 | RX ring overflow is bounded, counted, and never silently swallowed |
| 12 | 6 nodes × 4 workers sustained: **no two workers ever inside one node's security state** |
| 13 | latency scaling with 1/2/3 concurrent RDUs against a forced 3 ms derivation |

Cases 3 and 12 are the load-bearing ones. Case 3 is the race the brief asks
about (`§11`): two copies of one transaction, handled concurrently, must not
both be accepted. Case 12 is the invariant that makes case 3 true — the claim
protocol — checked continuously under contention rather than once.

Case 7's second assertion is the easy thing to get wrong: it is not enough that
a forged frame is rejected. If it advanced `lastCounter` on the way out, an
attacker could push a node's counter forward and lock out the genuine RDU.

## What this does not prove

`gwValidateSlot()` cannot compile on a host — it needs Arduino, LoRa and
mbedTLS. `validateSlot()` in this file **mirrors** its order of operations, the
same way `test_bypass_cap.cpp` mirrors the geofence cap. Mirrors drift.

That gap is closed from the other side, the same way: `verify_rdu_tree.py`'s
`check_rx_pipeline_wired()` fails the tree if `ICU_RxPipeline.h` is not
included, if `gwServiceValidated()`/`gwStartRxPipeline()`/`gwDrainPipeLog()` are
defined but never called, if anything other than `gwRxTask()` touches `LoRa.*`,
if `gwWorkerTask()` stops claiming the node, or if a critical section reappears
inside `gwValidateSlot()`.

That last check matters most. A future reader who sees concurrent access to
shared security state will reach for a mutex — it is the obvious fix, and it is
the one thing this architecture must not have. The check says so, in the diff,
at the moment they try.

## The number in case 13 is not a speedup claim

With the derivation cost forced to 3 ms, three concurrent RDUs complete in
roughly two derivations' time, not one and not three, because there are two
workers. That is the honest shape: **bounded**, not eliminated. See README §7.4
for why two workers is the right count and why six would not help.
