# ICU — Intersection Control Unit

Signal cabinet. Receives authenticated vehicle reports over IF-2 (434.5 MHz, SF7)
from six roadside nodes, tracks vehicles, arbitrates between three approaches,
and drives preemption.

## Files

| | |
|---|---|
| `ICU.ino` | main loop, timing constants, console |
| `ICU_Decision.ino` | demand tuple, stage rules, arbitration |
| `ICU_EVU.ino` | vehicle track ingest and lifecycle |
| `ICU_Tracks.ino` | track storage |
| `ICU_Safety.ino` | watchdog, commit rate limiting, RF noise floor |
| `ICU_Geometry.ino` | node roles, approach usability, health FSM |
| `ICU_Link.ino` | IF-2 framing and console link |
| `ICU_Site.ino` | site geometry, NVS |
| `ICU_EvuTypes.h` | **ICU-only** decision-layer types — deliberately not on the wire |
| `GreenwaveTypes.h` | wire protocol, shared with the RDU |
| `GreenwaveKeys.h` | ICU private key + all six RDU public keys |

## Three layers, kept apart

1. **Track layer** (`ICU_EVU.ino`) — who is out there, where, moving which way.
2. **Demand layer** (`gwBuildDemand`) — one tuple per approach: priority, evidence, stage, ETA.
3. **Arbitration** (`gwDemandScore`) — which approach wins.

Layer 3 reads only the demand tuple and never reaches back into per-track state.
Anything a decision needs must be *carried* into the tuple, not fetched. That is
why `nearZone` and `distM` are fields rather than recomputed calls.

## Stage rules, in the order they apply

```
base        DEGRADED evidence -> PREPARE, otherwise COMMIT
receding    moving away       -> cap to PREPARE, clear ETA
v7 bypass   geofence not enforced -> cap to PREPARE, clear ETA
near-zone   inside NEAR_ZONE_M and approaching -> urgency clamp
```

**The order is load-bearing.** The bypass cap must run after receding (so they
compose) and before the near-zone clamp (so a distance from an untrusted report
cannot mark a demand maximally urgent). `tests/test_bypass_cap.cpp` case 7 and
`verify_rdu_tree.py` both check this.

## Policy constants worth knowing

| | |
|---|---|
| `ALLOW_DEGRADED_COMMIT 0` | a lone surviving node may not COMMIT — breaking one node must not be an *upgrade* in what one node can cause |
| `ALLOW_BYPASSED_COMMIT 0` | **v7.** Same reasoning applied to a bypassed geofence. See root README §7.1 |
| `COMMIT_MIN_CONFIDENCE 0.70` | second, independent gate so raising node sensitivity cannot silently raise commitment rates |
| `HEARTBEAT_TIMEOUT_MS 60000` | paired with `NODE_FAILED_MS` in `ICU_Geometry.ino` and with the RDU's `EXPECTED_ICU_HB_TIMEOUT_MS`. Cross-checked by the verifier |

## Before reflashing

The key set is one unit. Regenerating node keys means reflashing the ICU too —
its public-key table and the six private keys cannot be updated separately.

Order matters in `GW_RDU_PUBLIC_KEYS`: the firmware indexes it arithmetically as
`(lane-1)*2 + (node-1)`. Reordering the entries silently swaps two nodes'
identities, and the only symptom is `BAD TAG`.
