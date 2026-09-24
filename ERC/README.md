# ERC — Emergency Response Console

Operator display. Connected to the ICU over UART. Shows the active alert, its
stage, evidence class and ETA.

## Why it has its own timeout

The ERC enforces an alert ceiling independently of the ICU's `MAX_ALERT_MS`.

Two limits on purpose, covering different failures: the console's backstop
covers an **ICU that has hung**; the ICU's covers a **decision loop that is
running but stuck in a state it cannot leave**. Neither covers both, so neither
is redundant. Do not remove one because the other exists.

## Files

| | |
|---|---|
| `ERC.ino` | display and backstop timer |
| `GreenwaveLink.h` | stage and evidence enums, shared with the ICU |
| `greenwave_logo.h` | splash bitmap |

`GreenwaveLink.h` defines `LS_*` (stage) and `LE_*` (evidence class). Both are
also read by `ICU_Decision.ino`. Changing either enum's numbering requires
reflashing both the ICU and the console — the values cross the UART as integers.
