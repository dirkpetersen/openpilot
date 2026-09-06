# MADS2PNW — "lateral survives a brake press"

**Branches:** `mads2pnw` in `dirkpetersen/pnw-pilot` and in `dirkpetersen/pnw-opendbc`.
**Status: BUILT, REVIEWED, INERT.** It does nothing on the car until the panda is reflashed
*and* the openpilot-side engagement work below lands. Do not merge to `3devpnw`/`3testpnw` yet.

## The problem

The F-150 Lightning runs **stock ACC**. Speed is the truck's; openpilot only steers. So a brake
tap disengages stock cruise, openpilot disengages with it, and the driver loses **everything** —
including the one thing openpilot was actually doing. On the Tesla Raven this doesn't arise the
same way (openpilot owns longitudinal there, and the Raven's EPS self-inhibits via
`EAC_INHIBITED` on a brake press anyway, so this feature would not help it).

## The design, and why it is not a weakening

Ported from sunnypilot's **MADS** (via `sunny/bluepilot vin-lightning-2024-25`,
`opendbc/safety/sunnypilot/mads.h`). The trick is that upstream did **not** touch the brake check.
They added a **second, parallel authority flag** next to it:

* `generic_rx_checks()` still clears `controls_allowed` on the rising edge of the brake pedal, in
  every path, for every car. **Untouched.**
* A new `controls_allowed_lateral` is consulted **only** by lateral tx gates, as
  `(controls_allowed || controls_allowed_lateral)`.
* Longitudinal, gas, resume-button, relay: all still read `controls_allowed` alone.

So a brake press still removes longitudinal authority; it just no longer removes lateral authority.

### Opt-in chain (all four must hold)

1. `PnwVehicle.mads_lateral` — capability view, today the Lightning only. Never a fingerprint test
   in feature code.
2. `PandaMadsSafety` — an explicit declaration that the **currently flashed** panda carries this
   safety build (see *Honest capability detection*).
3. `set_safety_hooks()` refuses the MADS bits in **every safety mode except `SAFETY_FORD`** — the
   panda does not trust the host's capability gate.
4. `m_update_control_state()` gates the write on `system_enabled`, so a build with MADS disabled
   can never flip the global on.

`mads_set_system_state()` re-inits the whole struct on every safety-mode change, so
`controls_allowed_lateral` is cleared on every mode change — including the panda's drop to
`SAFETY_SILENT` when the heartbeat is lost.

### The toggle — inverted polarity, deliberately

**"Disengage on brake"**, param `DisengageOnBrake`, default `"0"`. Same idiom as
`DisableLaneCentering` / `NoFordAngleSteering`:

| Toggle | Param | MADS mode | Behaviour |
|---|---|---|---|
| **OFF (default)** | `"0"` | REMAIN_ACTIVE | brake takes speed, **leaves steering** |
| ON | `"1"` | DISENGAGE | stock: brake takes everything |

The param ships `"0"` (satisfying this fork's default-OFF rule) and `"0"` is *also* the new
behaviour. **That is intentional, not a slip.** `PAUSE` exists in the safety code and is not
exposed.

The toggle is **greyed and painted ON (stock)** whenever the car lacks the capability *or*
`PandaMadsSafety` is `0`. The grey-out is display-only (no `put_bool` — this is ONE physical
device moved between two cars); the real forcing is in `card.py`, which sends
`alternativeExperience = 0` whenever either condition fails.

`needs_restart` is `True` on the toggle, so flipping it requests an onroad cycle. The panda only
latches the MADS bits at safety-mode init — without the cycle the flip would silently do nothing.

## What changed

### `pnw-opendbc` (branch `mads2pnw`)
| File | Change |
|---|---|
| `opendbc/safety/pnw/mads_declarations.h` | new — state machine types, `ALT_EXP_*` defines, and the full rationale + deviations |
| `opendbc/safety/pnw/mads.h` | new — the state machine |
| `opendbc/safety/lateral.h` | 9 lateral gates → `(controls_allowed \|\| controls_allowed_lateral)` |
| `opendbc/safety/safety.h` | `mads_state_update()` once per rx after `generic_rx_checks()`; `mads_exit_controls(LAG)` on lag and on an invalid rx message; MADS bits applied at `set_safety_hooks`, Ford-only |
| `opendbc/safety/modes/ford.h` | the two angle-mode disengaged-steering guards, the angle-mode corroboration gate and `angle_mode_active` → lateral authority; `acc_main_on` rx write ported |
| `opendbc/safety/__init__.py` | `ALTERNATIVE_EXPERIENCE.ENABLE_MADS / MADS_DISENGAGE_LATERAL_ON_BRAKE / MADS_PAUSE_LATERAL_ON_BRAKE` |
| tests | `mads_common.py` mixin + Ford/Toyota/Tesla tests, libsafety hooks, `mutation.py` pin re-base |

### `pnw-pilot` (branch `mads2pnw`)
| File | Change |
|---|---|
| `selfdrive/car/card.py` | `alternativeExperience` was **hardcoded to 0**; now computed by `_alternative_experience()` |
| `selfdrive/controls/lib/pnw_vehicle.py` | new `mads_lateral` capability |
| `common/params_keys.h` | `DisengageOnBrake`, `PandaMadsSafety` (both default `"0"`) |
| `selfdrive/ui/layouts/settings/toggles.py` | the "Disengage on brake" toggle + its capability clamp |
| `selfdrive/car/tests/test_mads_alternative_experience.py` | new |

### Deliberate deviations from upstream
* **No MADS button, no ACC-main-rising engage.** The *only* thing that ever sets
  `controls_allowed_lateral` is the rising edge of openpilot's own `controls_allowed`.
* **Added beyond upstream:** openpilot losing controls for any **non-brake** reason (CANCEL, a
  fault, anything) also drops lateral. Upstream can leave the latch standing because it has an
  openpilot-side MADS state machine and the `heartbeat_engaged_mads` watchdog; this tree has
  neither yet, so without this a CANCEL press would leave the panda permitting lateral forever.
* **`acc_main_on` rx write ported** (bp-7.0 line the angle2pnw port had skipped) so that turning
  ACC MAIN off is a real, driver-reachable revoke. A brake press drops `CcStat_D_Actl` 4/5 → 3
  (standby, main still on) and correctly does **not** revoke.
* **Also clears lateral on an invalid rx message** (`is_msg_valid`), where upstream clears only
  `controls_allowed`. Strictly narrower.
* **The Ford reset-bypass latch keeps its plain `controls_allowed` gate.** That latch grants
  *amnesty from rate-of-change checks*; it does not select limits. Extending it to the MADS flag
  would re-open the 2026-07-11 hole (openpilot streams neutral frames while longitudinally
  disengaged, which would keep the amnesty permanently armed). MADS therefore runs with the ROC
  checks fully enforced, every frame.
* **`mads_state_update()` placement.** Called once per rx message from `safety_rx_hook`, right
  after `generic_rx_checks()`. bluepilot calls it from inside `stock_ecu_check()`, which runs once
  per relay-checked tx_msg — a variable number of times per frame, which is wrong for an edge
  detector.
* **`mads_set_alternative_experience()` is actually called.** In bluepilot it is invoked only from
  the test harness, so MADS there is dead code in real firmware.
* **`heartbeat_engaged_mads` NOT ported.** See below — this is a hard blocker for flashing.

## Honest capability detection — and why it is a param

The panda stores `alternative_experience` verbatim and echoes it back regardless of whether its
safety build understands the MADS bits, so **the echo cannot distinguish a MADS panda from a stock
one**. There is no version, safety-param or health field in this tree that can either. The honest
signal would be a new `controls_allowed_lateral` field in the panda health packet →
`PandaState` (as sunnypilot does), which is itself a panda-firmware change and therefore only
available *after* the flash.

So `PandaMadsSafety` is an **explicit, conservative, default-OFF declaration** set by hand as the
last step of the flash procedure. It is not inferred from anything. Until it is set, `card.py`
sends `alternativeExperience = 0` — the stock contract — and the toggle is greyed to stock.

**Note the half-state hazard does not arise in this port**: the greyed `NoDisengageOnBrake` toggle
exists because suppressing openpilot's *own* disengage against a panda that still clears
`controls_allowed` produces "UI says engaged, car does nothing", ending in `controlsMismatch`
(`ET.IMMEDIATE_DISABLE`) after 2 s. **This port does not suppress openpilot's disengage at all**
(see below), so that state is unreachable. `PandaMadsSafety` guards the converse — sending MADS
bits to a panda that cannot honour them.

## ⚠️ What is NOT done — read before planning the flash

### 1. The openpilot-side lateral engagement state machine is NOT ported
sunnypilot's `sunnypilot/mads/mads.py` + `state.py` + its custom events and UI are **not** in this
branch. Consequence: when the Ford PCM drops cruise on the brake press, openpilot still
disengages, `selfdriveState.active` goes False, `controlsd` sets `CC.latActive = False`, and
**openpilot stops sending steering commands**. The panda's new permission goes unused.

**So flashing the panda alone will not produce the behaviour the owner asked for.** That work is
a separate, larger effort: a lateral-engagement state distinct from `enabled`, its own alerts, and
a UI that never shows "disengaged" while the truck is steering itself. Do not shortcut it by
latching `CC.latActive` in `controlsd` — that would steer while the UI says off.

### 2. No lateral watchdog (`heartbeat_engaged_mads`) — HARD BLOCKER for flashing
`panda/board/main.c` has a 3-second watchdog for `controls_allowed`:

```c
if (controls_allowed && !heartbeat_engaged) {
  heartbeat_engaged_mismatches += 1U;
  if (heartbeat_engaged_mismatches >= 3U) { controls_allowed = false; }
} else { heartbeat_engaged_mismatches = 0U; }
```

There is **no lateral equivalent**. Upstream's is fed by `heartbeat_engaged_mads`, which the panda
receives in heartbeat cmd `0xf3` **param2** — plumbing this tree's panda does not have. Before any
flash, add to `pnw-panda` (`master-pnw` → a `mads2pnw` branch):

```c
/* board/main_comms.h, case 0xf3: */   heartbeat_engaged_mads = (req->param2 == 1U);
/* board/main.c, in the 1 Hz block:  */ mads_heartbeat_engaged_check();
```

plus `heartbeat_engaged_mads` / `heartbeat_engaged_mads_mismatches` globals and
`mads_heartbeat_engaged_check()` restored in `opendbc/safety/pnw/mads.h` (it was removed here
rather than shipped as unused code that trips MISRA 8.7 — the exact upstream body is in
`sunny/bluepilot/opendbc_repo/opendbc/safety/sunnypilot/mads.h`). pandad must then send a real
"openpilot still wants lateral" flag, which only exists once item 1 lands. **The two are coupled:
neither the watchdog nor the flash is safe without the openpilot-side state.**

### 3. Latch state is invisible
Nothing publishes `controls_allowed_lateral` — no health field, no `PandaState`, no `ces_events`.
Add it with item 2 so `selfdrived` can cross-check it and telemetry can see it.

## The panda flash procedure (owner's call, with the cars present — NOT done here)

**Do not flash before items 1 and 2 above are done.** When they are:

1. **Repos and the matched set.** The panda firmware is built from **`pnw-panda`** together with
   **`pnw-opendbc`** (the safety C lives in opendbc, the board code in panda). They are a
   *matched set*: `panda/board/` and `opendbc/safety/` share the `CANPacket_t` layout, and panda
   embeds `CAN_PACKET_VERSION_HASH` computed over it. A panda flashed from one pair and driven by
   an openpilot pinned to a different pair will refuse to talk (or, worse, mis-frame). So:
   * push `pnw-opendbc mads2pnw` and `pnw-panda mads2pnw`,
   * bump **both** submodule pins in the same `pnw-pilot` commit,
   * build panda from *that* checkout's submodules only — never mix a locally built panda with a
     differently pinned opendbc.
2. **Build + flash** from the device (this fork's normal path), with the car **disengaged**:
   `cd /data/openpilot/panda && scons -u -j4 && python board/flash.py`. Then verify
   `pandaState.pandaType`, no `relayMalfunction`, and that the panda's reported
   `alternativeExperience` matches `CarParams.alternativeExperience` (`selfdrived.py` raises
   `controlsMismatch` if not).
3. **Only then** set `PandaMadsSafety=1` and reboot; the "Disengage on brake" toggle un-greys.
4. **Rollback** = re-pin the submodules to the previous SHAs (`951c7725` opendbc / `56920ec6`
   panda as of this writing), rebuild, reflash, and set `PandaMadsSafety=0`. Keep a known-good
   `panda.bin.signed` on the device first — `pandad` reflashes to whatever `FW_PATH` holds, so a
   half-flashed panda recovers by putting the old signed binary back and rebooting. The dual-panda
   flash blocker documented in `XNOR2BP.md` is the reason this is the owner's call and not a
   background task.

## Test evidence

* Full panda safety suite (all cars, serial): **3067 passed**, 0 failed. Under xdist the Ford file
  is **flaky at the base commit too** (shared C statics in a process-wide singleton); run it
  serially (`-o addopts=""`) for a trustworthy result. A `tearDown` was added to stop the MADS
  tests contributing to that.
* MISRA/cppcheck: **43 violations, identical to the base commit** — zero new.
* `ruff`: identical to the base commit.
* openpilot side: 10 tests in `test_mads_alternative_experience.py`.
* **Mutation testing: 22 mutations, 0 survivors** (see the branch's commit message for the list),
  including two anti-weakening mutants — removing the brake clear of `controls_allowed`, and
  making the reset latch ignore authority entirely — both killed loudly.
