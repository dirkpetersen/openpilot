# ICBMCURV2PNW — measuring the map polyline on the ICBM path

**Status: STEP 1 SHIPPED (telemetry only, nothing reads it for control). STEP 2 PROPOSED, NOT
ENABLED, NOT MERGED.**

Branch `icbmcurv2pnw`, based on `3devpnw` `b148c4768c`.

---

## 1. The defect

ICBM is the stock-ACC speed controller on the Lightning: it taps the SET− cruise button to walk the
truck's own set speed down. Twice in 18 hours it commanded a large unrequested slowdown on straight
freeway.

| | 2026-09-05 19:48:14 PT | 2026-09-06 13:15:09 PT |
|---|---|---|
| road | I-5, posted 70 mph | same signature |
| `icbmT` | 17.77 m/s (39.7 mph) | 18.24 m/s (40.8 mph) |
| `icbmSrc` / `icbmDir` | `map` / `dec`, held 15 s | `map` / `dec` |
| mapd's claim | — | `mapV` 20.3 m/s at `mapDist` counting 210 m → 8 m |
| posted limit | correct throughout | `spdLim` 31.3 m/s (70 mph), correct throughout |
| curve evidence | none anywhere in telemetry | `mapK` 0.0, `apexCurvature` 0.0, `curveWin` "none", `vtscState` "idle" |
| effect | stock ACC set 77 → 58 mph; truck 71.6 → 62.9 mph | same class |

Not a lead: one sat 55 m ahead at matching speed and the gap **grew** as we slowed.

**mapd reported a curve target velocity where there is no curve, and ICBM faithfully executed it.**

## 2. Why nothing contradicted it

mapd publishes both a per-point *velocity* and the *lat/lon polyline* those velocities belong to.
The velocity is derived from OSM node geometry by mapd's own curvature calculation and is known to be
noisy — see the `icbmonset` note in `ces_pnw.py` (raw reads of 46.5–128.6 m/s at curve entry), and
[[mapd-velocity-vs-polyline]]: *mapd's velocity can't separate a real curve from an artifact; measure
curvature from the lat/lon polyline in the same message.*

That polyline measurement **already exists** — `polyline_curvature()` in
`selfdrive/controls/lib/vtsc_pnw/vtsc_pnw.py`, a pure, never-raising Menger-curvature scan with a
node-spacing gate. `vtsc_controller.py` calls it, and its own guard comment says the measurement
"exists only with CESMode>0 AND op-long AND VtscMapCurves=1".

**ICBM exists precisely because this truck runs STOCK ACC — `openpilotLongitudinalControl` is False
by definition on the Lightning — so `cap()` early-returns and `polyline_curvature()` never runs
here.** Live proof from a `ces_events` tick pulled 2026-09-06 while driving:

```
"mapPts": 8, "mapReach": 414.0,      <- mapd delivered a 414 m polyline, 8 points (~59 m spacing)
"mapK": 0.0, "mapKN": 0,             <- ...and ZERO of it was measured
"shadow": true, "car": "FORD_F_150_LIGHTNING_MK1"
```

(that same tick carried `"mapV": 49.5` at 218 m — 110 mph — which is the noise floor of the field
ICBM trusts.)

## 3. `mapKN == 0` means UNMEASURABLE, NOT STRAIGHT

This is the trap, and it is why step 1 is not simply a clamp.

`polyline_curvature()` returns `k == 0.0` for **two different roads**: one that is genuinely straight,
and one whose geometry could not be measured (fewer than 3 in-horizon points, or node spacing outside
the `[25 m, 300 m]` jitter/locality gate — a real R=1500 m curve with 366 m node gaps returns exactly
0.0). `n_ok` — the count of triplets that passed the spacing gate — is the only thing that separates
them, which is why the function returns it at all.

**A clamp gated on `icbmK ≈ 0` would therefore fire on every Lightning tick today** (where nothing is
measured at all) **and suppress every map slowdown, including real curves. DO NOT BUILD THAT.**
Anything downstream must read `icbmKN` first and treat `icbmKN == 0` as "no opinion", never as
"straight".

## 4. What step 1 changes

`selfdrive/controls/lib/ces_pnw/ces_pnw.py` only. No new params, no toggle, no control path.

* `_read_map()` (~1 Hz) now runs the same `polyline_curvature()` on the same cached
  `MapTargetVelocities` + `LastGPSPosition` that ICBM's own map candidate scan uses, with
  `MAP_SOURCE_HORIZON_M` (mapd's 500 m cap) and `A_LAT_TARGET` (2.5, the value VTSC's `mapKV` uses in
  both the DEFAULT and GENTLE profiles, so `icbmKV` and `mapKV` are directly comparable).
* Five new fields, on **every** `ces_events` record (both cars), and — because that block sits under
  `if self._shadow:` alongside the other `icbm*` overlay keys — on the 5 Hz `CESStatus` overlay feed
  **on the Lightning only**. On the Tesla they reach `ces_events` and nothing else:

  | field | meaning |
  |---|---|
  | `icbmK` | measured curvature, 1/m, 6 dp (0.0 = straight **or** unmeasurable — read `icbmKN`) |
  | `icbmKD` | distance to that point, m |
  | `icbmKV` | `sqrt(2.5 / icbmK)` m/s. **`0.0` means NO FINITE BOUND (straight) *or* unmeasurable — never "slow"**: `v_safe` is `inf` there and `inf` is not valid JSON, so it is encoded as `0.0`. Any consumer comparing `icbmKV` against a speed fails every `>=` test on the *straightest possible road*, which is exactly backwards. Use `icbmK` (curvature space) for comparisons; `icbmKV` is for reading |
  | `icbmKN` | triplets that passed the spacing gate. **0 = no measurement was possible** |
  | `icbmKAhead` | the measured point is in front of the car (mapd publishes nodes behind us too) |

* The reset of those five sits at the **top** of `_read_map()`, above the `mem_params is None` early
  return, so they always describe the polyline cached on *this* refresh or nothing at all. Resetting
  them next to the measurement would let a stale reading survive a blind refresh and present as live —
  the exact failure class this change exists to remove.
* **Deliberately not gated to the Lightning.** No fingerprint branches in feature code (capability-view
  rule), and on the Tesla — where VTSC measures the same polyline with the same `A_LAT_TARGET` —
  `icbmK` and `mapK` on the same tick are a free correctness check on this wiring, from real drive
  data, at zero risk.

### Known limitations, stated up front

**(a) `icbmKN == 0` is a legitimate answer on real roads.** Ways whose OSM nodes are >300 m apart, or
a horizon holding fewer than 3 usable points, produce no measurement. That is not a bug and it is not
"straight". Whether the Lightning's `icbmKN` is usually non-zero is an **empirical question this
change is designed to answer**; the 2026-09-06 tick above (414 m / 8 points ≈ 59 m spacing) suggests
yes, but one tick is not a dataset.

**(b) The bridge thins the polyline hardest on straight roads.** `system/mapd/mapd_configd.py` drops
every path point whose `targetVelocity` is non-finite, and upstream mapd's Heron-formula curvature
returns NaN on **near-collinear** OSM nodes — i.e. on straights. So `MapTargetVelocities` is already
sparsest exactly where the phantom slowdowns happen, and the surviving legs can exceed the 300 m
gate. `icbmKN == 0` on straight freeway may therefore be entirely correct and common. **This is the
main threat to step 2's coverage**, and measuring it is a prerequisite, not an afterthought.

**(c) Mixed node spacing can produce a MEASURED ZERO with a real corner in the horizon.** Reproduced
against the unchanged `polyline_curvature`: a dense straight, a genuine 90° corner drawn with two
350 m legs, then a dense straight, returns `k = 0.0` with `n_ok = 2`. The straight triplets pass the
spacing gate; the corner's are rejected. So "a measurement exists AND it says straight" is **not**
the same claim as "the road is straight", and a naive veto keyed on it would suppress a real
slowdown. Pinned by `test_a_coarse_corner_among_dense_nodes_is_a_MEASURED_ZERO`. Step 2 must carry a
coverage guard for this; see §5.

### Verifying it on the car (no drive needed)

```bash
ssh comma@$COMMA_IP "python3 -c \"import json;print({k:v for k,v in json.load(open('/dev/shm/params/d/CESStatus')).items() if k.startswith('icbmK') or k=='mapPts'})\""
```
Lightning only (the overlay block is shadow-gated); on the Tesla read `ces_events` instead.
`mapPts > 0` with `icbmKN == 0` is **not by itself** evidence of broken wiring — see limitations (a)
and (b): the legs may simply be too long or too few. It is evidence of broken wiring only on a road
you have confirmed has 25–300 m node spacing in the horizon. To settle it, dump `MapTargetVelocities`
itself and measure the leg lengths.

---

## 5. PROPOSED — step 2, a credibility check (NOT ENABLED, NOT MERGED)

**Nothing below is implemented, and as written it is NOT implementable from the fields this branch
adds.** It must not be enabled until the step-1 telemetry from at least one full I-5 drive shows
(a) `icbmKN > 0` for a usable fraction of ticks, (b) `icbmK` agreeing with `mapK` on the Tesla where
both are measured, and (c) a workable `coverage_ok` condition — which needs either a new field in
`polyline_curvature()` or offline analysis of the raw polylines. See the `coverage_ok` subsection.

### The rule

**Formulated in CURVATURE space, not speed space.** The obvious form —
`icbmKV >= icbmT * RATIO` — is broken twice over: `icbmKV` is `0.0` on a measured-straight road (see
the `icbmKV` row in §4), so the veto would silently fail to fire on the *cleanest* phantom there is;
and it has a singularity at `k -> 0`. The equivalent test with no singularity and the right behaviour
at zero is:

```
suspect = (icbm_src == "map")                      # map-sourced targets only
          and (icbmKN >= N_MIN)                    # a measurement EXISTS. 0 => no opinion, never veto
          and icbmKAhead                           # the geometry is in front of us
          and coverage_ok                          # see below -- NOT satisfiable from step-1 fields
          and (icbmK * (icbmT * RATIO) ** 2 <= A_LAT)
```

The last line reads: *"even at RATIO times the speed mapd wants us down to, the measured curvature
would still hold lateral accel under A_LAT (2.5 m/s²)"* — i.e. the geometry does not justify anything
like this slowdown. At `icbmK == 0.0` it is trivially true, which is the correct answer for a road
measured straight.

Starting points for tuning **against replay only**:

* `N_MIN = 2` — one triplet is a single node's jitter; two independent ones is a road.
* `RATIO = 1.6` — the geometry must be comfortable at 60 % more speed than the target being
  commanded. A genuine curve has `icbmK ≈ A_LAT / icbmT²` and is nowhere near it.

### `coverage_ok` — the guard that does NOT exist yet, and why this is blocked

`icbmKN >= N_MIN` is **not** sufficient. Limitation (c) in §4 is a reproduced counterexample: a real
90° corner drawn with two 350 m legs, flanked by dense straight nodes, reports `icbmKN = 2` and
`icbmK = 0.0`. The rule above would fire on it and veto a genuine slowdown — the exact failure
direction that matters, because a suppressed real slowdown is worse than the over-slow being fixed.

What is needed is a statement that the measurement actually **covered the part of the polyline the
map target came from**, e.g.:

* the point carrying the map target (at `mapDist`) lies inside a triplet that passed the spacing
  gate, **or**
* no in-horizon leg was rejected by the spacing gate at all.

**Neither is computable from the step-1 fields.** `polyline_curvature()` does not report rejected
triplets or leg statistics, and adding an `icbmKR` (in-horizon triplets rejected) would mean changing
the shared function VTSC calls at 20 Hz — deliberately out of scope for a telemetry-only step. So:
**step 2 is blocked on either that field or on offline analysis of raw `MapTargetVelocities`
polylines, and must not be implemented from the fields this branch adds.**

### What it would be allowed to do

**Only refuse to START a new map-sourced cap episode**, and only that:

* It must **not** cancel an episode already in progress — a mid-curve un-cap, via ICBM's restore
  ("inc") taps raising the stock set speed, is a worse failure than the over-slow it would prevent.
  The accepted consequence: **a phantom that begins before a measured refresh runs its full ~15 s.**
  That is tolerable only because the veto is evaluated on the first decision tick, which is
  consistent by construction — ICBM's own map scan and this measurement read the same
  `_map_targets` cache filled by the same `_read_map()` call.
* It must **not** touch a vision-sourced (`vis`) or far-map target, or the `restore` path.
* It must **not** raise a speed, extend a restore, or alter any tap: ICBM stays dec-only against the
  driver's own set speed.
* It must set an `icbmGate` value (e.g. `"kVeto"`) so a suppressed slowdown is **visible in
  `ces_events`**, never silent. A suppression that leaves no trace is the same defect in the other
  direction.

### The failure mode to fear

A **false veto on a real curve**. `coverage_ok` is the primary guard and it does not exist yet;
`icbmKN >= N_MIN` and the generous `RATIO` are secondary and, on their own, demonstrably
insufficient (§4 limitation (c)). All three must be validated against replay of the sharp-curve
reference cases in `SHARPCURVE2PNW.md` / `CES_I90.md` — specifically the I-90 Snoqualmie and I-5
Woodland curves — showing **zero** vetoes there, before this goes near a car.

### Suggested order of work

1. Ship step 1 (this branch). Drive I-5 and I-90.
2. Offline, from the drive logs: (i) per-drive `icbmKN > 0` coverage — limitation (b) says this may
   be poor exactly on straight freeway; (ii) on the Tesla, `abs(icbmK - mapK)` on ticks where both
   are measured, as the correctness check on this wiring; (iii) leg-length statistics of the raw
   `MapTargetVelocities` polylines, which is what says whether `coverage_ok` is achievable at all.
3. Only if (i) and (iii) come back usable: decide whether `coverage_ok` needs an `icbmKR` field in
   `polyline_curvature()` (a change to the shared VTSC function, reviewed on its own).
4. Only then implement step 2 as a **shadow field** (`icbmKVeto: true/false` in `ces_events`, gating
   nothing) and confirm on replay it fires on the two phantom events and on nothing else — including
   zero fires on the Snoqualmie and Woodland reference curves.
5. Only then let it gate an episode start, behind its own default-OFF param.

---

## References

* `selfdrive/controls/lib/vtsc_pnw/vtsc_pnw.py` — `polyline_curvature()` (pure, never raises)
* `selfdrive/controls/lib/vtsc_pnw/vtsc_controller.py` — the `mapcurv2pnw` call site + its op-long guard
* `docs/pnw/ICBM2PNW.md` — the ICBM design (SET− tap executor, dec-only envelope)
* `docs/pnw/MAPD-SYSTEM.md` — the mapd bridge and `MapTargetVelocities`
* Tests: `selfdrive/controls/lib/ces_pnw/tests/test_icbmcurv.py` (measurement + staleness + JSON
  safety + overlay), `test_ces_record_fields.py::TestIcbmCurvatureTelemetry` (last mile into
  `ces_events`)
