"""
madsresume2pnw — bounded auto-resume of the driver's OWN cruise set speed after a MADS brake.

WHAT THIS IS
------------
On the F-150 Lightning openpilot does not own longitudinal: speed is the truck's stock ACC.
mads2pnw/madsop2pnw taught the panda and openpilot to keep STEERING alive across a brake press
("Steering only", `madsState.lateralOnly`), but the driver still has to reach for the cruise stalk
to get speed back. This module decides — and it only ever DECIDES; the press itself is executed in
`opendbc/car/ford/icbm_pnw.py` — whether openpilot may tap RESUME once on the driver's behalf.

THIS IS SELF-ENGAGEMENT AND IT IS TREATED AS SUCH
-------------------------------------------------
Lateral authority (held by MADS) is being used to unlock a longitudinal re-engagement openpilot
did not otherwise have. The owner accepted that objection explicitly, on two conditions that are
the axioms of everything below:

  1. Resume ONLY to the speed the driver ALREADY SET. Never higher, never a new speed.
  2. The brake is always under the driver's foot, so they can always take it back.

Condition 1 deserves an honest note, because the mechanism does not let us be more precise than
this: the button we send is Ford's RESUME (`CcAsllButtnResPress` on 0x083). **The PCM chooses the
speed, not openpilot** — Ford ACC resume returns to the PCM's own last set speed. We therefore
cannot *command* a target at all. What we CAN do, and what this module does, is:
  * positively observe the driver's set speed BEFORE the brake and refuse to press at all if we
    never saw one (gate `noSet`);
  * refuse if the truck's currently-reported set speed is ABOVE the one we observed (`setRaised`);
  * refuse to press while stock cruise is ENGAGED, where RES is a SET+ (+1 mph) — enforced again,
    independently, in the executor (`decide_resume`); and
  * VERIFY after the fact: once cruise comes back, compare the truck's set speed against the one
    we captured and emit a LOUD record + `cloudlog.error` if it came back higher (`setHigher`).
That last one is the honest answer to "can the PCM resume to something we didn't expect?" — we
cannot prevent it, so we make it impossible to miss in the log. See docs/pnw/MADSRESUME2PNW.md.

NOTHING FAILS SILENTLY
----------------------
Every arm ends in exactly ONE terminal record ("fire" or "refuse"), and every refusal names the
gate that bound. A missing gate input is a REFUSAL with its own reason (`leadUnknown`, `noSet`) —
never a permissive default. A silent no-resume and a silent wrong-resume are therefore distinct
in ces_events.jsonl: no record at all means this module never even armed.

PURITY
------
No params, no sockets, no clock, no cereal. Everything arrives in `ResumeInputs`; `now` is passed
in (monotonic seconds). That is what makes the whole gate matrix unit-testable, and it is why the
call site (selfdrive/selfdrived/selfdrived.py) does the I/O.
"""

import math
from dataclasses import dataclass, field

# ---------------------------------------------------------------------------------------------
# Thresholds. Every one of these is justified in docs/pnw/MADSRESUME2PNW.md; the short form is in
# the comment beside it. They are deliberately module constants (not JSON-tunable): this is a
# self-engagement envelope, not a ride-feel tune.
# ---------------------------------------------------------------------------------------------

# Gate 3 — the bounded window, measured from the moment the brake is FULLY released.
# MIN: the truck's ACC state (CcStat_D_Actl 4/5 -> 3) and the driver's foot both need to settle;
#      firing on the same tick as the release would also fire on a brake *bounce*.
# MAX: 3 s. At highway speed that is ~90 m — the traffic situation that caused the brake is still
#      the same situation. Past it the driver has demonstrably chosen not to resume, and a resume
#      would be a surprise rather than a completion of what they were doing.
RELEASE_MIN_S = 0.5
RELEASE_MAX_S = 3.0

# How long the offer stays on the wire once every gate has passed. The executor polls the
# mem-param at 4 Hz and its own freshness limit is 0.5 s, so 1.0 s is comfortably enough for
# exactly one poll+press while keeping the total exposure short. Re-published every tick and
# WITHDRAWN the instant any gate stops holding.
OFFER_S = 1.0

# Gate 6 — the captured set speed.
# The capture is refreshed on EVERY tick stock cruise reports enabled, and is REMEMBERED across the
# cruise-off gap that the brake itself creates. That memory is the whole point: the driver's set
# speed does not stop existing because their foot touched the brake pedal.
#
# It used to expire 0.75 s after the last engaged frame, sized to cover only MADS's brake-grace
# window. That was wrong, and the 2026-09-06 truck drive shows exactly how (drives/2026-09-06/):
# three brake episodes, and the capture was valid for only the FIRST one. Once a resume misses for
# any reason, stock cruise stays off, so nothing refreshes the capture -- and every later brake for
# the rest of the drive refused with `noSet` (observed setAgeS 22.15 s, then -1.0 = never captured).
# The feature could fail exactly once and was then dead until the driver manually re-engaged cruise,
# which is the thing the driver was asking not to have to do.
#
# Why lengthening this does NOT reopen the hazard it was sized against: the specific attack -- driver
# cancels cruise on the stalk, then brakes a beat later, and we resume to the speed they just
# deliberately cancelled -- is blocked ONE LAYER UP and always was. A stalk CANCEL is not in
# mads_pnw.MADS_TOLERATED_EVENTS, so `blocked` is True at the falling edge and MADS refuses to hand
# this module a lateral-only episode at all. The 0.75 s bound was belt-and-braces on top of a gate
# that already holds; removing the braces does not remove the belt.
#
# What remains is a genuine, bounded residual: a set speed captured on a fast road, remembered while
# the driver steers a long way on MADS lateral without cruise, and resumed somewhere it no longer
# suits. Three things bound it -- this 10-minute backstop, clearing the memory the moment the ACC
# master goes off (`cruise_available` False, i.e. the driver switched cruise off entirely), and the
# fact that a resume can never target ABOVE what the driver themselves last set.
SET_MAX_AGE_S = 600.0
# Sanity floor on a CAPTURED set speed -- it rejects a decode fault, it does not impose policy.
#
# This used to be 20 mph, justified as "Ford's ACC will not hold a set speed below 20 mph". That is
# FALSE for this truck, and it was silently refusing the driver's real city speeds. Two independent
# checks, 2026-09-06:
#   * opendbc/car/ford/interface.py:105 sets minEnableSpeed = 20 mph only on the MANUAL-transmission
#     branch. The Lightning is automatic, so that limit never applied to it.
#   * measured on the truck's own log: stock ACC was ENGAGED with set speeds of 15, 16, 17, 18 and
#     19 mph (34 ticks, minimum 6.71 m/s). The car plainly holds a set speed below 20 mph.
# 5.0 m/s (~11 mph) sits below every engaged set speed actually observed, with margin.
SET_MIN_MS = 5.0
# Sanity ceiling — a reading above this is a decode fault, not a driver intent.
SET_MAX_MS = 45.0                    # ~100 mph
# One Ford SET tap is 1 mph; tolerate under half of one so our own comparison can never trip on
# quantisation, while a real driver/PCM change of a full step still trips it.
SET_TOL_MS = 0.4 * 0.44704

# Gate 5 — lead / TTC. Resuming hands speed back to stock ACC, which will then ACCELERATE toward
# the set speed. So the bar is not "is this safe right now" but "is the road already at least as
# open as the gap ACC itself would hold".
#   HEADWAY 2.0 s  — at or better than stock ACC's own following distance, so re-engaging cannot
#                    ask ACC to close a gap it would not otherwise close.
#   TTC     8.0 s  — at 30 m/s behind a 60 m lead, 8 s TTC means closing at under 7.5 m/s: we are
#                    not meaningfully overtaking. Anything faster-closing is exactly the situation
#                    the driver braked for.
#   DIST   20.0 m  — an absolute floor, because headway alone is far too permissive at low speed
#                    (2 s at 5 m/s is 10 m).
LEAD_MIN_HEADWAY_S = 2.0
LEAD_MIN_TTC_S = 8.0
LEAD_MIN_DIST_M = 20.0

# Speed floor for actually resuming. Kept equal to the capture floor rather than DERIVED from it,
# so that changing one does not silently move the other. Below ~11 mph a brake-and-release is
# creeping in stop-and-go, where handing speed back to ACC is a surprise; `standstill` is gated
# separately. Note this is a POLICY floor -- the lead gate (20 m absolute) is what actually protects
# the low-speed case, and it is unchanged.
V_EGO_MIN_MS = 5.0

# brakeretry2pnw: the driver's OPT-OUT. Two brake presses inside this window mean "no, leave
# longitudinal off" -- auto-resume is then suppressed until the driver themselves brings cruise back
# (a manual RES, or any SET+/SET- adjustment; both engage stock cruise, which is the clear signal).
#
# This exists BECAUSE arming now happens on every brake press. Without it the driver has no way to
# say "stay off": each press would open another resume opportunity, and a driver who genuinely wants
# cruise gone would be arguing with the feature. One deliberate double-tap is a clearer, faster
# statement of intent than any toggle, and it is available in the moment, with the foot already
# there. A single press keeps its plain meaning ("slow down, then carry on"), which is the common
# case, so the opt-out costs the common case nothing.
DOUBLE_BRAKE_S = 1.0
# Pedal BOUNCE filter. Two edges closer together than this are one press that chattered, not two
# presses -- counting them as a double-tap would silently kill the feature on a rough road, and
# counting them as two arms would re-arm on chatter (Gemini review 2026-09-06, finding C).
BRAKE_DEBOUNCE_S = 0.15
# A brake press this soon after WE resumed is the driver REJECTING that resume, not asking for
# another one. Without it, braking to cancel an unwanted resume immediately queues the next one:
# brake, release, surge again -- a fight the driver cannot win using the one reflex they will
# actually reach for (Gemini review 2026-09-06, finding B). It latches the same opt-out as the
# double-tap, so a single firm brake is enough to say "stop".
REJECT_AFTER_FIRE_S = 5.0

# THE CROSS-CONTEXT GUARD (Gemini review 2026-09-06, finding A -- BLOCK).
# Remembering the set speed across the cruise-off gap is what makes this feature work at all, but a
# memory with only a time bound is a loaded gun: set 70 mph on the freeway, exit, drive several
# minutes on MADS lateral through town, brake and release at 15 mph -- and a purely time-bounded
# memory would hand the truck back a 70 mph target on a residential street. The lead gate is no
# protection there, because an empty street has no lead.
#
# Time alone cannot separate that from the case the driver actually wants, so this does not try.
# The discriminator is whether the captured set speed is a speed THIS DRIVE HAS RECENTLY BEEN DOING.
# `_v_max` is an approximate rolling maximum of v_ego over the last V_MAX_WINDOW_S; a resume is
# refused when the captured set speed stands more than V_MAX_MARGIN_MS above it.
#   * brake hard 70 -> 40 for traffic, release:   recent max 70, set 70   -> PASS (the main case)
#   * following a slow lead at 20 with set 31:    recent max ~31          -> PASS
#   * set above what traffic ever allowed:        margin covers it        -> PASS
#   * freeway memory used in a 25 mph town:       recent max ~7, set 31   -> REFUSE
# It also bounds finding D: it caps how much ACCELERATION any resume can command, since the target
# can never stand far above a speed the truck has just been holding.
V_MAX_WINDOW_S = 60.0
V_MAX_MARGIN_MS = 5.0        # ~11 mph of slack for a set speed traffic never let the truck reach
# The ABSOLUTE cap, and the primary bound on uncommanded acceleration (Fable review 2026-09-06, A1).
# `staleContext` alone is time-scoped, so it still permits the freeway-exit-then-yield case: brake
# 70 -> 25 mph down a ramp, release at the yield onto an arterial, and RES targets 70 from 15 mph
# with cross traffic and no lead to gate on. Time cannot separate that from "braked 70 -> 40 for
# traffic"; the SIZE OF THE JUMP can. 15 m/s (~34 mph) clears a hard brake from the set speed and
# refuses a resume that would command more acceleration than any brake-and-continue needs.
RESUME_MAX_DELTA_MS = 15.0

# Total lifetime of one arm. Lateral-only can persist indefinitely (that is the point of MADS);
# this bounds how long a resume can still be pending behind it so a resume can never arrive
# minutes after the brake that armed it.
ARM_MAX_S = 20.0

# How long after the press we keep watching for cruise to come back, to VERIFY what speed it came
# back at. Ford ACC re-engages well inside this.
VERIFY_S = 10.0


@dataclass
class ResumeInputs:
  """One tick of everything the brain is allowed to see. Plain python only."""
  now: float                       # monotonic seconds
  mads_available: bool             # madsState.available -- False => this module is fully inert
  lateral_only: bool               # madsState.lateralOnly
  op_enabled: bool                 # selfdriveState.enabled (openpilot's own engagement)
  blocked: bool                    # any blocking/disabling event this frame
  # selfdriveState.engageable -- would openpilot's OWN state machine accept an engage right now?
  # See gate `noEntry` in _gates(): this is NOT the same question as `blocked`.
  engageable: bool
  brake_pressed: bool
  regen_braking: bool
  gas_pressed: bool
  cruise_enabled: bool             # stock ACC actively engaged
  cruise_available: bool           # stock ACC main on (standby counts)
  set_speed_ms: float              # carState.cruiseState.speed as reported RIGHT NOW
  v_ego: float
  standstill: bool
  # radarState.leadOne. `has_lead is None` means the read FAILED -- that is a refusal, not "no
  # lead". d_rel/v_lead are only meaningful when has_lead is True.
  has_lead: bool | None
  d_rel: float | None = None
  v_lead: float | None = None
  # onetoggle2pnw: the separate MadsAutoResume toggle is GONE -- "Disengage on brake" governs both
  # halves of the behaviour. This is not a loosening: the arm gate below requires the rising edge of
  # `lateral_only`, and mads_pnw sets
  #     lateral_only = (not disengage_on_brake) and braking and not blocked
  # so lateral_only can ONLY be true when DisengageOnBrake is OFF. The toggle was therefore already
  # implied by gate 1, and a second control that can never independently be false is a control the
  # driver can be misled by. Pinned by test_resume_impossible_when_disengage_on_brake_is_on.


@dataclass
class ResumeDecision:
  """What the caller should do this tick."""
  offer: bool = False              # publish the resume command
  eid: float = 0.0                 # episode id -- constant for one offer, the executor's one-shot key
  set_ms: float = 0.0              # the driver's captured set speed (carried for the executor's own check)
  records: list = field(default_factory=list)   # telemetry records to append (usually empty)


def _finite(x) -> bool:
  try:
    return math.isfinite(float(x))
  except (TypeError, ValueError):
    return False


def lead_gate(has_lead, d_rel, v_lead, v_ego) -> str | None:
  """PURE. Returns None if the road ahead is open enough to hand speed back, else the name of the
  binding sub-gate. A FAILED radar read (`has_lead is None`) is `leadUnknown` -- a refusal, never
  a permissive default (CLAUDE.md rule 2: an error is not a negative result)."""
  if has_lead is None:
    return "leadUnknown"
  if not has_lead:
    return None
  if not (_finite(d_rel) and _finite(v_lead) and _finite(v_ego)):
    return "leadUnknown"
  d = float(d_rel)
  if d < LEAD_MIN_DIST_M:
    return "leadClose"
  # headway needs a speed to divide by; below the speed floor we have already refused on `slow`,
  # but be defensive rather than divide by ~0.
  v = float(v_ego)
  if v <= 0.1:
    return "leadClose"
  if d / v < LEAD_MIN_HEADWAY_S:
    return "leadGap"
  v_close = v - float(v_lead)
  if v_close > 0.0 and (d / v_close) < LEAD_MIN_TTC_S:
    return "leadTtc"
  return None


class MadsResumeBrain:
  """The bounded auto-resume state machine. One instance per drive; `update()` every control tick.

  Lifecycle of ONE brake event:
      (continuously) capture the driver's set speed while stock cruise is enabled
      lateralOnly rising edge, OR any
        later brake press while it holds -> ARM   (record "arm")
      brake+regen both released        -> the release clock starts
      RELEASE_MIN_S..RELEASE_MAX_S     -> if every gate passes: OFFER (record "fire"), latch _done
      offer ends                       -> record "offerEnd"
      window passes without an offer   -> record "refuse" naming the binding gate
      cruise comes back within VERIFY_S-> record "verify" (LOUD if it came back above the capture)
      lateralOnly falls                -> DISARM

  `_done` is the once-per-EPISODE latch: one brake press gets one press attempt, and it is cleared
  by a disarm. Since brakeretry2pnw a disarm no longer requires lateral_only to go False -- the next
  brake press starts a fresh episode. So a failed attempt does not get a retry *within* that press,
  but the driver always gets another attempt simply by braking again, which is the driver's own
  stated rule ("I can always push the brake"). The set-speed capture is REMEMBERED across all of
  this; see SET_MAX_AGE_S for why it must be, and what bounds it."""

  def __init__(self):
    # continuous set-speed capture
    self._set_ms: float | None = None
    self._set_t: float | None = None
    # arm state
    self._armed = False
    self._arm_t = 0.0
    self._armed_set: float | None = None
    self._armed_set_age = 0.0
    self._released_t: float | None = None
    self._done = False              # once-per-event latch
    self._terminal = False          # a terminal record has already been written for this arm
    self._last_block = "init"       # the most recent binding gate, for the terminal record
    # offer state
    self._offer_t: float | None = None
    self._eid = 0.0
    # post-press verification
    self._verify_until: float | None = None
    self._verify_set: float | None = None
    # Edge detector. THREE-STATE: None = "never observed", which is NOT the same fact as
    # "observed False" (Gemini review 2026-09-06). With a plain False, the first tick after the
    # brain becomes active -- e.g. a selfdrived restart mid-drive
    # on while already steering-only -- reads as a rising edge and ARMS without any brake
    # transition having been observed at all: precisely outside the bounded state. The first
    # observation now only SEEDS the detector; arming needs a genuine False->True after that.
    self._lat_prev = None
    # brakeretry2pnw double-tap opt-out: time of the last brake rising edge, and the latch it sets.
    self._last_brake_t: float | None = None
    self._suppressed = False
    # Pedal-only edge detector (regen deliberately excluded -- see the edge block in update()).
    # THREE-STATE like _lat_prev: None = never observed.
    self._pedal_prev = None
    self._pedal_off_t: float | None = None
    # when our own resume last fired, for the post-resume rejection check
    self._fired_t: float | None = None
    # cruise_enabled edge detector, for clearing the opt-out. THREE-STATE like _lat_prev.
    self._cc_prev = None
    # approximate rolling max of v_ego, for the cross-context guard
    self._v_max: float | None = None
    self._v_max_t = 0.0
    # diagnostics: how many times update() raised inside the caller's guard (caller-owned counter
    # lives in selfdrived; this one just proves the brain itself ran).
    self.ticks = 0

  # -- helpers ---------------------------------------------------------------------------------

  def _snap(self, i: ResumeInputs, extra: dict | None = None) -> dict:
    """The common telemetry body. Everything a post-hoc reader needs to re-derive the decision."""
    ttc = None
    hdwy = None
    try:
      if i.has_lead and _finite(i.d_rel) and _finite(i.v_lead) and float(i.v_ego) > 0.1:
        hdwy = round(float(i.d_rel) / float(i.v_ego), 2)
        vc = float(i.v_ego) - float(i.v_lead)
        ttc = round(float(i.d_rel) / vc, 1) if vc > 0.0 else None
    except (TypeError, ValueError, ZeroDivisionError):
      ttc, hdwy = None, None
    rec = {
      "vEgo": round(float(i.v_ego), 2) if _finite(i.v_ego) else None,
      "setMs": round(self._armed_set, 2) if self._armed_set is not None else None,
      "setAgeS": round(self._armed_set_age, 2),
      "stockSet": round(float(i.set_speed_ms), 2) if _finite(i.set_speed_ms) else None,
      "lead": i.has_lead,
      "dRel": round(float(i.d_rel), 1) if (i.has_lead and _finite(i.d_rel)) else None,
      "vLead": round(float(i.v_lead), 1) if (i.has_lead and _finite(i.v_lead)) else None,
      "ttc": ttc, "hdwy": hdwy,
      "relS": round(i.now - self._released_t, 2) if self._released_t is not None else None,
      "armS": round(i.now - self._arm_t, 2) if self._armed else None,
      "brk": bool(i.brake_pressed), "regen": bool(i.regen_braking), "gas": bool(i.gas_pressed),
      "ccOn": bool(i.cruise_enabled), "ccAvail": bool(i.cruise_available),
      "blocked": bool(i.blocked), "latOnly": bool(i.lateral_only),
      "opEn": bool(i.op_enabled), "engbl": bool(i.engageable), "eid": self._eid,
      # without these a staleContext/setFar refusal cannot be re-derived from the record
      "vMax": round(self._v_max, 2) if self._v_max is not None else None,
      "vMaxAgeS": round(i.now - self._v_max_t, 1) if self._v_max is not None else None,
      "supp": bool(self._suppressed),
      "sinceFireS": round(i.now - self._fired_t, 2) if self._fired_t is not None else None,
    }
    if extra:
      rec.update(extra)
    return rec

  def _terminate(self, i: ResumeInputs, out: ResumeDecision, reason: str) -> None:
    """Write the ONE terminal record for this arm, if it hasn't been written yet."""
    if self._terminal:
      return
    self._terminal = True
    out.records.append(self._snap(i, {"phase": "refuse", "reason": reason, "fired": False}))

  def _disarm(self) -> None:
    self._armed = False
    self._armed_set = None
    self._armed_set_age = 0.0
    self._released_t = None
    self._done = False
    self._terminal = False
    self._offer_t = None
    self._last_block = "idle"

  # -- the tick --------------------------------------------------------------------------------

  def update(self, i: ResumeInputs) -> ResumeDecision:
    self.ticks += 1
    out = ResumeDecision()

    # --- gate 8: inert unless MADS is actually available (never acts on the Tesla) --------------
    # and gate 0: the driver-facing kill switch, default OFF.
    if not i.mads_available:
      # Hold every latch cleared so enabling mid-drive can never see a stale edge/arm. The edge
      # detector is SEEDED from this tick's observation rather than forced False: forcing False
      # while lateral_only is already True is exactly what would manufacture a rising edge on the
      # tick the driver flips the toggle back on (Gemini review 2026-09-06).
      self._lat_prev = bool(i.lateral_only)
      self._last_brake_t = None
      self._suppressed = False
      self._pedal_prev = bool(i.brake_pressed)
      self._pedal_off_t = None
      self._fired_t = None
      self._v_max = None
      self._v_max_t = 0.0
      self._cc_prev = bool(i.cruise_enabled)
      self._set_ms = None
      self._set_t = None
      self._verify_until = None
      if self._armed:
        self._disarm()
      return out

    # --- continuous capture of the driver's OWN set speed (gate 6's only source) ----------------
    # Refreshed on every tick stock cruise reports engaged. This is the ONLY place _set_ms is
    # written, so it can never pick up a value from a frame where cruise was off.
    # --- rolling max of v_ego, for the cross-context guard ---------------------------------------
    # Approximate by design: hold the max, and let it expire to the current speed once it is older
    # than the window. That is one comparison per tick and needs no buffer. A NON-FINITE v_ego does
    # not update it (and the gate below refuses outright on one), so a bad read can never inflate
    # the ceiling and thereby permit a resume it should have refused.
    if _finite(i.v_ego):
      v = float(i.v_ego)
      if self._v_max is None or v >= self._v_max or (i.now - self._v_max_t) > V_MAX_WINDOW_S:
        self._v_max = v
        self._v_max_t = i.now

    cc_rising = bool(i.cruise_enabled) and self._cc_prev is False
    self._cc_prev = bool(i.cruise_enabled)
    if cc_rising:
      # The driver has cruise engaged again -- by a manual RES, by a SET+/SET- adjustment, or
      # because our own press landed. Whichever it was, the opt-out is spent.
      #
      # RISING EDGE, not level (Fable review 2026-09-06, P2). On the level, a single frame where
      # cruiseState.enabled still reads True after the driver's rejection brake wipes `_suppressed`
      # immediately -- and the truck resumes again, which is precisely the unwinnable fight the
      # opt-out exists to end. mads_pnw.py:235-237 states the PCM ordering is not guaranteed, so
      # that frame is not hypothetical.
      self._suppressed = False
      self._last_brake_t = None
    if i.cruise_enabled and _finite(i.set_speed_ms):
      s = float(i.set_speed_ms)
      if SET_MIN_MS <= s <= SET_MAX_MS:
        self._set_ms = s
        self._set_t = i.now
    elif not i.cruise_available:
      # The ACC master switch is OFF -- the driver has turned cruise off entirely, not merely had it
      # dropped by the brake. Whatever they had set is no longer "the speed they set"; forget it now
      # rather than letting the 10-minute backstop carry it across an explicit switch-off.
      self._set_ms = None
      self._set_t = None

    # --- post-press verification (runs independently of arm/disarm) ----------------------------
    if self._verify_until is not None:
      if i.now > self._verify_until:
        # Cruise never came back inside the window. That is not an error (the press may have been
        # correctly ignored), but it IS the difference between "we pressed and nothing happened"
        # and "we pressed and it worked", so it is logged.
        out.records.append(self._snap(i, {"phase": "verify", "reason": "noCruise", "fired": True}))
        self._verify_until = None
      elif i.cruise_enabled and _finite(i.set_speed_ms) and float(i.set_speed_ms) > 0.0:
        got = float(i.set_speed_ms)
        want = self._verify_set if self._verify_set is not None else 0.0
        # Fable B1: the axiom is "the speed the driver ALREADY SET", which is violated by a
        # DIFFERENT speed in either direction -- a come-back well BELOW the capture means the PCM
        # did not restore its remembered set, so something set a new speed. That is not dangerous
        # (it is slower), but reporting it as "ok" would hide the fact that the mechanism did not
        # behave as this whole design assumes, which is exactly the thing the log exists to catch.
        if got > want + SET_TOL_MS:
          reason = "setHigher"
        elif got < want - SET_TOL_MS:
          reason = "setLower"
        else:
          reason = "ok"
        out.records.append(self._snap(i, {
          "phase": "verify", "reason": reason, "fired": True,
          "gotMs": round(got, 2), "wantMs": round(want, 2),
        }))
        if reason != "ok":
          out.records[-1]["loud"] = True
        self._verify_until = None

    # --- arm on the lateral-only edge OR on any later brake press (gate 1) ---------------------
    lat = bool(i.lateral_only)
    # `is False`, not `not self._lat_prev`: a None (never-observed) previous state must NOT
    # produce a rising edge -- see the _lat_prev comment in __init__.
    lat_rising = lat and self._lat_prev is False
    self._lat_prev = lat

    # Brake EDGE detection -- for re-arming AND for the opt-out -- reads the PEDAL ONLY, never
    # regen. Regen braking flickers as the driver modulates, and every flicker would be an "edge":
    # that would manufacture both spurious re-arms and spurious double-taps (Gemini review
    # 2026-09-06, finding C). Gating and the arm precondition still use brake-or-regen; this is
    # only about detecting a discrete PRESS.
    pedal = bool(i.brake_pressed)
    pedal_rising = pedal and self._pedal_prev is False
    if pedal_rising and self._pedal_off_t is not None and (i.now - self._pedal_off_t) < BRAKE_DEBOUNCE_S:
      pedal_rising = False                      # chatter within one press, not a second press
    if self._pedal_prev is not False and not pedal:
      self._pedal_off_t = i.now                 # pedal just came up (or first observation, released)
    self._pedal_prev = pedal

    # brakeretry2pnw: the opt-out. Evaluated on the brake EDGE, before arming, so a second press
    # suppresses rather than re-arms.
    if pedal_rising:
      double = self._last_brake_t is not None and (i.now - self._last_brake_t) <= DOUBLE_BRAKE_S
      post_resume = self._fired_t is not None and (i.now - self._fired_t) <= REJECT_AFTER_FIRE_S
      if double or post_resume:
        self._suppressed = True
        why = "doubleBrake" if double else "postResumeBrake"
        # Rule 2: a feature that quietly stops acting is exactly the thing that must say so.
        out.records.append(self._snap(i, {"phase": "suppress", "reason": why, "fired": False}))
        if self._armed:
          # An episode was open. The driver has just overruled it; end it now rather than letting
          # its window keep running behind the opt-out they just asked for.
          self._terminate(i, out, why)
          self._disarm()
      self._last_brake_t = i.now

    # brakeretry2pnw: EVERY brake press while MADS is holding lateral opens a resume opportunity,
    # not only the first one after cruise dropped. Before this, arming required the RISING EDGE of
    # lateral_only, which can happen only once per cruise-off transition -- so if that single
    # attempt refused for any reason (the 2026-09-06 drive refused on `gas` one second in), there
    # was no second chance until the driver manually re-engaged cruise to create a new edge. The
    # driver's rule is "I can always push the brake", and this is that rule: press, release, resume.
    if pedal_rising and lat and self._suppressed:
      # Rule 2: this is a no-resume class of its own, and the whole 2026-09-06 investigation was
      # about diagnosing a no-resume. Silence here would recreate exactly that problem.
      out.records.append(self._snap(i, {"phase": "refuse", "reason": "suppressed", "fired": False}))

    start = (lat_rising or (lat and pedal_rising)) and not self._suppressed

    if start:
      if self._armed:
        # A previous episode is still open. _terminate() is a no-op if its terminal record was
        # already written, so this cannot double-report -- but an arm still inside its window has
        # NO terminal record yet, and dropping it silently would break the one-terminal-record-per
        # -arm contract in the class docstring (Fable review 2026-09-06, P4).
        self._terminate(i, out, "reBrake")
        self._disarm()
      # ARM. Snapshot the captured set speed and its age RIGHT HERE -- nothing after this point may
      # move _armed_set, so the target can never drift after the driver's foot left the brake.
      self._armed = True
      self._arm_t = i.now
      self._done = False
      self._terminal = False
      self._released_t = None
      self._offer_t = None
      self._eid = round(i.now, 3)
      age = (i.now - self._set_t) if self._set_t is not None else float("inf")
      self._armed_set_age = age if math.isfinite(age) else -1.0
      self._armed_set = self._set_ms if (self._set_ms is not None and age <= SET_MAX_AGE_S) else None
      self._last_block = "armed"
      out.records.append(self._snap(i, {"phase": "arm", "reason": None, "fired": False}))
      if not (i.brake_pressed or i.regen_braking):
        # Fable A2: mads_pnw only ever raises lateral_only on a frame where `braking` is true (both
        # the immediate arm and the brake-grace arm test it), so this is unreachable today. It is
        # here so that a FUTURE mads arming path cannot silently hand this feature an episode that
        # was not brake-induced -- the one precondition the whole envelope rests on.
        self._done = True
        self._terminate(i, out, "noBrake")
        return out
      if self._armed_set is None:
        # Gate 6 can never be satisfied for this arm. Refuse NOW and say so, rather than letting
        # the window run and reporting a vaguer reason 3 s later. Name the SPECIFIC cause: if the
        # ACC master is off, "accOff" is what a reader needs to see -- reporting the `noSet` that
        # the master being off just caused would hide the actual reason one level down.
        self._done = True
        self._terminate(i, out, "accOff" if not i.cruise_available else "noSet")
      return out

    if not self._armed:
      return out

    if not lat:
      # Lateral-only ended -- either openpilot re-engaged (our press worked, or the driver pressed
      # resume themselves) or steering was lost. Either way this arm is over.
      self._terminate(i, out, "latOff")
      self._disarm()
      return out

    # --- gate 7: aborts. Any of these ends the arm outright (no retry until a new brake cycle). --
    abort = None
    if i.gas_pressed:
      abort = "gas"
    elif i.blocked:
      abort = "blocked"
    elif i.op_enabled:
      abort = "opEngaged"
    elif not i.cruise_available:
      abort = "accOff"
    elif i.cruise_enabled:
      # Stock cruise is back without openpilot re-engaging (driver hit resume themselves, or the
      # PCM did). Nothing left to ask for.
      abort = "ccOn"
    elif i.now - self._arm_t > ARM_MAX_S:
      abort = "armExpired"
    if abort is not None:
      if self._offer_t is not None:
        # An offer was on the wire; say so explicitly rather than letting it vanish from the log.
        out.records.append(self._snap(i, {"phase": "offerEnd", "reason": abort, "fired": True}))
      self._terminate(i, out, abort)
      self._disarm()
      return out

    # --- gate 2: the brake must be FULLY released before the clock starts ----------------------
    braking = bool(i.brake_pressed) or bool(i.regen_braking)
    if braking:
      # The release clock must measure a CONTINUOUS release. A re-press that BRAKE_DEBOUNCE_S
      # swallowed as chatter is still braking, however long it is then held -- without this reset a
      # fire can land 50 ms after the real release, skipping the settle the window exists to
      # enforce. The old `reBrake` abort used to guarantee this for free (Fable review, P1).
      self._released_t = None
      if self._offer_t is not None:
        out.records.append(self._snap(i, {"phase": "offerEnd", "reason": "braking", "fired": True}))
        self._offer_t = None
      self._last_block = "braking"
      return out
    if self._released_t is None:
      self._released_t = i.now
      self._last_block = "settling"
      return out

    since = i.now - self._released_t

    # --- an offer already in flight: keep it alive only while every gate still holds ------------
    if self._offer_t is not None:
      block = self._gates(i)
      if block is None and (i.now - self._offer_t) <= OFFER_S:
        out.offer = True
        out.eid = self._eid
        out.set_ms = float(self._armed_set)
        return out
      # Offer over. Record why, and stop offering. `_done` stays set: no second offer.
      out.records.append(self._snap(i, {
        "phase": "offerEnd", "reason": block or "expired", "fired": True,
      }))
      self._offer_t = None
      return out

    if self._done:
      return out

    # --- gate 3: the bounded window ------------------------------------------------------------
    if since < RELEASE_MIN_S:
      self._last_block = "settling"
      return out
    if since > RELEASE_MAX_S:
      # The window closed without firing. ONE terminal record, naming the gate that was binding.
      self._terminate(i, out, self._last_block if self._last_block not in ("settling", "armed") else "windowExpired")
      self._done = True
      return out

    # --- gates 5 + 6 + speed floor -------------------------------------------------------------
    block = self._gates(i)
    if block is not None:
      self._last_block = block
      return out

    # --- FIRE. Latch first, offer second. -------------------------------------------------------
    self._done = True
    self._offer_t = i.now
    self._fired_t = i.now
    self._verify_until = i.now + VERIFY_S
    self._verify_set = float(self._armed_set)
    self._terminal = True          # "fire" IS this arm's terminal record
    out.records.append(self._snap(i, {"phase": "fire", "reason": None, "fired": True}))
    out.offer = True
    out.eid = self._eid
    out.set_ms = float(self._armed_set)
    return out

  def _gates(self, i: ResumeInputs) -> str | None:
    """The gates that are re-evaluated every tick of the window AND every tick of the offer.
    Returns None (clear) or the name of the binding gate. Ordered cheapest/most-fundamental first
    so the reported reason is the most informative one."""
    if self._armed_set is None:
      return "noSet"                                    # gate 6 (already terminal at arm, defensive)
    if not _finite(i.v_ego) or float(i.v_ego) < V_EGO_MIN_MS or i.standstill:
      return "slow"
    # Fable A1 (HIGH), and the single most important gate that was MISSING: openpilot's own state
    # machine must be willing to engage. A NO_ENTRY event (resumeBlocked, tooDistracted, outOfSpace,
    # stockLkas, speedTooHigh, selfdriveInitializing...) carries no DISABLE type, so it does not show
    # up in `blocked` and MADS happily keeps holding lateral. But if our RES press engages the stock
    # ACC while a NO_ENTRY stands:
    #     stock cruise engages -> openpilot REFUSES to engage with it (NO_ENTRY)
    #     -> controlsd.py sends cruiseControl.cancel (`CS.cruiseState.enabled and not CC.enabled`)
    #     -> mads_pnw sees `cruise_engage_edge` and REVOKES lateral (mads_pnw.py:238)
    # Net effect: the driver was steering-only, and OUR press blipped cruise on/off and took their
    # STEERING away. Fail-to-stock in direction, but caused by this feature and entirely avoidable.
    if not i.engageable:
      return "noEntry"
    # gate 6, live half: the truck must not be reporting a set speed ABOVE the one we captured.
    # A LOWER reported set is fine -- resume would go there, which is still not above the driver's,
    # and an absent/zero reading is expected (the Lightning may report 0 in ACC standby).
    # A NON-FINITE reading is a REFUSAL, not a pass: gate 6's live half cannot be evaluated at all,
    # and `isfinite(x) and x > thresh` short-circuits to "permit" on a NaN. Same bug, same fix, as
    # decide_resume() in opendbc/car/ford/icbm_pnw.py.
    if not _finite(i.set_speed_ms):
      return "setUnknown"
    if float(i.set_speed_ms) > 0.0 and float(i.set_speed_ms) > self._armed_set + SET_TOL_MS:
      return "setRaised"
    # the cross-context guard -- see V_MAX_WINDOW_S. An absent rolling max is a REFUSAL, not a pass:
    # the gate cannot be evaluated, and this is the gate that bounds uncommanded acceleration.
    if self._armed_set - float(i.v_ego) > RESUME_MAX_DELTA_MS:
      return "setFar"
    if self._v_max is None or self._armed_set > self._v_max + V_MAX_MARGIN_MS:
      return "staleContext"
    return lead_gate(i.has_lead, i.d_rel, i.v_lead, i.v_ego)     # gate 5
