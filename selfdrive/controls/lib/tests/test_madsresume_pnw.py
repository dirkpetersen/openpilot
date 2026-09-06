"""
madsresume2pnw -- the gate matrix for the bounded auto-resume brain.

Every hard gate in the feature spec gets at least one test that FAILS if the gate is removed
(mutation-verified; see the branch doc for the log). The brain is pure, so these drive it directly
with a `ResumeInputs` sequence -- no cereal, no msgq, no params.

The helper below is deliberately explicit rather than clever: `run()` feeds a list of per-tick
input overrides at 100 Hz and returns (offers, records), so a test reads as the drive it describes.
"""

import pytest

from openpilot.selfdrive.controls.lib import madsresume_pnw as M
from openpilot.selfdrive.controls.lib.madsresume_pnw import MadsResumeBrain, ResumeInputs, lead_gate

SET = 29.0            # ~65 mph, a plausible driver set speed
DT = 0.01


def mk(now, **kw):
  """A tick of 'cruising normally at the set speed, no lead' with overrides applied."""
  base = dict(
    now=now, mads_available=True, lateral_only=False, op_enabled=True, blocked=False,
    brake_pressed=False, regen_braking=False, gas_pressed=False,
    cruise_enabled=True, cruise_available=True, set_speed_ms=SET, v_ego=SET,
    standstill=False, has_lead=False, d_rel=None, v_lead=None, enabled=True, engageable=True,
  )
  base.update(kw)
  return ResumeInputs(**base)


class Drive:
  """Runs the brain over a scripted drive. `t` advances 10 ms per tick."""

  def __init__(self, **defaults):
    self.b = MadsResumeBrain()
    self.t = 0.0
    self.defaults = defaults
    self.offers = []          # (t, eid, set_ms) for every tick an offer was published
    self.records = []

  def tick(self, n=1, **kw):
    kw = {**self.defaults, **kw}
    for _ in range(n):
      out = self.b.update(mk(self.t, **kw))
      if out.offer:
        self.offers.append((round(self.t, 3), out.eid, out.set_ms))
      self.records.extend(out.records)
      self.t += DT
    return self

  def phases(self):
    return [r["phase"] for r in self.records]

  def reasons(self, phase):
    return [r.get("reason") for r in self.records if r["phase"] == phase]

  def fired(self):
    return len(self.offers) > 0


# Timeline of the reference drive (100 Hz):
#   t=0.00 .. 0.50   cruising with stock ACC engaged -> the set speed is captured
#   t=0.50 .. 0.70   brake down, openpilot disengages, MADS holds lateral (the ARM edge)
#   t=0.70 ..        brake fully released -> the release clock starts
#   t=1.20 ..        RELEASE_MIN_S elapsed: the earliest a resume may fire
#   t=3.70           RELEASE_MAX_S elapsed: the window closes, terminal refusal if it never fired
# The post block runs 5 s so every drive reaches its own terminal record.
RELEASE_T = 0.70


def normal_brake_and_resume(post_ticks=500, **overrides):
  """The reference drive: cruising -> brake (MADS holds lateral) -> release -> clear road.

  `overrides` apply to the brake block AND the post-release block (never to the capture block, so
  the driver's set speed is always observed first) -- that is what lets `mads_available=False` and
  `enabled=False` suppress the ARM edge itself rather than only the fire."""
  d = Drive()
  d.tick(50)                                                        # capture the set speed
  brake = dict(lateral_only=True, op_enabled=False, cruise_enabled=False,
               brake_pressed=True, set_speed_ms=0.0)
  brake.update(overrides)
  d.tick(20, **brake)                                               # braking, MADS armed
  post = dict(lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  post.update(overrides)
  d.tick(post_ticks, **post)
  return d


# ---------------------------------------------------------------------------------------------
# The happy path -- everything below is a deviation from THIS.
# ---------------------------------------------------------------------------------------------

def test_happy_path_fires_once_with_the_captured_set_speed():
  d = normal_brake_and_resume()
  assert d.fired(), f"reference drive must resume; records={d.records}"
  assert d.phases().count("fire") == 1
  assert d.phases().count("refuse") == 0
  # gate 6: the offered set speed is EXACTLY the one captured before the brake, never higher.
  assert all(abs(o[2] - SET) < 1e-6 for o in d.offers)
  # one episode id for the whole offer -- the executor's one-shot key must not change under it.
  assert len({o[1] for o in d.offers}) == 1


def test_fire_waits_for_the_settle_time():
  d = normal_brake_and_resume()
  first = d.offers[0][0]
  assert first - RELEASE_T >= M.RELEASE_MIN_S - 1e-9, f"fired {first - RELEASE_T:.3f}s after release"
  assert first - RELEASE_T <= M.RELEASE_MAX_S + 1e-9


# ---------------------------------------------------------------------------------------------
# Gate 1 -- only from the brake-induced lateral-only state
# ---------------------------------------------------------------------------------------------

def test_gate1_never_arms_without_lateral_only():
  """A plain brake press with openpilot fully disengaged (no MADS latch) must do nothing at all."""
  d = Drive()
  d.tick(50)
  d.tick(20, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(200, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.records == [], "no lateral-only means the brain must not even arm"


def test_gate1_lateral_only_ending_aborts_the_window():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(30, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(100, lateral_only=False, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "latOff" in d.reasons("refuse")


# ---------------------------------------------------------------------------------------------
# Gate 2 -- only after the brake is FULLY released (brake AND regen)
# ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("held", ["brake_pressed", "regen_braking"])
def test_gate2_never_fires_while_still_braking(held):
  d = normal_brake_and_resume(**{held: True})
  assert not d.fired(), f"{held} still true -- must not resume"


def test_gate2_the_release_clock_starts_at_the_release_not_at_the_arm():
  """Gate 2 is enforced twice over -- the release clock only starts once the brake is fully up, AND
  a re-press after a release aborts outright. This pins the first half: a longer brake press must
  push the earliest possible fire out by exactly as much, never fire early off the arm time."""
  short = Drive()
  short.tick(50)
  short.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  short.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  long_ = Drive()
  long_.tick(50)
  long_.tick(120, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  long_.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert short.fired() and long_.fired()
  assert long_.offers[0][0] - short.offers[0][0] == pytest.approx(1.0, abs=2 * DT)


def test_gate2_regen_alone_holds_the_release_clock():
  """Foot off the friction brake but regen still decelerating is NOT 'fully released'."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(200, lateral_only=True, op_enabled=False, cruise_enabled=False, regen_braking=True, set_speed_ms=0.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 3 -- bounded window after the release
# ---------------------------------------------------------------------------------------------

def test_gate3_window_closes_and_refuses():
  """A lead too close for the whole window -> the window expires with ONE explained refusal."""
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(600, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         has_lead=True, d_rel=15.0, v_lead=SET)
  assert not d.fired()
  assert d.phases().count("refuse") == 1, d.records
  assert d.reasons("refuse") == ["leadClose"]


def test_gate3_a_clear_road_after_the_window_is_too_late():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  # blocked by a close lead until well past RELEASE_MAX_S...
  d.tick(int((M.RELEASE_MAX_S + 0.5) / DT), lateral_only=True, op_enabled=False,
         cruise_enabled=False, set_speed_ms=0.0, has_lead=True, d_rel=15.0, v_lead=SET)
  # ...then the road clears completely. Too late: the window is a hard bound.
  d.tick(500, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 4 -- ONCE per brake event
# ---------------------------------------------------------------------------------------------

def test_gate4_only_one_offer_episode_per_arm():
  """The offer stops after OFFER_S and never restarts, even though every gate still passes."""
  d = normal_brake_and_resume()
  span = d.offers[-1][0] - d.offers[0][0]
  assert span <= M.OFFER_S + 2 * DT, f"offer ran {span:.2f}s, bound is {M.OFFER_S}s"
  assert d.phases().count("fire") == 1
  assert d.phases().count("offerEnd") == 1


def test_gate4_a_second_resume_needs_a_new_brake_to_lateral_only_cycle():
  d = normal_brake_and_resume()
  n_first = len(d.offers)
  assert n_first > 0
  # ONE fire record for this arm -- this is the assertion that pins "once", independently of how
  # long the caller happens to run: the window gate alone would also stop a re-fire eventually, and
  # an offer-count comparison taken after the window closed would silently absorb a broken latch.
  assert d.phases().count("fire") == 1
  # lateral-only stays true for a long time; nothing may fire again.
  d.tick(1000, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert len(d.offers) == n_first
  assert d.phases().count("fire") == 1
  # A genuinely NEW cycle: re-engage, then brake into lateral-only again.
  d.tick(100)                                                              # op re-engaged, cruise on
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False,
         brake_pressed=True, set_speed_ms=0.0)
  d.tick(120, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert len(d.offers) > n_first, "a new brake->lateral-only cycle must be allowed to resume"


def test_gate4_re_braking_inside_the_window_aborts_rather_than_restarting_it():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)   # released
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "reBrake" in d.reasons("refuse")


# ---------------------------------------------------------------------------------------------
# Gate 5 -- never with a close lead or low TTC
# ---------------------------------------------------------------------------------------------

def test_gate5_close_lead_blocks():
  d = normal_brake_and_resume(has_lead=True, d_rel=18.0, v_lead=SET)
  assert not d.fired()


def test_gate5_short_headway_blocks():
  # 40 m at 29 m/s = 1.38 s headway, under the 2.0 s bar, but not under the 20 m floor.
  d = normal_brake_and_resume(has_lead=True, d_rel=40.0, v_lead=SET)
  assert not d.fired()
  assert "leadGap" in d.reasons("refuse")


def test_gate5_fast_closing_lead_blocks_even_at_a_long_gap():
  # 100 m (3.4 s headway, passes) but closing at 20 m/s -> TTC 5 s, under the 8 s bar.
  d = normal_brake_and_resume(has_lead=True, d_rel=100.0, v_lead=SET - 20.0)
  assert not d.fired()
  assert "leadTtc" in d.reasons("refuse")


def test_gate5_open_road_behind_a_matched_lead_is_allowed():
  # 90 m at matched speed: 3.1 s headway, no closing rate. This is what "clear enough" means.
  d = normal_brake_and_resume(has_lead=True, d_rel=90.0, v_lead=SET)
  assert d.fired()


def test_gate5_a_failed_radar_read_is_a_refusal_not_an_open_road():
  """CLAUDE.md rule 2: an error is not a negative result. has_lead=None must REFUSE."""
  d = normal_brake_and_resume(has_lead=None)
  assert not d.fired()
  assert "leadUnknown" in d.reasons("refuse")


def test_lead_gate_pure():
  assert lead_gate(None, None, None, 30.0) == "leadUnknown"
  assert lead_gate(False, None, None, 30.0) is None
  assert lead_gate(True, 10.0, 30.0, 30.0) == "leadClose"
  assert lead_gate(True, 40.0, 30.0, 30.0) == "leadGap"
  assert lead_gate(True, 100.0, 10.0, 30.0) == "leadTtc"
  assert lead_gate(True, 100.0, 30.0, 30.0) is None
  assert lead_gate(True, float("nan"), 30.0, 30.0) == "leadUnknown"


# ---------------------------------------------------------------------------------------------
# Gate 6 -- the driver's PREVIOUS set speed, never above it, refuse if unknown
# ---------------------------------------------------------------------------------------------

def test_gate6_no_captured_set_speed_refuses_immediately_at_arm():
  """Cruise was never engaged, so no set speed was ever observed -> refuse at the arm tick."""
  d = Drive()
  d.tick(50, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.reasons("refuse") == ["noSet"]
  assert d.records[0]["phase"] == "arm" and d.records[0]["setMs"] is None


def test_gate6_a_stale_capture_refuses():
  """Cruise off for longer than SET_MAX_AGE_S before the brake -> the capture is not this event's."""
  d = Drive()
  d.tick(50)
  d.tick(int((M.SET_MAX_AGE_S + 0.5) / DT), cruise_enabled=False, set_speed_ms=0.0, op_enabled=False)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert "noSet" in d.reasons("refuse")


def test_gate6_a_set_speed_below_fords_own_minimum_is_not_a_set_speed():
  d = Drive()
  d.tick(50, set_speed_ms=5.0, v_ego=5.0)          # 5 m/s ~ 11 mph, below Ford's 20 mph ACC minimum
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0, v_ego=5.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=5.0)
  assert not d.fired()
  assert "noSet" in d.reasons("refuse")


def test_gate6_a_higher_reported_set_speed_blocks():
  """If the truck reports a set speed ABOVE the captured one, resuming could go above it."""
  d = normal_brake_and_resume(set_speed_ms=SET + 3.0)
  assert not d.fired()
  assert "setRaised" in d.reasons("refuse")


def test_gate6_a_nonfinite_reported_set_speed_refuses():
  """A gate whose input is unreadable fails CLOSED -- `isfinite(x) and x > t` would permit on NaN."""
  for bad in (float("nan"), float("inf")):
    d = normal_brake_and_resume(set_speed_ms=bad)
    assert not d.fired()
    assert "setUnknown" in d.reasons("refuse")


def test_gate6_a_lower_reported_set_speed_is_fine():
  """Resume can only ever go to the PCM's own remembered set; lower than captured is not 'above'."""
  d = normal_brake_and_resume(set_speed_ms=SET - 3.0)
  assert d.fired()


def test_gate6_the_offer_is_never_above_the_captured_set_speed():
  """Exhaustive over the reference drive: no offer may exceed the captured value, ever."""
  d = normal_brake_and_resume()
  assert d.offers and all(o[2] <= SET + 1e-9 for o in d.offers)


def test_verify_records_a_resume_that_came_back_too_high():
  d = normal_brake_and_resume()
  assert d.fired()
  # cruise comes back, but at 5 m/s above what the driver had set.
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET + 5.0)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "setHigher" and verify[0]["loud"] is True


def test_verify_records_a_correct_resume_quietly():
  d = normal_brake_and_resume()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "ok" and "loud" not in verify[0]


# ---------------------------------------------------------------------------------------------
# Gate 7 -- aborts
# ---------------------------------------------------------------------------------------------

@pytest.mark.parametrize("override,reason", [
  ({"gas_pressed": True}, "gas"),
  ({"blocked": True}, "blocked"),
  ({"op_enabled": True}, "opEngaged"),
  ({"cruise_available": False}, "accOff"),
  ({"cruise_enabled": True}, "ccOn"),
])
def test_gate7_aborts(override, reason):
  d = normal_brake_and_resume(**override)
  assert not d.fired(), f"{reason}: must not resume"
  assert reason in d.reasons("refuse"), d.records


def test_gate7_arm_expiry_bounds_a_long_lateral_only():
  """Lateral-only can last indefinitely; a pending resume must not."""
  d = Drive()
  d.tick(50)
  d.tick(int((M.ARM_MAX_S + 1.0) / DT), lateral_only=True, op_enabled=False,
         cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  assert not d.fired()
  assert "armExpired" in d.reasons("refuse")


def test_gate7_gas_during_the_offer_withdraws_it():
  d = normal_brake_and_resume(post_ticks=60)      # stop 0.1 s into the 1.0 s offer
  n = len(d.offers)
  assert n > 0 and n < int(M.OFFER_S / DT), "fixture must stop mid-offer"
  d.tick(50, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         gas_pressed=True)
  assert len(d.offers) == n, "the offer must be withdrawn the instant the driver touches the gas"
  assert "gas" in [r.get("reason") for r in d.records if r["phase"] == "offerEnd"]


def test_low_speed_refuses():
  d = Drive()
  d.tick(50, v_ego=6.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0, v_ego=6.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, v_ego=6.0)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Gate 8 -- inert unless MADS is available (so it can never act on the Tesla), and the kill switch
# ---------------------------------------------------------------------------------------------

def test_gate8_inert_without_mads():
  d = normal_brake_and_resume(mads_available=False)
  assert not d.fired()
  assert d.records == [], "no MADS -> not even a log record"


def test_gate8_inert_without_mads_even_on_the_arming_tick():
  """mads_available False must also suppress the ARM edge, not just the fire."""
  d = Drive(mads_available=False)
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired() and d.records == []


def test_the_inputs_dataclass_defaults_the_toggle_to_off():
  """A call site that forgets the toggle must get the feature OFF. Self-engagement fails closed."""
  b = MadsResumeBrain()
  bare = ResumeInputs(now=0.0, mads_available=True, lateral_only=False, op_enabled=True,
                      blocked=False, engageable=True, brake_pressed=False, regen_braking=False,
                      gas_pressed=False, cruise_enabled=True, cruise_available=True,
                      set_speed_ms=SET, v_ego=SET, standstill=False, has_lead=False)
  assert bare.enabled is False
  out = b.update(bare)
  assert out.offer is False and out.records == []


def test_toggle_off_is_fully_inert():
  d = normal_brake_and_resume(enabled=False)
  assert not d.fired() and d.records == []


def test_toggle_flipped_off_mid_window_stops_everything():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0, enabled=False)
  assert not d.fired()


# ---------------------------------------------------------------------------------------------
# Review-driven hardening (Gemini + Fable, 2026-09-06)
# ---------------------------------------------------------------------------------------------

def test_a_first_observation_of_lateral_only_is_not_a_rising_edge():
  """selfdrived restarting mid-drive, or the toggle flipped on while ALREADY steering-only, must
  not read as a brake transition. The edge detector is three-state: None != observed-False."""
  d = Drive()
  d.tick(50)                                                     # capture a set speed
  # first tick the brain ever sees lateral_only is already True -- no brake edge was ever observed
  b = MadsResumeBrain()
  out = b.update(mk(0.0, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0))
  assert out.offer is False and out.records == [], "a first observation must only SEED the detector"


def test_toggle_flipped_on_while_already_lateral_only_does_not_arm():
  d = Drive()
  d.tick(50)
  # steering-only for a while with the feature OFF...
  d.tick(200, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         enabled=False)
  # ...driver flips it ON mid-episode. There was no observed brake transition, so nothing arms.
  d.tick(600, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.records == [], f"must not arm off a toggle flip; got {d.phases()}"


def test_mads_becoming_available_mid_drive_while_lateral_only_does_not_arm():
  d = Drive()
  d.tick(50)
  d.tick(200, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         mads_available=False)
  d.tick(600, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired() and d.records == []


def test_capture_age_bound_covers_the_mads_brake_grace_window():
  """SET_MAX_AGE_S is COUPLED to mads_pnw.MADS_BRAKE_GRACE_FRAMES: the brake may land that many
  frames after the falling edge, and the capture has already stopped refreshing by then. This
  module cannot import mads_pnw (that would drag in cereal), so the coupling is pinned by reading
  the constant out of the source text. If someone widens the grace window, this fails."""
  import pathlib
  import re
  src = (pathlib.Path(M.__file__).parent.parent.parent / "selfdrived" / "mads_pnw.py").read_text()
  m = re.search(r"^MADS_BRAKE_GRACE_FRAMES = (\d+)$", src, re.M)
  assert m, "could not find MADS_BRAKE_GRACE_FRAMES in mads_pnw.py"
  grace_s = int(m.group(1)) * DT
  assert M.SET_MAX_AGE_S > grace_s, (
    f"SET_MAX_AGE_S={M.SET_MAX_AGE_S}s must exceed the MADS brake grace window ({grace_s}s) or a legitimate late-brake arm would refuse with noSet")
  assert M.SET_MAX_AGE_S <= grace_s + 0.5, (
    f"SET_MAX_AGE_S={M.SET_MAX_AGE_S}s exceeds grace {grace_s}s + margin; slack is time in which another cruise drop can be mistaken for this brake's")


def test_a_late_brake_inside_the_mads_grace_window_still_captures_the_set_speed():
  """The whole reason SET_MAX_AGE_S is not tiny: MADS may arm up to 0.45 s after cruise dropped."""
  d = Drive()
  d.tick(50)                                                     # cruise on, set speed captured
  # cruise drops first (the measured pedal lead), brake lands 0.40 s later, MADS arms then.
  d.tick(40, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True,
         set_speed_ms=0.0)
  d.tick(500, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert d.fired(), f"a late brake inside the MADS grace window must still resume; {d.records}"
  assert all(abs(o[2] - SET) < 1e-6 for o in d.offers)


def test_the_published_wire_contract_matches_what_the_executor_parses():
  """The brain and the executor live in DIFFERENT REPOS (pnw-pilot and pnw-opendbc) and the only
  thing between them is a JSON mem-param. Nothing else in either test suite covers that seam, and a
  key rename on one side would fail SILENTLY -- the executor would parse None and simply never
  press, which looks exactly like "the gates refused". Pin the exact key set here; the matching
  assertion on the other side is opendbc test_parses_a_well_formed_offer.

  Read out of selfdrived's source by AST rather than executed, because importing selfdrived needs
  cereal/capnp, which is not built on the dev host."""
  import ast
  import pathlib
  src = (pathlib.Path(M.__file__).parent.parent.parent / "selfdrived" / "selfdrived.py").read_text()
  tree = ast.parse(src)
  fn = next(n for n in ast.walk(tree)
            if isinstance(n, ast.FunctionDef) and n.name == "_mads_resume_step")
  pubs = [n for n in ast.walk(fn)
          if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "put_nonblocking"
          and n.args and isinstance(n.args[0], ast.Constant) and n.args[0].value == "MadsResumeTarget"]
  assert len(pubs) == 2, f"expected one offer publish and one withdrawal, got {len(pubs)}"
  offer = next(c for c in pubs if isinstance(c.args[1], ast.Dict) and c.args[1].keys)
  keys = {k.value for k in offer.args[1].keys}
  assert keys == {"dir", "ts", "eid", "set"}, f"wire contract drifted: {keys}"
  direction = next(v for k, v in zip(offer.args[1].keys, offer.args[1].values, strict=True) if k.value == "dir")
  assert direction.value == "res", "the resume payload must be marked dir='res'"
  withdraw = next(c for c in pubs if isinstance(c.args[1], ast.Dict) and not c.args[1].keys)
  assert withdraw is not None, "there must be an explicit empty-dict withdrawal"


def test_a_standing_no_entry_blocks_the_resume():
  """Fable A1 (the highest-value finding): a NO_ENTRY carries no DISABLE type, so it does not show
  up in `blocked` and MADS keeps holding lateral. But if our RES engages stock cruise while one
  stands, openpilot refuses to engage, controlsd sends CANCEL, and mads_pnw revokes lateral on the
  cruise-engage edge -- our press would take the driver's STEERING away."""
  d = normal_brake_and_resume(engageable=False)
  assert not d.fired()
  assert "noEntry" in d.reasons("refuse"), d.records


def test_no_entry_appearing_during_the_offer_withdraws_it():
  d = normal_brake_and_resume(post_ticks=60)
  n = len(d.offers)
  assert n > 0
  d.tick(50, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0,
         engageable=False)
  assert len(d.offers) == n
  assert "noEntry" in [r.get("reason") for r in d.records if r["phase"] == "offerEnd"]


def test_engageable_is_recorded_so_a_no_entry_refusal_is_diagnosable():
  d = normal_brake_and_resume(engageable=False)
  assert all("engbl" in r for r in d.records)
  assert d.records[0]["engbl"] is False


def test_arm_without_the_brake_down_refuses():
  """Fable A2: unreachable through today's mads_pnw (it only raises lateral_only on a braking
  frame), but a future arming path must not be able to hand this feature a non-brake episode."""
  d = Drive()
  d.tick(50)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False, set_speed_ms=0.0)
  assert not d.fired()
  assert d.reasons("refuse") == ["noBrake"]


def test_verify_flags_a_resume_that_came_back_too_LOW_as_well():
  """Fable B1: the axiom is 'the speed the driver ALREADY set'. A come-back well below the capture
  means the PCM did not restore its remembered set -- not dangerous, but not what this design
  assumes either, and reporting it as 'ok' would hide it."""
  d = normal_brake_and_resume()
  assert d.fired()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET - 6.0)
  verify = [r for r in d.records if r["phase"] == "verify"]
  assert len(verify) == 1 and verify[0]["reason"] == "setLower" and verify[0]["loud"] is True


# ---------------------------------------------------------------------------------------------
# Telemetry -- a silent no-resume and a silent wrong-resume must be distinguishable
# ---------------------------------------------------------------------------------------------

def test_every_arm_produces_exactly_one_terminal_record():
  for kw in ({}, {"has_lead": True, "d_rel": 15.0, "v_lead": SET}, {"gas_pressed": True},
             {"has_lead": None}, {"set_speed_ms": SET + 3.0}):
    d = normal_brake_and_resume(**kw)
    terminal = [r for r in d.records if r["phase"] in ("fire", "refuse")]
    assert len(terminal) == 1, f"{kw} -> {[r['phase'] for r in d.records]}"


def test_a_refusal_always_names_a_reason():
  d = normal_brake_and_resume(has_lead=True, d_rel=15.0, v_lead=SET)
  refusals = [r for r in d.records if r["phase"] == "refuse"]
  assert refusals and all(r["reason"] for r in refusals)
  assert all(r["fired"] is False for r in refusals)


def test_records_are_json_serializable():
  import json
  d = normal_brake_and_resume()
  d.tick(10, lateral_only=False, op_enabled=False, cruise_enabled=True, set_speed_ms=SET)
  for r in d.records:
    json.loads(json.dumps(r))     # would raise on a NaN/inf or a non-primitive


def test_records_survive_nonfinite_inputs():
  d = Drive()
  d.tick(50)
  d.tick(20, lateral_only=True, op_enabled=False, cruise_enabled=False, brake_pressed=True, set_speed_ms=0.0)
  d.tick(300, lateral_only=True, op_enabled=False, cruise_enabled=False,
         set_speed_ms=float("nan"), v_ego=float("nan"), has_lead=True,
         d_rel=float("inf"), v_lead=float("nan"))
  import json
  for r in d.records:
    json.loads(json.dumps(r))
  assert not d.fired()
