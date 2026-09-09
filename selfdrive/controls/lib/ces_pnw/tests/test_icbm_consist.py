"""icbmconsist2pnw — the polyline consistency check that separates a phantom map curve from a real one.

Every number here is REPLAYED FROM REAL TELEMETRY, not invented. Source:
drives/2026-09-08/i5-evening-police-miss-and-onramp-steer/ces_events_*.jsonl (4077 ticks, one drive).

THE PROBLEM THIS SOLVES, and why the obvious tests would have passed the wrong fix:

  A  20:28:51 PT, 60 mph motorway. mapd claimed a curve, ICBM commanded 44.9 mph, the stock set was
     dragged 70 -> 44 mph. There was NO curve: achLat -0.05..-1.1 m/s^2, vision silent, curveSrc
     fell back to "map" with curvePct saturated at 100.
  C  19:44:07 PT, same evening, same 60 mph limit, same road class, also map-sourced. Drop 26.0 mph
     vs A's 25.9. But this one was REAL and hard -- 3.13 m/s^2 achieved, R ~= 172 m, `steerEvent
     angSat` twice, and the driver took the wheel.

A and C are indistinguishable on drop size, posted limit, road class and source. Any threshold on
DEPTH suppresses the real one too. What separates them is the polyline geometry in mapd's own
message, which icbmcurv2pnw already measures (icbmK/icbmKN).

Governing safety property: the check is RAISE-ONLY. It can only make ICBM slow LESS. So the failure
mode to guard against is not "it braked too hard" but "it suppressed a real curve" -- which is what
C exists to catch, and why the margin is 1.5 and not 1.0.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw_constants as C

A_LAT = 2.5          # vtsc_constants.A_LAT_TARGET, what the caller passes
MPH = 2.23694
LIM_60 = 26.8        # m/s, the 60 mph posted limit on both episodes
LIM_25 = 11.2        # m/s, the 25 mph roads earlier in the same drive


def sanity(target_ms, spd_lim=LIM_60, k=0.003228, kn=10, ahead=True, src="map", **kw):
  return C.icbm_curvature_sanity(target_ms, spd_lim, k, kn, ahead, src, A_LAT, **kw)


class TestTheTwoRealEpisodes:
  def test_A_the_phantom_is_raised(self):
    """20:28:52-20:29:07 PT: icbmT 44.9 mph, icbmK 0.003228, icbmKN 10, ahead. mapd implied R=197 m;
    the polyline said 310-350 m; reality was ~580 m (1.12 m/s^2 at 57 mph)."""
    out, fired = sanity(44.9 / MPH)
    assert fired
    assert out * MPH == pytest.approx(50.8, abs=0.5), "the 70->44 drop becomes 70->51"
    assert out > 44.9 / MPH, "raise-only"

  def test_C_the_real_curve_still_gets_enough_slowing(self):
    """19:44:10-19:44:21 PT: icbmT 34.0 mph, icbmK 0.004242, icbmKN 3-6, ahead. THE test that kills a
    too-aggressive margin. The curve's measured R was ~172 m, so holding A_LAT needs
    sqrt(2.5*172) = 20.7 m/s = 46.3 mph. The check must leave the target AT OR BELOW that."""
    out, fired = sanity(34.0 / MPH, k=0.004242, kn=6)
    assert fired
    needed_mph = math.sqrt(A_LAT * 172.0) * MPH
    assert out * MPH <= needed_mph, (
      f"raised to {out * MPH:.1f} mph but the real curve needs <= {needed_mph:.1f} mph")
    assert out * MPH == pytest.approx(44.3, abs=0.5)

  def test_margin_1_0_would_be_UNSAFE_on_the_real_curve(self):
    """Why ICBM_CONSIST_MARGIN is 1.5 and not 1.0, in one assertion. At margin 1.0 the real curve's
    target becomes 54.3 mph -- ABOVE the 52 mph at which this truck's steering actually saturated on
    that very curve. The polyline under-read C by ~1.4x (R 236 m measured vs 172 m achieved); the
    margin is what absorbs that."""
    out, _ = sanity(34.0 / MPH, k=0.004242, kn=6, margin=1.0)
    assert out * MPH > 52.0, "margin 1.0 raises above the measured saturation speed"
    safe, _ = sanity(34.0 / MPH, k=0.004242, kn=6, margin=C.ICBM_CONSIST_MARGIN)
    assert safe * MPH < 52.0, "the shipped margin stays below it"

  def test_B_abstains_because_the_polyline_was_unmeasurable(self):
    """19:28:53-19:29:08 PT: icbmKN was 0-2 throughout. Too few points to overrule mapd, so the check
    must leave the (over-aggressive, 3.4x) target completely alone rather than guess."""
    for kn, k in ((2, 0.000321), (1, 6.2e-05), (0, 0.0)):
      out, fired = sanity(34.7 / MPH, spd_lim=17.9, k=k, kn=kn)
      assert not fired, f"KN={kn} must abstain"
      assert out == 34.7 / MPH


class TestScope:
  def test_below_the_floor_limit_it_abstains(self):
    """The posted-limit floor owns <= 30 mph. Verified against the same drive: on its 25 mph roads
    (19:25, 19:36, 19:37) the polyline was measurable and the check would otherwise have fired --
    e.g. 24.0 mph target with k=0.002928/KN=4 -- suppressing real low-speed slowdowns."""
    out, fired = sanity(24.0 / MPH, spd_lim=LIM_25, k=0.002928, kn=4)
    assert not fired
    assert out == 24.0 / MPH

  def test_the_scope_boundary_is_the_floor_limit(self):
    assert not sanity(20.0, spd_lim=C.ICBM_FLOOR_MAX_LIMIT, k=0.004, kn=9)[1]
    assert sanity(20.07, spd_lim=C.ICBM_FLOOR_MAX_LIMIT + 0.1, k=0.004, kn=9)[1]

  def test_vision_sourced_targets_are_never_second_guessed(self):
    """Vision has its own evidence; this check only adjudicates mapd's velocity against mapd's own
    polyline. 19:44:07 was vis-sourced and must pass through untouched."""
    out, fired = sanity(47.2 / MPH, src="vis", k=0.004242, kn=6)
    assert not fired and out == 47.2 / MPH

  def test_a_near_zero_curvature_abstains_rather_than_disabling_icbm(self):
    """THE suppress-everything trap. polyline_curvature's contract says `KN > 0 and k ~= 0` does NOT
    mean "straight" -- it is ambiguous. Trusting it would drive the allowed curvature to ~0, raise
    every target toward infinity and silently disable ICBM."""
    out, fired = sanity(20.0, k=1e-05, kn=12)
    assert not fired, "an ambiguous ~0 reading must abstain, never license an unbounded raise"
    assert out == 20.0

  def test_curvature_not_ahead_abstains(self):
    assert not sanity(20.07, ahead=False)[1]


class TestSafetyProperties:
  def test_it_is_raise_only_across_the_whole_plausible_range(self):
    """The core safety property: this can never make ICBM slow MORE."""
    for t_mph in (20, 30, 40, 50, 60, 70):
      for k in (0.0011, 0.002, 0.003228, 0.006, 0.02):
        out, _ = sanity(t_mph / MPH, k=k)
        assert out >= t_mph / MPH - 1e-9, f"lowered a target at {t_mph} mph, k={k}"

  def test_a_consistent_target_is_untouched(self):
    """mapd agreeing with the geometry must be a no-op, not a nudge."""
    k = 0.003228
    consistent = math.sqrt(A_LAT / (k * C.ICBM_CONSIST_MARGIN)) * 1.01   # slightly slower than allowed
    out, fired = sanity(consistent, k=k)
    assert not fired and out == consistent

  @pytest.mark.parametrize("bad", [None, float("nan"), 0.0, -5.0])
  def test_unusable_targets_abstain(self, bad):
    out, fired = sanity(bad)
    assert not fired and out == bad or (bad is not None and bad != bad)

  @pytest.mark.parametrize("kwargs", [
    {"spd_lim": None}, {"k": None}, {"kn": None}, {"k": float("nan")},
    {"spd_lim": float("nan")}, {"kn": "x"}, {"src": None},
  ])
  def test_malformed_inputs_abstain_rather_than_raise(self, kwargs):
    """Pure and total: this runs in the control path and must never throw into it."""
    out, fired = sanity(20.07, **kwargs)
    assert not fired
    assert out == 20.07
