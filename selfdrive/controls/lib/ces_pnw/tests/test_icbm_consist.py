"""icbmconsist2pnw — POINT-MATCHED polyline curvature beside mapd's own target. TELEMETRY ONLY.

WHY THIS IS A MEASUREMENT AND NOT A FIX, which is the whole story of this change:

The 2026-09-08 20:28 phantom (mapd claimed a curve on a straight 60 mph motorway; ICBM dragged the
stock set 70 -> 44 mph) needs mapd's velocity checked against real geometry. The obvious check —
compare the curvature mapd's target implies against `icbmK` — was BUILT, and then rejected on the
evidence of a full replay of that drive:

  * `icbmK` is the horizon MAXIMUM curvature, not the curvature at MAPD'S point. Across all 28 ticks
    a check would have fired on, the two points were 51-328 m apart — NEVER closer than 50 m. It was
    never comparing mapd's claim against mapd's curve.
  * At 19:37:47 it contradicted a CORRECT mapd claim (a trunk ramp at R~=38 m, 22 m ahead) using an
    unrelated gentler curve 172 m further along, and would have suppressed a real slowdown on
    R 62-172 m geometry — the one outcome that is unacceptable.
  * It fired on 28 of 28 eligible ticks with ZERO reading consistent. A consistency check that never
    finds consistency is measuring a systematic offset (the pipeline's own scale and penalty
    factors), not discriminating.
  * The polyline's under-read tail is p99 3.29x — beyond any margin that still keeps a real curve safe.

So this ships the missing MEASUREMENT instead, telemetry-first, exactly as icbmcurv2pnw did before
anything consumed `icbmK`. `icbmKAtGap` is the load-bearing field: it says how far the nearest
measurable triplet fell from mapd's point, i.e. whether comparing them is legitimate at all. On the
one drive we have, it never was.
"""
import math

import pytest

from openpilot.selfdrive.controls.lib.vtsc_pnw.vtsc_pnw import (polyline_curvature,
                                                                polyline_curvature_at)

HORIZON = 500.0


def _arc(lat0, lon0, radius_m, n=12, step_m=40.0):
  """n points along a circular arc of the given radius, starting at (lat0, lon0) heading north."""
  pts = []
  for i in range(n):
    theta = (i * step_m) / radius_m
    x = radius_m * (1.0 - math.cos(theta))       # east offset
    y = radius_m * math.sin(theta)               # north offset
    pts.append({"latitude": lat0 + y / 111320.0,
                "longitude": lon0 + x / (111320.0 * math.cos(math.radians(lat0)))})
  return pts


def _straight(lat0, lon0, n=12, step_m=40.0):
  return [{"latitude": lat0 + (i * step_m) / 111320.0, "longitude": lon0} for i in range(n)]


class TestItMeasuresAtTheRequestedPoint:
  def test_it_lands_near_the_requested_distance(self):
    pts = _arc(47.0, -122.0, 300.0, n=12, step_m=40.0)
    for want in (80.0, 200.0, 360.0):
      k, d, n_ok, gap, _ = polyline_curvature_at(pts, 47.0, -122.0, HORIZON, want)
      assert n_ok > 0, "the arc must be measurable"
      assert gap == pytest.approx(abs(d - want), abs=1e-6)
      assert gap <= 60.0, f"asked for {want} m, matched at {d:.0f} m (gap {gap:.0f} m)"

  def test_it_recovers_a_known_radius(self):
    """Sanity that this is really Menger curvature and not something else: a 300 m arc must read
    ~1/300."""
    pts = _arc(47.0, -122.0, 300.0, n=14, step_m=45.0)
    k, _, n_ok, _, _ = polyline_curvature_at(pts, 47.0, -122.0, HORIZON, 200.0)
    assert n_ok > 0
    assert k == pytest.approx(1.0 / 300.0, rel=0.25)

  def test_it_differs_from_the_horizon_maximum_when_the_road_does(self):
    """THE DEFECT THIS EXISTS FOR. A road that is straight near the car and bends hard further on:
    the horizon max reports the far bend; the point-matched reading at the near point must not."""
    pts = _straight(47.0, -122.0, n=6, step_m=45.0)
    tail = _arc(pts[-1]["latitude"], pts[-1]["longitude"], 80.0, n=8, step_m=40.0)
    road = pts + tail
    k_max, d_max, _, _, _ = polyline_curvature(road, 47.0, -122.0, HORIZON)
    k_at, d_at, n_ok, gap, _ = polyline_curvature_at(road, 47.0, -122.0, HORIZON, 90.0)
    assert n_ok > 0 and gap <= 60.0
    assert k_max > k_at, "the horizon max must see the far bend"
    assert d_max > d_at, "...at a greater distance than the point we asked about"
    assert k_at < 1.0 / 500.0, "the near, straight part must read as gentle"


class TestTheContract:
  def test_a_straight_road_reads_zero_WITH_a_nonzero_n_ok(self):
    """k == 0 is ambiguous on its own; n_ok is what separates 'straight' from 'unmeasurable'. A
    collinear triplet is a LEGITIMATE match — 'straight at mapd's point' is the reading a future
    check needs most."""
    k, _, n_ok, gap, _ = polyline_curvature_at(_straight(47.0, -122.0), 47.0, -122.0, HORIZON, 200.0)
    assert k == 0.0
    assert n_ok > 0, "a straight road IS measurable; n_ok must say so"
    assert gap <= 60.0

  def test_unmeasurable_reports_n_ok_zero(self):
    """Nodes spaced beyond the leg gate cannot be measured -- must report 0, not a fake reading."""
    far = [{"latitude": 47.0 + i * 0.02, "longitude": -122.0} for i in range(4)]   # ~2.2 km legs
    k, _, n_ok, _, _ = polyline_curvature_at(far, 47.0, -122.0, 20000.0, 2000.0)
    assert (k, n_ok) == (0.0, 0)

  def test_gap_is_zero_when_nothing_matched(self):
    k, d, n_ok, gap, _ = polyline_curvature_at([], 47.0, -122.0, HORIZON, 200.0)
    assert (k, d, n_ok, gap) == (0.0, 0.0, 0, 0.0)

  def test_ahead_flag_marks_a_point_behind_us(self):
    pts = _arc(47.0, -122.0, 300.0, n=12, step_m=40.0)
    _, _, _, _, ahead_n = polyline_curvature_at(pts, 47.0, -122.0, HORIZON, 200.0, cur_bearing=0.0)
    _, _, _, _, ahead_s = polyline_curvature_at(pts, 47.0, -122.0, HORIZON, 200.0, cur_bearing=180.0)
    assert ahead_n and not ahead_s

  @pytest.mark.parametrize("bad_at", [float("nan"), float("inf")])
  def test_a_nonfinite_target_distance_abstains(self, bad_at):
    pts = _arc(47.0, -122.0, 300.0)
    assert polyline_curvature_at(pts, 47.0, -122.0, HORIZON, bad_at)[2] == 0

  @pytest.mark.parametrize("pts", [
    None, [], [{"latitude": "x", "longitude": None}],
    [{"latitude": float("nan"), "longitude": -122.0}] * 5,
    [{}, {}, {}],
  ])
  def test_malformed_input_never_raises(self, pts):
    """Pure and total: this runs beside the control path and must never throw into it."""
    out = polyline_curvature_at(pts, 47.0, -122.0, HORIZON, 200.0)
    assert out == (0.0, 0.0, 0, 0.0, True) or out[2] == 0

  def test_none_position_abstains(self):
    pts = _arc(47.0, -122.0, 300.0)
    assert polyline_curvature_at(pts, None, None, HORIZON, 200.0)[2] == 0


class TestItCannotPerturbTheSibling:
  def test_the_horizon_max_function_is_unchanged_on_the_same_inputs(self):
    """polyline_curvature is on the Tesla's live VTSC control path. This change must be additive."""
    pts = _arc(47.0, -122.0, 250.0, n=14, step_m=45.0)
    a = polyline_curvature(pts, 47.0, -122.0, HORIZON)
    polyline_curvature_at(pts, 47.0, -122.0, HORIZON, 150.0)
    b = polyline_curvature(pts, 47.0, -122.0, HORIZON)
    assert a == b, "the point-matched call must not mutate anything the sibling reads"
