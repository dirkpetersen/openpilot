"""curvefloor2pnw — posted-limit floor for the Lightning's ICBM (stock-ACC) path.

The event this exists for (drives/2026-08-11, 06:52:45-57 PT, ces_events_full.jsonl):
    spdLim 11.2 m/s (25 mph) · mapd target 7.3 m/s SUSTAINED 12 s · icbmSrc "map"
    · stock set tapped 23.7 -> 7.15 m/s (16 mph) on a 25 mph road.
icbmratchet2pnw does not catch it: that gate confirms single-tick OUTLIER drops
(ICBM_RATCHET_OUTLIER_DROP_MS = 3.0 over 0.6 s), and this was a sustained map target.

Scope note (Fable review 2026-09-05): the ORIGINAL branch floored on ALL roads and guarded it with a
debounced steering-saturation "evidence gate". Both were rejected and are NOT ported --
 * all-roads: at 45 mph on an R=80 m bend, flooring at the limit demands 5.05 m/s^2 lateral, ~2x
   A_LAT_TARGET and above the Lightning's measured ~4.5 m/s^2 ceiling -> understeer into a takeover;
 * the evidence gate needed >=2 consecutive ~1 Hz reads (2-3 s) while the 08-11 apex saturation
   lasted ONE tick, so it would never have opened -- it was an escape hatch that could not fire.
Only the low-limit floor is ported, where the physics is guaranteed rather than assumed.
"""
import inspect
import time as _t

import pytest

from openpilot.selfdrive.controls.lib import pnw_vehicle as pv
from openpilot.selfdrive.controls.lib.pnw_vehicle import PnwVehicle
from openpilot.selfdrive.controls.lib.ces_pnw import ces_pnw as m
from openpilot.selfdrive.controls.lib.ces_pnw.ces_pnw_constants import (ICBM_FLOOR_HYST_MS,
                                                                        ICBM_FLOOR_MAX_LIMIT,
                                                                        icbm_floor_limit)

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"


# --- the debounced limit selector -------------------------------------------------------------
class TestFloorLimitSelector:
  def test_a_low_posted_limit_qualifies(self):
    assert icbm_floor_limit(11.2, 0.0) == pytest.approx(11.2)   # 25 mph, the 08-11 road

  def test_a_high_posted_limit_never_qualifies(self):
    """THE scope bound. Above this the posted limit is not guaranteed holdable through a bend, and
    flooring there is what the review rejected."""
    assert icbm_floor_limit(ICBM_FLOOR_MAX_LIMIT + 0.1, 0.0) == 0.0
    assert icbm_floor_limit(20.1, 0.0) == 0.0                   # 45 mph
    assert icbm_floor_limit(ICBM_FLOOR_MAX_LIMIT, 0.0) == pytest.approx(ICBM_FLOOR_MAX_LIMIT)

  def test_unknown_limit_means_no_floor(self):
    assert icbm_floor_limit(0.0, 0.0) == 0.0
    assert icbm_floor_limit(-1.0, 0.0) == 0.0

  def test_flicker_inside_the_deadband_is_held(self):
    """Logs show spd_lim flapping 11.2 <-> 8.9 (a 2.3 m/s step, UNDER the ratchet's 3.0 outlier
    band). Chasing it would produce SET-button tap flicker."""
    assert icbm_floor_limit(8.9, 11.2) == pytest.approx(11.2)

  def test_a_real_change_is_followed(self):
    assert icbm_floor_limit(11.2 - ICBM_FLOOR_HYST_MS - 0.1, 11.2) == pytest.approx(8.6, abs=0.11)

  def test_garbage_never_raises_and_means_no_floor(self):
    for bad in (None, "x", float("nan"), float("inf")):
      assert icbm_floor_limit(bad, 0.0) == 0.0


# --- end-to-end through the real _icbm_step ---------------------------------------------------
def _rig(tmp_path, monkeypatch):
  monkeypatch.setattr(pv, "CURVE_CONFIG_PATH", str(tmp_path / "nope.json"))
  cls = next(o for o in vars(m).values() if inspect.isclass(o) and hasattr(o, "_icbm_step"))

  class FakeMem:
    last = None

    def put_nonblocking(self, k, v):
      self.last = v

  class FakeCP:
    carFingerprint = LIGHTNING
    brand = "ford"
    openpilotLongitudinalControl = False

  class Stub:
    pass

  g = Stub()
  g.mem_params = FakeMem()
  g._veh = PnwVehicle(FakeCP())
  g._icbm_ceiling = None
  g._map_targets = []
  g._cur_lat = g._cur_lon = None
  g._icbm_ep = m.IcbmEpisode()
  g._icbm_dir = None
  g._icbm_floor_lim = 0.0
  g._icbm_floor_hit = False
  g._icbm_gate = None
  g._icbm_map_reach = None
  g._stock_set = 0.0
  g._stock_on = False
  step = cls._icbm_step.__get__(g)

  def run(map_v, spd_lim, v_ego=11.0, v_set=23.7, dist=60.0):
    g._stock_set, g._stock_on = v_set, True
    g._icbm_last_pub = _t.monotonic() - 1.0
    step({"v_ego": v_ego, "v_set": v_set, "map_target_v": map_v, "map_target_dist": dist,
          "curve_lat_accel_vision": 0.0, "time_to_curve": 5.0, "lat_accel_now": 0.0,
          "spd_lim": spd_lim, "pitch": None, "gas": False, "brake": False}, active=True)
    return g
  return run, g


class TestFloorOnTheIcbmPath:
  def test_low_limit_road_raises_a_too_low_target(self, tmp_path, monkeypatch):
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(6):
      run(map_v=7.3, spd_lim=11.2)
    assert g._icbm_floor_lim == pytest.approx(11.2), "the floor never armed on a 25 mph road"
    assert g._icbm_floor_hit is True, "the floor did not raise a 7.3 m/s target under an 11.2 limit"

  def test_the_same_target_on_a_45mph_road_is_NOT_floored(self, tmp_path, monkeypatch):
    """Control for the scope bound -- proves the assertion is about the LIMIT, not the target."""
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(6):
      run(map_v=7.3, spd_lim=20.1, v_ego=20.0, v_set=20.1)
    assert g._icbm_floor_lim == 0.0 and g._icbm_floor_hit is False

  def test_the_floor_never_exceeds_the_reference(self, tmp_path, monkeypatch):
    """min(..., ref): the floor may only raise a too-low target TOWARD the limit, never command
    anything above what the driver/episode already allows.

    Numbers matter here or the test is vacuous (the first cut guarded on `if t is not None` and a
    mutation removing the `min()` SURVIVED it). On a 13.0 m/s limit the floor lands at 12.55 after
    the Lightning curve penalty. With the set speed at 11.0 the floor is ABOVE the set, so the
    `min()` is the only thing stopping the ICBM commanding 12.55 on an 11.0 set."""
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(8):
      run(map_v=5.0, spd_lim=13.0, v_set=11.0, v_ego=11.0)
    pub = g.mem_params.last or {}
    assert "target" in pub, "nothing was published -- the scenario did not exercise the floor"
    assert pub["target"] == pytest.approx(11.0, abs=1e-6), \
      f"floor commanded {pub['target']} above the 11.0 set speed (min(..., ref) is not binding)"

  def test_the_floor_lands_at_the_limit_minus_the_penalties(self, tmp_path, monkeypatch):
    """NOT the bare posted limit. Flooring at the raw value would undo the Lightning curve margin
    and the rain margin that icbmalign2pnw / rain2pnw exist to apply."""
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(8):
      run(map_v=5.0, spd_lim=13.0, v_set=23.7, v_ego=13.0)
    pub = g.mem_params.last or {}
    assert "target" in pub
    assert pub["target"] < 13.0 - 1e-6, "floored at the BARE limit -- penalties were not subtracted"
    assert pub["target"] == pytest.approx(12.55, abs=0.2)

  def test_a_target_already_above_the_floor_is_untouched(self, tmp_path, monkeypatch):
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(6):
      run(map_v=12.0, spd_lim=11.2)
    assert g._icbm_floor_hit is False, "the floor fired on a target that was already fine"

  def test_the_floor_state_is_published_for_telemetry(self, tmp_path, monkeypatch):
    """Without icbmFlr/icbmFlrHit on the drive log there is no way to distinguish 'the floor never
    applied' from 'the floor applied and was not needed' -- the ambiguity that made the 08-11
    over-slow hard to close in the first place. (waysel2pnw shipped telemetry that reached nothing.)"""
    run, g = _rig(tmp_path, monkeypatch)
    for _ in range(6):
      run(map_v=7.3, spd_lim=11.2)
    assert hasattr(g, "_icbm_floor_lim") and hasattr(g, "_icbm_floor_hit")
