"""policenear2-2pnw + policelastseen2pnw — the DISPLAY pick is proximity-first at EVERY range, and
the minutes shown are "last seen", not "first reported".

Both were driver-reported on the 2026-09-08 I-5 drive and both were root-caused from the on-device
forensics log (drives/2026-09-08/i5-evening-police-miss-and-onramp-steer/):

  * "there's another police report ahead and my police shows 11 miles" — measured: the displayed
    report was outside a 60-degree forward cone in 30/77 ticks, and a NEARER in-hemisphere report
    existed and lost in 28/77. Cause: `_select` was proximity-first only within POLICE_NEAR_MI (1 mi)
    and beyond that delegated to geo.nearest_ahead(), which ranks on along-track distance measured
    from path[0] -- the first node of the whole current OSM way, which can be kilometres BEHIND the
    car. So a report behind us, or one a bend projects onto the polyline interior, scored a tiny
    `along` and won.
  * "the minute indicator should show ... when the police was last seen" — `age_min` is the Waze
    publication age, so "Police 1.5 mi (65 min)" read as "first reported 65 min ago".

The governing driver rule (2026-09-08): "if there's a police report too many that's fine, I just
don't want to miss any." So the selector must NEVER return empty when a usable candidate exists, and
confidence is carried by tier COLOUR, never by dropping a report.
"""
import pytest

from openpilot.system.location_services import location_servicesd as lsd
from openpilot.system.location_services.location_servicesd import (_line_police, _now_epoch,
                                                                   merge_retained_police)

# Due north of (47.0, -122.0); ~69 mi per degree of latitude at this longitude.
DEG_PER_MI = 1.0 / 69.0


def _alert(uuid, mi_north, age_min=5, thumbs=0, lon=-122.0):
  """A report `mi_north` miles due north of the reference point (negative = south/behind)."""
  return {"lat": 47.0 + mi_north * DEG_PER_MI, "lon": lon, "magvar": None, "uuid": uuid,
          "street": "", "town": "T", "thumbs": thumbs,
          "ts": None if age_min is None else (_now_epoch() - age_min * 60.0) * 1000.0}


def _line(alerts, brg=0.0, path=None):
  """brg 0.0 = heading due north, so a report with mi_north > 0 is AHEAD."""
  recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
  return _line_police(alerts, "ok", "", 47.0, -122.0, brg, path if path is not None else [], recede)


class TestProximityFirstAtEveryRange:
  def test_near_report_beats_a_far_one_well_beyond_the_near_lock(self):
    """THE REGRESSION. Both are far outside POLICE_NEAR_MI (1.0), so the old code fell through to
    the along-track projection; the 11 mi report won there. 3.8 mi must win."""
    out = _line([_alert("far", 11.3), _alert("near", 3.8)])
    assert out["state"] == "alert"
    assert out["dist_mi"] == pytest.approx(3.8, abs=0.2)

  def test_the_measured_1958_case(self):
    """Reconstruction of 2026-09-08 19:58:10 PT: 14 kept reports, nearest ahead 3.8 mi, shipped code
    displayed 11.3 mi."""
    alerts = [_alert(f"r{i}", d) for i, d in enumerate([3.8, 8.4, 8.5, 8.8, 9.0, 9.3,
                                                        10.1, 11.3, 11.8, 15.0])]
    out = _line(alerts)
    assert out["dist_mi"] == pytest.approx(3.8, abs=0.2), "the nearest AHEAD report must win"

  def test_a_report_behind_us_is_never_displayed(self):
    """11/77 shipped picks were behind the car (rel bearing 91-130 deg). Behind must lose to any
    report ahead, however much farther."""
    out = _line([_alert("behind", -1.8), _alert("ahead", 9.0)])
    assert out["dist_mi"] == pytest.approx(9.0, abs=0.3)

  def test_behind_only_yields_no_alert_rather_than_a_wrong_one(self):
    out = _line([_alert("behind", -2.0)])
    assert out["state"] != "alert"

  def test_never_empty_when_a_candidate_is_ahead(self):
    """The never-miss rule: a bounded remedy that can return None where the shipped code returned a
    report would be a regression, whatever else it fixed."""
    for d in (0.4, 1.5, 4.0, 9.0, 14.5):
      assert _line([_alert("x", d)])["state"] == "alert", f"{d} mi ahead must still display"

  def test_beyond_the_display_range_is_not_shown(self):
    assert _line([_alert("x", 40.0)])["state"] != "alert"

  def test_unknown_heading_does_not_drop_reports(self):
    """brg None -> no bearing filter at all. Fail OPEN: never lose a report because we do not know
    our own heading."""
    out = _line([_alert("x", 5.0)], brg=None)
    assert out["state"] == "alert"

  def test_path_projection_cannot_beat_a_nearer_report_ahead(self):
    """FAITHFUL REPRODUCTION of the shipped defect.

    `path` is the whole current OSM way, so path[0] is the way's FIRST NODE -- which can be behind
    the car. geo.ahead()'s path branch measures `along` from path[0], not from us, and applies no
    cone and no perpendicular bound. Measured with this exact geometry:

        report 7.08 mi away, 87 deg off-axis  -> along =  1668 m, perp = 11377 m, source='path'
        report 3.00 mi straight ahead         -> along =  4835 m

    so the far, nearly-abeam report used to WIN on along_m. It must not."""
    path = [{"latitude": 46.99 + i * 0.003, "longitude": -122.0} for i in range(11)]
    far_abeam = {"lat": 47.005, "lon": -122.15, "magvar": None, "uuid": "abeam", "street": "",
                 "town": "T", "thumbs": 0, "ts": (_now_epoch() - 300.0) * 1000.0}
    out = _line([far_abeam, _alert("near", 3.0)], path=path)
    assert out["dist_mi"] == pytest.approx(3.0, abs=0.3), "the nearer report AHEAD must win"

  def test_path_projection_cannot_resurrect_a_report_behind_us(self):
    """The 11/77 shipped picks that were BEHIND the car. With path[0] behind us, a report behind the
    car but ahead of the way's start projects at along=556 m (perp 7585 m) and beat a 3 mi report
    straight ahead at along=4835 m."""
    path = [{"latitude": 46.99 + i * 0.003, "longitude": -122.0} for i in range(11)]
    behind = {"lat": 46.995, "lon": -122.10, "magvar": None, "uuid": "behind", "street": "",
              "town": "T", "thumbs": 0, "ts": (_now_epoch() - 300.0) * 1000.0}
    out = _line([behind, _alert("near", 3.0)], path=path)
    assert out["state"] == "alert"
    assert out["dist_mi"] == pytest.approx(3.0, abs=0.3), "a report BEHIND us must never be shown"

  def test_path_projection_cannot_reach_the_control_channel_either(self):
    """`cap` is the slowdown/siren channel and runs through the same _select."""
    path = [{"latitude": 46.99 + i * 0.003, "longitude": -122.0} for i in range(11)]
    behind_conf = {"lat": 46.995, "lon": -122.10, "magvar": None, "uuid": "bc", "street": "",
                   "town": "T", "thumbs": 9, "ts": (_now_epoch() - 60.0) * 1000.0}
    out = _line([behind_conf], path=path)
    assert out.get("cap") is None, "a CONFIRMED report behind us must not command a slowdown"

  def test_control_channel_also_refuses_a_report_behind_us(self):
    """`cap` is the only channel allowed to command a slowdown / siren, and it runs through the same
    _select. A CONFIRMED report behind the car must not become cap (it would fire for police already
    passed)."""
    out = _line([_alert("behind_confirmed", -0.4, age_min=1, thumbs=9)])
    assert out.get("cap") is None


class TestLastSeenMinutes:
  def test_live_report_reports_its_poll_age_not_its_publication_age(self):
    now = _now_epoch()
    al = _alert("live", 5.0, age_min=65)
    merged, _ = merge_retained_police({}, [al], now)
    out = _line(merged)
    assert out["age_min"] >= 60, "age_min stays the Waze publication age (tier grades on it)"
    assert out["last_seen_min"] == 0, "we have feed evidence from this poll -> 0 min since last seen"

  def test_retained_report_reports_when_it_left_the_feed(self):
    now = _now_epoch()
    al = _alert("ghost", 5.0, age_min=65)
    cache, _ = {}, None
    _, cache = merge_retained_police({}, [al], now - 600.0)     # last in the feed 10 min ago
    merged, _ = merge_retained_police(cache, [], now)           # gone from the feed now
    assert merged and merged[0].get("retained") is True
    out = _line(merged)
    assert out["last_seen_min"] == pytest.approx(10, abs=1)

  def test_missing_last_seen_yields_none_so_the_ui_falls_back(self):
    """An alert that never went through merge_retained_police must not fabricate a 0."""
    out = _line([_alert("raw", 5.0)])
    assert out["last_seen_min"] is None

  def test_last_seen_is_never_negative(self):
    now = _now_epoch()
    al = dict(_alert("skewed", 5.0), last_seen=now + 120.0)     # clock skew / future stamp
    out = _line([al])
    assert out["last_seen_min"] == 0


class TestForensics:
  def test_debug_entries_carry_relative_bearing(self, tmp_path, monkeypatch):
    """The 2026-09-08 root-cause had to recompute every report's bearing offline because the log did
    not store it."""
    monkeypatch.setattr(lsd, "_POLICE_DEBUG_PATH", str(tmp_path / "police_debug.jsonl"))
    lsd._police_dbg_last["sig"] = None
    _line([_alert("a", 5.0), _alert("b", -3.0)])
    import json
    recs = [json.loads(x) for x in open(tmp_path / "police_debug.jsonl") if x.strip()]
    assert recs, "the forensics log must have been written"
    rels = [r.get("rel") for r in recs[-1]["reports"]]
    assert all(x is not None for x in rels), "every report needs its relative bearing logged"
    assert max(rels) > 90.0, "the report behind us must be visible as such in the log"
