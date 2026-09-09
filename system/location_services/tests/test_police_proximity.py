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
    with open(tmp_path / "police_debug.jsonl") as f:      # NOT a bare open(): pyproject addopts
      recs = [json.loads(x) for x in f if x.strip()]      # carries -Werror, so a leaked handle's
                                                          # ResourceWarning FAILS the test (Fable)
    assert recs, "the forensics log must have been written"
    rels = [r.get("rel") for r in recs[-1]["reports"]]
    assert all(x is not None for x in rels), "every report needs its relative bearing logged"
    assert max(rels) > 90.0, "the report behind us must be visible as such in the log"


class TestGeminiReviewFindings:
  """Defects found by the Gemini review of the first cut of this change (2026-09-08)."""

  def test_a_nan_coordinate_cannot_swallow_every_valid_report(self):
    """REAL REGRESSION vs the shipped code. json.loads accepts a bare NaN literal and float("nan")
    does not raise, so a bad proxy coordinate reaches _select as NaN. Every NaN comparison is False,
    so the range and hemisphere tests do NOT reject it, and min() returns it when it sorts first --
    silently discarding every valid report. The old code skipped it only incidentally (its
    `NaN < best_along` is also False)."""
    nan_alert = {"lat": float("nan"), "lon": -122.0, "magvar": None, "uuid": "nan1", "street": "",
                 "town": "T", "thumbs": 0, "ts": (_now_epoch() - 300.0) * 1000.0}
    out = _line([nan_alert, _alert("good", 3.0)])          # NaN FIRST -- the poisoning order
    assert out["state"] == "alert", "a NaN report must not suppress the overlay"
    assert out["dist_mi"] == pytest.approx(3.0, abs=0.2)
    assert out["dist_mi"] == out["dist_mi"], "published distance must never be NaN"

  def test_a_nan_only_feed_yields_no_alert_not_a_nan_distance(self):
    nan_alert = {"lat": float("nan"), "lon": float("nan"), "magvar": None, "uuid": "nan2",
                 "street": "", "town": "T", "thumbs": 0, "ts": (_now_epoch() - 300.0) * 1000.0}
    out = _line([nan_alert])
    assert out["state"] != "alert"

  def test_ranking_uses_unrounded_distance(self):
    """recede.live_mi rounds to 0.1 mi, so ranking on it makes 4.04 and 3.96 tie at 4.0 and the pick
    flip as they separate. Ordering must use the raw distance."""
    a = _alert("aaa", 4.04)      # sorts FIRST on uuid, so a rounded tie would pick it
    b = _alert("bbb", 3.96)
    out = _line([a, b])
    assert out["dist_mi"] == pytest.approx(4.0, abs=0.05)
    # the published value is rounded, so assert identity instead: the nearer report must win
    import openpilot.system.location_services.location_servicesd as _l
    recede = _l._PoliceRecede(_l.POLICE_RECEDE_MI)
    line = _l._line_police([a, b], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert line["uuid"] == "bbb", "the genuinely nearer report must win, not the uuid tie-break"

  def test_cap_carries_last_seen_min(self):
    """The banner renders {**p, **cap}; a field on the display line but missing from cap is
    inherited from the OTHER report -- the splice a previous review round fixed for dir/town."""
    now = _now_epoch()
    confirmed = _alert("conf", 0.4, age_min=1, thumbs=9)
    merged, _ = merge_retained_police({}, [confirmed], now)
    out = _line(merged)
    assert out.get("cap") is not None
    assert "last_seen_min" in out["cap"], "cap must carry last_seen_min or the banner splices"
    assert out["cap"]["last_seen_min"] == 0


class TestHemisphereHysteresis:
  """policenear2-2pnw, Fable review 2026-09-08. A hard 90-degree edge made a report sitting abeam
  toggle in and out every 2-3 s. Replaying the new rule at 1 Hz against the drive's GPS: strict 90
  gave 37 pick changes / 11 flip-backs within 30 s / 9 displays under 5 s; with a 15-degree hold for
  the report already shown, 16 / 0 / 0. recede-tracking does NOT bound this -- it retires reports you
  drive PAST, and a report abeam on a parallel road never recedes past closest approach."""

  @staticmethod
  def _at(bearing_deg, mi):
    """A report `mi` away at an absolute bearing from (47.0, -122.0)."""
    import math
    R = 3958.8
    b, d = math.radians(bearing_deg), mi / R
    p1, l1 = math.radians(47.0), math.radians(-122.0)
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(b))
    l2 = l1 + math.atan2(math.sin(b) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * math.sin(p2))
    return math.degrees(p2), math.degrees(l2)

  def _report(self, uuid, bearing_deg, mi):
    la, lo = self._at(bearing_deg, mi)
    return {"lat": la, "lon": lo, "magvar": None, "uuid": uuid, "street": "", "town": "T",
            "thumbs": 0, "ts": (_now_epoch() - 300.0) * 1000.0}

  def test_the_shown_report_is_held_past_90_degrees(self):
    """The measured case: 5.4 mi at ~86-89.5 deg toggling against 7.2 mi at 74 deg."""
    recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
    near = self._report("near", 88.0, 5.4)
    far = self._report("far", 74.0, 7.2)
    a = _line_police([near, far], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert a["uuid"] == "near", "inside 90 deg the nearer report wins"
    # it drifts just past the hard edge -- must NOT hand the slot to the farther report
    near2 = self._report("near", 96.0, 5.4)
    b = _line_police([near2, far], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert b["uuid"] == "near", "the shown report is held out to 105 deg, so no toggle"

  def test_a_report_never_shown_is_not_admitted_past_90(self):
    recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
    out = _line_police([self._report("fresh", 96.0, 2.0), self._report("ahead", 10.0, 9.0)],
                       "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert out["uuid"] == "ahead", "the hold applies ONLY to the incumbent"

  def test_a_nearer_report_still_takes_the_slot_immediately(self):
    """Admission-only: the hold must never make a farther report win."""
    recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
    held = self._report("held", 88.0, 5.0)
    _line_police([held], "ok", "", 47.0, -122.0, 0.0, [], recede)
    held2 = self._report("held", 96.0, 5.0)
    out = _line_police([held2, self._report("closer", 20.0, 2.0)],
                       "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert out["uuid"] == "closer", "ranking is untouched -- a nearer report wins at once"

  def test_the_two_channels_hold_separately(self):
    """display and cap must not share a hysteresis slot, or the cap pick would hold the DISPLAY's
    report out to 105 deg (and vice versa). Set them up to differ: a near UNCONFIRMED report takes
    the display line while a farther CONFIRMED one takes cap."""
    recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
    la1, lo1 = self._at(10.0, 3.0)
    near_unconf = {"lat": la1, "lon": lo1, "magvar": None, "uuid": "near_unconf", "street": "",
                   "town": "T", "thumbs": 0, "ts": (_now_epoch() - 3600.0) * 1000.0}   # 60 min -> unconfirmed
    la2, lo2 = self._at(10.0, 6.0)
    far_conf = {"lat": la2, "lon": lo2, "magvar": None, "uuid": "far_conf", "street": "",
                "town": "T", "thumbs": 9, "ts": (_now_epoch() - 60.0) * 1000.0}         # 1 min -> confirmed
    out = _line_police([near_unconf, far_conf], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert out["uuid"] == "near_unconf", "display is proximity-first over the whole set"
    assert out["cap"]["uuid"] == "far_conf", "cap is the nearest CONFIRMED report"
    assert recede.last_pick.get("display") == "near_unconf"
    assert recede.last_pick.get("cap") == "far_conf", "the channels must hold different reports"

  def test_the_hold_is_released_when_nothing_is_selectable(self):
    recede = lsd._PoliceRecede(lsd.POLICE_RECEDE_MI)
    _line_police([self._report("x", 88.0, 5.0)], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert recede.last_pick.get("display") == "x"
    _line_police([], "ok", "", 47.0, -122.0, 0.0, [], recede)
    assert recede.last_pick.get("display") is None, "a stale hold must not survive an empty tick"
