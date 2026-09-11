"""netcosttier2pnw — LOOP-LEVEL sequence tests: main() driven tick by tick against a fake NM.

WHY THIS FILE EXISTS, and it is not a style preference. Every pure function in this feature is
pinned by unit tests, and that was not enough twice:

  * a revision of this change shipped with 100 green tests while `_metered_states` had no `return`
    statement -- it returned None, every tick raised inside the loop's `except Exception`, and the
    arbiter was alive and doing NOTHING. No unit test touched the daemon.
  * a later revision reintroduced a defect a reviewer had already found and I had already fixed --
    an outright `con up` failure going unrecorded, so the hotspot was never re-raised -- with 118
    green tests. The regression was entirely in the loop's glue between tested functions.

So these tests drive `network_arbiterd.main()` itself, with nmcli, params, the clock, the geo-gate
and the modem all faked, and assert on the SEQUENCE of connections the arbiter actually brings up.
The scenarios are the measured failure modes from review; each one is a bug that shipped in some
revision of this branch. Adapted from the review harness.
"""
import json
import subprocess

import pytest

import openpilot.system.networkd.network_arbiterd as d

PHONE, STAR, HOME = "Dirk's iPhone 13", "KarlMoik", "Hannelore"
HOTSPOT = "Hotspot"


class FakeNM:
  """Just enough NetworkManager to drive the loop. `behave` scripts how each profile responds to
  `con up`: ok / refuse (association fails, nothing active) / no_dhcp (associates, no address)."""

  def __init__(self):
    self.active: str | None = None
    self.saved = [HOTSPOT, d.priority_connection_id(HOME), d.priority_connection_id(PHONE),
                  d.priority_connection_id(STAR), "lte"]
    self.scan: list[str] = []
    self.metered: dict[str, str] = {}
    self.ip: dict[str, str | None] = {}
    self.conn: dict[str, str] = {}
    self.behave: dict[str, object] = {}
    self.fail_reads: set[str] = set()
    self.t = 0.0
    self.ups: list[str] = []

  def nmcli(self, args):
    a = " ".join(args)
    if any(k in a for k in self.fail_reads):
      return None
    if "--active" in a:
      return f"{self.active}:802-11-wireless:wlan0\n" if self.active else ""
    if "dev wifi list" in a:
      return "\n".join(self.scan) + "\n"
    if a == "-t -f NAME con show":
      return "\n".join(self.saved) + "\n"
    if "connection.metered" in a:
      return f"connection.metered:{self.metered.get(d.ssid_of(args[-1]), 'unknown')}\n"
    if "IP4.ADDRESS" in a:
      c = args[-1]
      return f"IP4.ADDRESS[1]:{self.ip[c]}/24\n" if self.active == c and self.ip.get(c) else ""
    if "GENERAL.IP4-CONNECTIVITY" in a:
      return self.conn.get(self.active, "4 (full)") + "\n"
    if "CONNECTIVITY" in a:
      return "full\n"
    if args[:2] == ["con", "up"]:
      c = args[2]
      self.ups.append(c)
      if c == HOTSPOT:
        self.active = HOTSPOT
        return ""
      b = self.behave.get(c, "ok")
      if callable(b):
        b = b(self.t)
      if b == "refuse":
        self.active = None
        return None
      self.active = c
      self.ip[c] = "10.0.0.2" if b == "ok" else None
      return ""
    if args[:2] == ["con", "down"]:
      if self.active == args[2]:
        self.active = None
      return ""
    if args[:3] == ["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY"]:
      return ""
    return ""


def run_loop(monkeypatch, nm, ticks=8, near_home=True, hooks=(), priority=(HOME,), ladder=True):
  """Drive main() for `ticks` polls. Returns the list of connections it brought up, in order."""
  state = {"n": 0}
  monkeypatch.setattr(d, "_nmcli", nm.nmcli)
  monkeypatch.setattr(d, "_run", lambda args: subprocess.CompletedProcess(args, 0, "", ""))
  monkeypatch.setattr(d, "_modem_index", lambda: None)
  monkeypatch.setattr(d, "_lte_throttled_recently", lambda: False)
  monkeypatch.setattr(d, "_lte_has_ip", lambda: False)
  monkeypatch.setattr(d, "_read_gps", lambda p, m: (47.0, -122.0))
  monkeypatch.setattr(d, "near_any_home", lambda locs, gps: near_home)
  monkeypatch.setattr(d, "_usable_cache", {})

  nets = [{"label": "Home", "ssid": s, "lat": 47.0, "lon": -122.0} for s in priority]

  class P:
    def get_bool(self, k):
      if k == "DisableNetworkCostLadder":
        return not ladder
      return k == "TetheringEnabled"

    def get(self, k):
      return json.dumps(nets) if k == "TetheringPriorityNetworks" else None

    def put(self, *a):
      pass

    def put_bool(self, *a):
      pass

  monkeypatch.setattr(d, "Params", lambda *a, **k: P())

  def fake_sleep(sec):
    nm.t += sec
    state["n"] += 1
    for h in hooks:
      h(nm, state["n"])
    if state["n"] >= ticks:
      raise SystemExit

  monkeypatch.setattr(d.time, "sleep", fake_sleep)
  monkeypatch.setattr(d.time, "monotonic", lambda: float(nm.t))
  with pytest.raises(SystemExit):
    d.main()
  return nm.ups


class TestTheArbiterAlwaysHasAnUplink:
  """The property that matters more than which tier wins: the device must never end up with no
  working connection and no plan to get one."""

  def test_a_network_that_REFUSES_to_associate_falls_back_to_the_hotspot(self, monkeypatch):
    """`up_fallback` drops the hotspot BEFORE raising the client, and a refused association leaves
    NOTHING active -- so this failure is invisible unless the pending bring-up is judged. A revision
    of this branch shipped without that and retried forever with the hotspot down, tethered clients
    dark. Measured: 5 attempts, 0 hotspot re-raises."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "refuse"
    ups = run_loop(monkeypatch, nm, ticks=8, priority=())
    assert HOTSPOT in ups, f"never re-raised the hotspot after a refused association: {ups}"

  def test_and_it_backs_off_instead_of_hammering(self, monkeypatch):
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "refuse"
    ups = run_loop(monkeypatch, nm, ticks=8, priority=())
    client_ups = [u for u in ups if u != HOTSPOT]
    assert len(client_ups) <= 3, f"retried a refusing network every tick: {ups}"


class TestItDoesNotTearDownWorkingLinks:
  """Every one of these dropped a perfectly good connection in some revision."""

  def test_a_link_still_doing_DHCP_is_left_alone(self, monkeypatch):
    """NM's DHCP timeout is 45 s, the poll is 20 s, so the first look lands mid-activation."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    nm.behave[d.priority_connection_id(PHONE)] = "no_dhcp"
    hooks = [lambda nm, tk: nm.ip.__setitem__(d.priority_connection_id(PHONE), "10.0.0.9")
             if tk == 3 and nm.active == d.priority_connection_id(PHONE) else None]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert ups.count(HOTSPOT) == 0, f"tore down a link that was mid-DHCP: {ups}"

  def test_one_unreadable_nmcli_call_does_not_cost_us_the_radio(self, monkeypatch):
    """An ERROR is not a NEGATIVE RESULT. A revision folded a read timeout into 'dead' and handed
    the radio to the hotspot on the next tick."""
    nm = FakeNM()
    nm.active = d.priority_connection_id(PHONE)
    nm.ip[nm.active] = "10.0.0.2"
    nm.scan = []
    hooks = [lambda nm, tk: nm.fail_reads.add("IP4.ADDRESS") if tk == 2
             else nm.fail_reads.discard("IP4.ADDRESS")]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert HOTSPOT not in ups, f"one flaky read cost a working link: {ups}"

  def test_a_stale_cached_reading_does_not_either(self, monkeypatch):
    """The cache is a memory of the LAST time we could read a network, not a current fact."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    monkeypatch.setattr(d, "_usable_cache", {PHONE.lower(): False})
    hooks = [lambda nm, tk: nm.fail_reads.add("IP4.ADDRESS") if tk == 1
             else nm.fail_reads.discard("IP4.ADDRESS")]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert ups.count(HOTSPOT) == 0, f"a stale cache entry tore down a good link: {ups}"

  def test_a_network_the_driver_joined_himself_is_not_dropped_mid_DHCP(self, monkeypatch):
    nm = FakeNM()
    nm.active = d.priority_connection_id(STAR)
    nm.ip[nm.active] = None
    nm.scan = []
    nm.metered[STAR] = "yes"
    hooks = [lambda nm, tk: nm.ip.__setitem__(d.priority_connection_id(STAR), "10.0.0.5")
             if tk == 1 and nm.active == d.priority_connection_id(STAR) else None]
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, hooks=hooks, priority=())
    assert HOTSPOT not in ups, f"dropped the driver's own manual join: {ups}"


class TestItDoesNotParkOnADeadLink:
  def test_a_captive_portal_with_no_handler_does_not_hold_the_radio(self, monkeypatch):
    """A hotel WiFi joined once is still saved, so the ladder can pick it. With an address but no
    upstream it wins the default route (wlan0 metric 600 vs wwan0 1000) and black-holes the DEVICE's
    own traffic. Measured in a revision: parked on it indefinitely with no log line at all."""
    nm = FakeNM()
    nm.active = HOTSPOT
    nm.saved.append(d.priority_connection_id("HotelWifi"))
    nm.scan = ["HotelWifi"]
    nm.conn[d.priority_connection_id("HotelWifi")] = "3 (portal)"
    ups = run_loop(monkeypatch, nm, ticks=6, near_home=False, priority=())
    assert ups.count(HOTSPOT) >= 1, f"parked on a portal network with no handler: {ups}"


class TestTheKillSwitch:
  def test_the_ladder_runs_by_default(self, monkeypatch):
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    ups = run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(), ladder=True)
    assert d.priority_connection_id(PHONE) in ups

  def test_and_the_param_turns_it_off_without_a_deploy(self, monkeypatch):
    """A feature that can hold the radio needs a revert that does not need the radio."""
    nm = FakeNM()
    nm.active, nm.scan = HOTSPOT, [PHONE]
    nm.metered[PHONE] = "no"
    ups = run_loop(monkeypatch, nm, ticks=3, near_home=False, priority=(), ladder=False)
    assert d.priority_connection_id(PHONE) not in ups, f"kill switch ignored: {ups}"
