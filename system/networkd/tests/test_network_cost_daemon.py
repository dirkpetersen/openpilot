"""netcosttier2pnw — the daemon-side half of the cost ladder: reading cost, and the failure ledger.

These cover the two defects that live in the nmcli-facing code rather than in the pure ranker, both
found by review after the pure logic was already correct:

  * a TIMED-OUT metered read silently promoted a metered priority network back to tier 0, tearing
    down the cheaper link the ladder had just chosen -- a flap driven purely by nmcli flakiness;
  * an escalating backoff punished a REBOOTING ROUTER hardest, because the radio comes back before
    DHCP does, so the car sat on LTE in its own driveway long after the router was healthy.
"""
import openpilot.system.networkd.network_arbiterd as d
from openpilot.system.networkd.network_arbiter import priority_connection_id

PHONE, STARLINK = "Dirk's iPhone 13", "KarlMoik"
ID_PHONE, ID_STAR = priority_connection_id(PHONE), priority_connection_id(STARLINK)


class TestMeteredReadIsCached:
  """A failed nmcli read means NO NEW INFORMATION. It must not be laundered into 'unknown', because
  'unknown' is an answer with consequences -- it un-demotes a metered priority network."""

  def setup_method(self):
    d._metered_cache.clear()

  def test_a_successful_read_is_remembered(self, monkeypatch):
    monkeypatch.setattr(d, "_nmcli", lambda a: "connection.metered:yes")
    met, unmet = d._metered_states([ID_STAR], {STARLINK})
    assert met == {STARLINK} and unmet == set()
    assert d._metered_cache[STARLINK] == "yes"

  def test_a_FAILED_read_reuses_the_last_known_value(self, monkeypatch):
    """THE FLAP. Read once successfully, then fail: the network must stay metered, not silently
    become unknown and get promoted back to tier 0."""
    monkeypatch.setattr(d, "_nmcli", lambda a: "connection.metered:yes")
    d._metered_states([ID_STAR], {STARLINK})
    monkeypatch.setattr(d, "_nmcli", lambda a: None)          # nmcli times out
    met, _ = d._metered_states([ID_STAR], {STARLINK})
    assert met == {STARLINK}, "a timed-out read un-demoted a metered network"

  def test_a_failed_read_with_nothing_cached_stays_unknown(self, monkeypatch):
    """With no prior reading there is genuinely nothing to say, and it must not invent one."""
    monkeypatch.setattr(d, "_nmcli", lambda a: None)
    met, unmet = d._metered_states([ID_STAR], {STARLINK})
    assert met == set() and unmet == set()

  def test_it_only_asks_about_networks_that_could_be_chosen(self, monkeypatch):
    """The probe is one nmcli per candidate. Asking about every saved profile ever created, every
    20 s forever, is what this bound exists to prevent."""
    asked = []
    def fake(args):
      asked.append(args[-1])
      return "connection.metered:no"
    monkeypatch.setattr(d, "_nmcli", fake)
    d._metered_states([ID_PHONE, ID_STAR, "Hotspot", "lte"], {PHONE})
    assert asked == [ID_PHONE]

  def test_unknown_lands_in_neither_set(self, monkeypatch):
    monkeypatch.setattr(d, "_nmcli", lambda a: "connection.metered:unknown")
    met, unmet = d._metered_states([ID_STAR], {STARLINK})
    assert met == set() and unmet == set()


class TestTheLedger:
  def test_backoff_escalates_and_is_capped(self):
    ledger, now = {}, 1000.0
    got = []
    for _ in range(5):
      d._note_attempt(ledger, STARLINK, False, now)
      got.append(ledger[STARLINK.lower()][1] - now)
    assert got == [60.0, 300.0, 900.0, 900.0, 900.0], got
    assert max(got) <= 900.0, "an hour-long exile costs more than the retries it saves"

  def test_success_clears_it(self):
    ledger, now = {}, 1000.0
    d._note_attempt(ledger, STARLINK, False, now)
    d._note_attempt(ledger, STARLINK, True, now)
    assert ledger == {}
    assert d._blocked(ledger, now) == set()

  def test_a_reappearing_ssid_gets_a_clean_slate(self):
    """THE REBOOTING ROUTER. Radio comes back before DHCP, so the first attempts fail and the backoff
    escalates; by the time the router is healthy the car would sit out the sentence."""
    ledger, absent, now = {}, {}, 1000.0
    for _ in range(3):
      d._note_attempt(ledger, "Hannelore", False, now)
    assert d._blocked(ledger, now) == {"hannelore"}
    for _ in range(d.ABSENT_SCANS_FOR_FRESH_START):      # genuinely out of range, real scans
      d._forget_on_reappearance(ledger, absent, {"karlmoik"})
    d._forget_on_reappearance(ledger, absent, {"hannelore"})   # ...and back
    assert ledger == {}, "a router that came back is still serving its sentence"

  def test_a_SUPPRESSED_scan_is_not_absence(self):
    """FABLE: the geo-gate stops scanning whenever we are already on client WiFi away from a learned
    location -- normal operation, not absence. Reading that as 'every network went out of range'
    wiped the ledger every other tick and collapsed the backoff to the scan-flicker rate."""
    ledger, absent, now = {}, {}, 1000.0
    for _ in range(3):
      d._note_attempt(ledger, "Hannelore", False, now)
    for _ in range(10):
      d._forget_on_reappearance(ledger, absent, None)     # no scan ran
    assert d._blocked(ledger, now) == {"hannelore"}, "a suppressed scan wiped the backoff"

  def test_one_missing_scan_result_is_not_absence_either(self):
    """APs drop out of a single scan routinely. Absence has to be sustained to count as news."""
    ledger, absent, now = {}, {}, 1000.0
    d._note_attempt(ledger, "Hannelore", False, now)
    d._forget_on_reappearance(ledger, absent, {"karlmoik"})    # missing once
    d._forget_on_reappearance(ledger, absent, {"hannelore"})   # back immediately
    assert "hannelore" in ledger

  def test_an_ssid_that_never_left_keeps_its_record(self):
    """A network continuously in range and continuously failing must keep escalating."""
    ledger, absent, now = {}, {}, 1000.0
    d._note_attempt(ledger, "Hannelore", False, now)
    for _ in range(5):
      d._forget_on_reappearance(ledger, absent, {"hannelore"})
    assert "hannelore" in ledger

  def test_blocked_expires_on_its_own(self):
    ledger = {}
    d._note_attempt(ledger, STARLINK, False, 1000.0)
    assert d._blocked(ledger, 1030.0) == {STARLINK.lower()}
    assert d._blocked(ledger, 1061.0) == set()


class TestLinkUsability:
  """TRI-STATE. True / False / None-we-could-not-tell."""

  @staticmethod
  def _nm(addr="IP4.ADDRESS[1]:192.168.1.79/24", conn="4 (full)"):
    def fake(args):
      return conn if "GENERAL.IP4-CONNECTIVITY" in args else addr
    return fake

  def test_an_address_and_full_connectivity_means_usable(self, monkeypatch):
    monkeypatch.setattr(d, "_nmcli", self._nm())
    assert d._client_link_usable(ID_PHONE) is True

  def test_an_address_with_NO_UPSTREAM_is_not_usable(self, monkeypatch):
    """A cafe portal we never accepted, a router whose ISP is down, an obstructed Starlink: all hold
    an address. wlan0's default route is metric 600 against wwan0's 1000, so such a link wins the
    route and black-holes the DEVICE's own traffic, not just tethered clients."""
    for bad in ("1 (none)", "2 (limited)", "none", "limited"):
      monkeypatch.setattr(d, "_nmcli", self._nm(conn=bad))
      assert d._client_link_usable(ID_PHONE) is False, bad

  def test_a_portal_link_WITH_a_handler_stays_usable(self, monkeypatch):
    """The exemption exists for exactly this: the auto-accept path has to be ON the network to POST
    the form, so demoting `portal` would tear the link down before the handler could ever run."""
    monkeypatch.setattr(d, "_nmcli", self._nm(conn="3 (portal)"))
    assert d._client_link_usable(ID_PHONE, has_portal_handler=True) is True

  def test_a_portal_link_WITHOUT_a_handler_is_a_black_hole(self, monkeypatch):
    """THE HOLE THE EXEMPTION RE-OPENED. A hotel WiFi joined once is still saved, so the ladder can
    pick it -- and with no accept handler there is nothing to protect. Measured: the device parked on
    it sticky and 'usable', hotspot down, wlan0 holding the default route at metric 600, forever,
    with no log line at all. That is the failure the connectivity check was added to close."""
    monkeypatch.setattr(d, "_nmcli", self._nm(conn="3 (portal)"))
    assert d._client_link_usable(ID_PHONE, has_portal_handler=False) is False

  def test_an_unreadable_connectivity_value_is_UNKNOWN(self, monkeypatch):
    def fake(args):
      return None if "GENERAL.IP4-CONNECTIVITY" in args else "IP4.ADDRESS[1]:192.168.1.79/24"
    monkeypatch.setattr(d, "_nmcli", fake)
    assert d._client_link_usable(ID_PHONE) is None

  def test_associated_with_no_address_is_NOT_usable(self, monkeypatch):
    """NM reports 'activated' on association; a dead DHCP server looks connected forever."""
    monkeypatch.setattr(d, "_nmcli", self._nm(addr="IP4.ADDRESS[1]:"))
    assert d._client_link_usable(ID_PHONE) is False

  def test_a_failed_read_is_UNKNOWN_not_dead(self, monkeypatch):
    """THE TEST THAT USED TO PIN THE WRONG BEHAVIOUR. It asserted `is False` -- that an nmcli
    timeout means the link is dead. Measured consequence: one flaky read recorded a failure against
    a working link, un-stuck it, and handed the radio to the hotspot on the next tick. An ERROR is
    not a NEGATIVE RESULT."""
    monkeypatch.setattr(d, "_nmcli", lambda a: None)
    assert d._client_link_usable(ID_PHONE) is None


class TestLedgerKeying:
  """FABLE: the same network reached the ledger under two different spellings -- the CONFIGURED case
  ("visitor") when it was a tier-0 pending target, and the AP's case ("Visitor") from ssid_of() on
  the per-tick verdict. Two entries for one network, and a success on one never cleared the other."""

  def test_one_network_cannot_hold_two_ledger_entries(self):
    ledger, now = {}, 1000.0
    d._note_attempt(ledger, "visitor", False, now)
    d._note_attempt(ledger, "Visitor", False, now)
    assert len(ledger) == 1, f"one network, two entries: {ledger}"
    assert ledger["visitor"][0] == 2, "the second failure did not escalate the first"

  def test_a_success_in_either_spelling_clears_it(self):
    ledger, now = {}, 1000.0
    d._note_attempt(ledger, "visitor", False, now)
    d._note_attempt(ledger, "VISITOR", True, now)
    assert ledger == {}
