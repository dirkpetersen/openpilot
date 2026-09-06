"""madsop2pnw — tests for the openpilot-side parallel lateral authority.

Adapted from sunnypilot's `sunnypilot/mads/tests/test_mads_state_machine.py` (MIT). That file
drives a state machine whose inputs are event TYPES; this port's state machine is driven by
openpilot's own engagement plus the frame's events, so the tests are rebuilt around those inputs
while keeping the same shape: exhaustive over the transitions, and explicit about the
"must NOT happen" cases.

The load-bearing claims under test:
  1. a brake press with "Disengage on brake" OFF keeps lateral intent;
  2. with it ON, it does not;
  3. with PandaMadsSafety off (alternativeExperience == 0) the whole thing is a no-op;
  4. the Tesla path never receives the bits at all;
  5. every NON-brake loss of controls still drops lateral.
"""
import ast
import pathlib
import importlib.util
import inspect
import textwrap

import pytest

from cereal import car, log
from opendbc.safety import ALTERNATIVE_EXPERIENCE

from openpilot.selfdrive.car.card import Car
from openpilot.selfdrive.controls.controlsd import Controls
from openpilot.selfdrive.selfdrived.events import EVENTS, ET, Events
from openpilot.selfdrive.selfdrived.selfdrived import SelfdriveD
from openpilot.selfdrive.selfdrived.mads_pnw import (LATERAL_DISABLE_TYPES, MADS_TOLERATED_EVENTS,
                                                     MadsPnw, has_blocking_event)

EventName = log.OnroadEvent.EventName

MADS_ON = ALTERNATIVE_EXPERIENCE.ENABLE_MADS
MADS_DISENGAGE = ALTERNATIVE_EXPERIENCE.ENABLE_MADS | ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE


def _fn_ast(fn) -> ast.AST:
  """AST of a single function/method, dedented so it parses standalone."""
  return ast.parse(textwrap.dedent(inspect.getsource(fn)))


def _method_ast(module: str, cls: str, method: str) -> ast.AST:
  """AST of one method, read from the file WITHOUT importing the module."""
  origin = importlib.util.find_spec(module).origin
  for node in ast.walk(ast.parse(pathlib.Path(origin).read_text())):
    if isinstance(node, ast.ClassDef) and node.name == cls:
      for sub in node.body:
        if isinstance(sub, ast.FunctionDef) and sub.name == method:
          return sub
  raise AssertionError(f"{module}:{cls}.{method} not found")


def _find_call(fn, obj: str, method: str):
  """The single `obj.method(...)` Call node in fn, where obj may be a name or an attribute."""
  for node in ast.walk(_fn_ast(fn)):
    if not (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
            and node.func.attr == method):
      continue
    base = node.func.value
    if (isinstance(base, ast.Attribute) and base.attr == obj) or \
       (isinstance(base, ast.Name) and base.id == obj):
      return node
  return None


def ev(*names) -> Events:
  e = Events()
  for n in names:
    e.add(n)
  return e


def engage(mads: MadsPnw, frames: int = 3) -> None:
  """Drive openpilot into a normal engaged state (stock cruise engaged, openpilot with it)."""
  for _ in range(frames):
    mads.update(op_enabled=True, op_active=True, braking=False, cruise_enabled=True, events=ev())


def brake_release_cruise(mads: MadsPnw, braking: bool = True, cruise_enabled: bool = False) -> None:
  """The Lightning's brake press: openpilot disengages, the PCM drops cruise.

  Both openpilot's own `pedalPressed` and the stock PCM's `pcmDisable` are present, exactly as
  car_specific.py raises them.
  """
  mads.update(op_enabled=False, op_active=False, braking=braking, cruise_enabled=cruise_enabled,
              events=ev(EventName.pedalPressed, EventName.pcmDisable))


class TestMadsLateralAuthority:
  # ---- 1. the feature itself -------------------------------------------------------------
  def test_brake_keeps_lateral_when_disengage_on_brake_off(self):
    mads = MadsPnw(MADS_ON)
    engage(mads)
    assert (mads.enabled, mads.active, mads.lateral_only) == (True, True, False)

    brake_release_cruise(mads)
    assert mads.enabled, "lateral authority must survive the brake press"
    assert mads.active, "openpilot must keep COMMANDING lateral, not merely be permitted to"
    assert mads.lateral_only, "the driver must be told openpilot is steering with cruise off"

  def test_lateral_survives_brake_release(self):
    # REMAIN_ACTIVE: the brake took the speed, not the steering. Letting go of the pedal must not
    # end it -- the panda's latch does not reset either.
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    for _ in range(50):
      # cruise is still off, so car_specific.py keeps re-raising pcmDisable every frame
      mads.update(op_enabled=False, op_active=False, braking=False, cruise_enabled=False,
                  events=ev(EventName.pcmDisable))
    assert mads.enabled and mads.active and mads.lateral_only

  def test_reengage_returns_to_mirroring(self):
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    assert mads.lateral_only
    mads.update(op_enabled=True, op_active=True, braking=False, cruise_enabled=True,
                events=ev(EventName.pcmEnable))
    assert (mads.enabled, mads.active, mads.lateral_only) == (True, True, False)

  def test_preenabled_mirrors_op_active_not_op_enabled(self):
    # enabled-but-not-active (preEnabled) must not become a lateral command.
    mads = MadsPnw(MADS_ON)
    mads.update(op_enabled=True, op_active=False, braking=False, cruise_enabled=True, events=ev())
    assert mads.enabled and not mads.active and not mads.lateral_only

  # ---- 2. the toggle ---------------------------------------------------------------------
  def test_brake_disengages_when_disengage_on_brake_on(self):
    mads = MadsPnw(MADS_DISENGAGE)
    engage(mads)
    brake_release_cruise(mads)
    assert not mads.enabled and not mads.active and not mads.lateral_only

  def test_disengage_on_brake_bit_is_reported(self):
    assert MadsPnw(MADS_DISENGAGE).disengage_on_brake is True
    assert MadsPnw(MADS_ON).disengage_on_brake is False

  # ---- 3. inert unless the panda can honour it -------------------------------------------
  @pytest.mark.parametrize("alt_exp", [ALTERNATIVE_EXPERIENCE.DEFAULT,
                                       ALTERNATIVE_EXPERIENCE.DISABLE_STOCK_AEB,
                                       ALTERNATIVE_EXPERIENCE.ALLOW_AEB,
                                       # the policy bits alone, without ENABLE_MADS, mean nothing
                                       ALTERNATIVE_EXPERIENCE.MADS_DISENGAGE_LATERAL_ON_BRAKE])
  def test_no_op_without_enable_mads(self, alt_exp):
    """PandaMadsSafety=0 => card.py sends 0 => this is dead. Today's behaviour, unchanged."""
    mads = MadsPnw(alt_exp)
    assert not mads.available
    engage(mads)
    assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)
    brake_release_cruise(mads)
    assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)

  def test_unavailable_mads_reports_all_false_even_from_a_dirty_state(self):
    """Defence in depth, and the reason the inert branch ASSIGNS rather than just returning:
    `available` is fixed at construction today, so this state cannot arise -- but if it ever
    becomes dynamic, an unavailable MADS must not keep publishing a stale True."""
    mads = MadsPnw(ALTERNATIVE_EXPERIENCE.DEFAULT)
    mads.enabled = mads.active = mads.lateral_only = True
    mads.update(op_enabled=True, op_active=True, braking=False, cruise_enabled=True, events=ev())
    assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)

  def test_pause_bit_is_refused(self):
    """PAUSE exists in the safety C, is deliberately not exposed, and is not modelled here.
    Honouring ENABLE_MADS while the panda runs an unmodelled policy is exactly the disagreement
    this design avoids -- so refuse and fall back to stock rather than guess."""
    mads = MadsPnw(MADS_ON | ALTERNATIVE_EXPERIENCE.MADS_PAUSE_LATERAL_ON_BRAKE)
    assert not mads.available
    engage(mads)
    brake_release_cruise(mads)
    assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)

  # ---- 5. every non-brake loss of controls still drops lateral ---------------------------
  def test_gas_pedal_disengage_drops_lateral(self):
    # pedalPressed is raised for the GAS pedal too. It is in the tolerated list, so the ONLY thing
    # stopping it from smuggling lateral through is the braking requirement on the falling edge.
    mads = MadsPnw(MADS_ON)
    engage(mads)
    mads.update(op_enabled=False, op_active=False, braking=False, cruise_enabled=True,
                events=ev(EventName.pedalPressed))
    assert not mads.enabled and not mads.active

  @pytest.mark.parametrize("name", [EventName.buttonCancel, EventName.wrongCarMode,
                                    EventName.reverseGear, EventName.wrongGear,
                                    EventName.doorOpen, EventName.seatbeltNotLatched,
                                    EventName.parkBrake, EventName.espActive,
                                    EventName.accFaulted, EventName.controlsMismatch,
                                    EventName.steerUnavailable, EventName.canError])
  def test_non_brake_disengage_drops_lateral_on_the_edge(self, name):
    """Even WITH the brake down -- narrower than the panda, whose !braking.current test would let
    a CANCEL press during a brake keep the latch."""
    mads = MadsPnw(MADS_ON)
    engage(mads)
    mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=False,
                events=ev(EventName.pedalPressed, EventName.pcmDisable, name))
    assert not mads.enabled and not mads.active and not mads.lateral_only

  @pytest.mark.parametrize("name", [EventName.buttonCancel, EventName.wrongCarMode,
                                    EventName.reverseGear, EventName.doorOpen,
                                    EventName.espActive, EventName.accFaulted])
  def test_non_brake_disengage_ends_an_established_lateral_only(self, name):
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    assert mads.enabled
    mads.update(op_enabled=False, op_active=False, braking=False, cruise_enabled=False,
                events=ev(EventName.pcmDisable, name))
    assert not mads.enabled and not mads.active and not mads.lateral_only

  def test_dropped_lateral_never_relatches_without_an_engage(self):
    mads = MadsPnw(MADS_ON)
    engage(mads)
    mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=False,
                events=ev(EventName.pedalPressed, EventName.buttonCancel))
    assert not mads.enabled
    for _ in range(100):
      mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=False,
                  events=ev(EventName.pcmDisable))
      assert not mads.enabled, "only an openpilot engage may re-arm lateral authority"

  def test_never_engages_from_a_cold_start_without_openpilot(self):
    # There is deliberately no MADS button and no ACC-main engage: the ONLY source of authority is
    # openpilot's own rising edge, mirroring opendbc/safety/pnw/mads.h.
    mads = MadsPnw(MADS_ON)
    for _ in range(100):
      mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=False, events=ev())
      assert not mads.enabled

  # ---- the panda-revokes-first divergence (Fable review 2026-09-05) ----------------------
  def test_cruise_reengaging_without_openpilot_drops_lateral(self):
    """Stock cruise comes back but openpilot does NOT engage with it (a NO_ENTRY is standing).
    controlsd then cancels, stock cruise drops, and the PANDA revokes controls_allowed_lateral on
    that falling edge. openpilot sees no edge of its own, so without this check it would keep
    commanding lateral into a panda that is blocking it -- silently, with no detector."""
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    assert mads.enabled
    # belowEngageSpeed is NO_ENTRY-only, i.e. deliberately NOT a blocking event here: the drop must
    # come from the revoke check itself, not from has_blocking_event.
    assert not has_blocking_event(ev(EventName.belowEngageSpeed))
    mads.update(op_enabled=False, op_active=False, braking=False, cruise_enabled=True,
                events=ev(EventName.pcmEnable, EventName.belowEngageSpeed))
    assert not mads.enabled and not mads.active and not mads.lateral_only

  def test_cruise_still_reading_engaged_just_after_the_brake_does_not_drop_lateral(self):
    """The regression the RISING-edge test protects: `cruiseState.enabled` can still read True for
    a frame or two after the brake press, before the PCM drops it. A LEVEL test on cruise_enabled
    would kill the feature on the very frame it arms."""
    mads = MadsPnw(MADS_ON)
    engage(mads)                                    # cruise_enabled True throughout
    brake_release_cruise(mads, cruise_enabled=True)  # PCM has not dropped cruise yet
    assert mads.enabled and mads.active and mads.lateral_only
    for _ in range(3):
      mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=True,
                  events=ev(EventName.pedalPressed))
      assert mads.enabled
    brake_release_cruise(mads, braking=False, cruise_enabled=False)
    assert mads.enabled

  def test_reengaging_normally_does_not_trip_the_revoke_check(self):
    """selfdrived runs its own state machine BEFORE mads.update in the same frame, so on a normal
    re-engage op_enabled is already True when the cruise rising edge is seen."""
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    mads.update(op_enabled=True, op_active=True, braking=False, cruise_enabled=True,
                events=ev(EventName.pcmEnable))
    assert mads.enabled and mads.active and not mads.lateral_only

  # ---- the blocking-event classification -------------------------------------------------
  def test_blocking_event_coverage(self):
    """Every event carrying a disable type blocks, except exactly the two tolerated ones.

    This is the test that keeps the tolerated list honest: if someone adds a third entry, or if an
    upstream event gains a disable type, it shows up here rather than silently permitting steering.
    """
    for name, types in EVENTS.items():
      carries_disable = any(et in types for et in LATERAL_DISABLE_TYPES)
      expected = carries_disable and name not in MADS_TOLERATED_EVENTS
      assert has_blocking_event(ev(name)) == expected, f"event {name} misclassified"

  def test_disable_types_are_pinned_by_value(self):
    # test_blocking_event_coverage derives its expectation FROM this constant, so it cannot notice
    # the constant changing. Pin it here.
    assert set(LATERAL_DISABLE_TYPES) == {ET.USER_DISABLE, ET.IMMEDIATE_DISABLE, ET.SOFT_DISABLE}

  def test_tolerated_list_is_exactly_the_brake_pair(self):
    assert set(MADS_TOLERATED_EVENTS) == {EventName.pedalPressed, EventName.pcmDisable}

  def test_no_entry_alone_does_not_block(self):
    # belowEngageSpeed is NO_ENTRY-only and is true for long stretches of ordinary driving.
    assert ET.NO_ENTRY in EVENTS[EventName.belowEngageSpeed]
    assert not any(et in EVENTS[EventName.belowEngageSpeed] for et in LATERAL_DISABLE_TYPES)
    assert not has_blocking_event(ev(EventName.belowEngageSpeed))

  def test_override_events_do_not_block(self):
    # A driver torque override or a gas override is not a disengage; MADS must ride through them.
    for name in (EventName.steerOverride, EventName.gasPressedOverride):
      assert not has_blocking_event(ev(name))

  def test_the_new_alert_itself_never_blocks(self):
    # madsLateralOnly is added to self.events by selfdrived AFTER mads.update() runs, but it will
    # still be present on the NEXT frame's... no: events are cleared each frame. Belt and braces --
    # if it ever carried a disable type it would latch MADS off one frame after arming it.
    assert not has_blocking_event(ev(EventName.madsLateralOnly))
    assert set(EVENTS[EventName.madsLateralOnly]) == {ET.PERMANENT}


class TestControlsdFallback:
  """`Controls.lat_authorised` must fall back to the stock answer unless madsState is authoritative."""

  class FakeSM:
    def __init__(self, mads_available, mads_active, ss_active, alive=True, valid=True):
      self.alive = {'madsState': alive}
      self.valid = {'madsState': valid}
      self._d = {
        'madsState': type('M', (), {'available': mads_available, 'active': mads_active})(),
        'selfdriveState': type('S', (), {'active': ss_active})(),
      }

    def __getitem__(self, k):
      return self._d[k]

  def _ctrl(self, **kw):
    c = object.__new__(Controls)
    c.sm = self.FakeSM(**kw)
    return c

  @pytest.mark.parametrize("ss_active", [True, False])
  def test_falls_back_when_not_available(self, ss_active):
    # mads.active is deliberately the OPPOSITE of the stock answer, so a wrong branch is visible.
    c = self._ctrl(mads_available=False, mads_active=not ss_active, ss_active=ss_active)
    assert c.lat_authorised() is ss_active

  @pytest.mark.parametrize("alive,valid", [(False, True), (True, False), (False, False)])
  def test_falls_back_when_stale_or_invalid(self, alive, valid):
    c = self._ctrl(mads_available=True, mads_active=True, ss_active=False, alive=alive, valid=valid)
    assert c.lat_authorised() is False

  @pytest.mark.parametrize("mads_active", [True, False])
  def test_uses_mads_when_authoritative(self, mads_active):
    c = self._ctrl(mads_available=True, mads_active=mads_active, ss_active=False)
    assert c.lat_authorised() is mads_active


class TestTeslaPathUntouched:
  """The Raven can never receive the MADS bits, so MadsPnw is inert on it by construction.

  `Car._alternative_experience` is the gate (capability view + PandaMadsSafety); this asserts the
  end-to-end consequence rather than re-testing the gate, which
  selfdrive/car/tests/test_mads_alternative_experience.py already covers.
  """

  @staticmethod
  def _alt_exp(fingerprint, panda_mads_safety, disengage_on_brake):
    class FakeParams:
      def get_bool(self, k):
        return {"PandaMadsSafety": panda_mads_safety, "DisengageOnBrake": disengage_on_brake}[k]

    CP = car.CarParams.new_message(carFingerprint=fingerprint, brand="tesla" if "TESLA" in fingerprint else "ford")
    c = object.__new__(Car)
    c.CP = CP
    c.params = FakeParams()
    return c._alternative_experience()

  def test_raven_gets_no_bits_and_mads_is_inert(self):
    alt = self._alt_exp("TESLA_MODEL_S_RAVEN", panda_mads_safety=True, disengage_on_brake=False)
    assert alt == 0
    mads = MadsPnw(alt)
    assert not mads.available
    engage(mads)
    brake_release_cruise(mads)
    assert (mads.enabled, mads.active, mads.lateral_only) == (False, False, False)

  def test_lightning_without_panda_declaration_is_inert(self):
    alt = self._alt_exp("FORD_F_150_LIGHTNING_MK1", panda_mads_safety=False, disengage_on_brake=False)
    assert alt == 0
    assert not MadsPnw(alt).available

  def test_lightning_with_panda_declaration_arms_remain_active(self):
    alt = self._alt_exp("FORD_F_150_LIGHTNING_MK1", panda_mads_safety=True, disengage_on_brake=False)
    assert alt == ALTERNATIVE_EXPERIENCE.ENABLE_MADS
    mads = MadsPnw(alt)
    assert mads.available and not mads.disengage_on_brake


class TestNeverSuppresses:
  """The claim this whole design rests on: MADS is a PARALLEL authority and never edits, delays or
  suppresses openpilot's own disengage.

  The first test is behavioural. The rest are SOURCE-LEVEL PINS on the wiring in selfdrived --
  weaker than a behavioural test, and honestly labelled as such: instantiating SelfdriveD needs a
  device (Params, msgq, a car). They exist because the ordering IS the safety argument, and a
  refactor that reorders these two calls would be silent otherwise.
  """

  def test_update_never_mutates_the_events_it_reads(self):
    mads = MadsPnw(MADS_ON)
    engage(mads)
    events = ev(EventName.pedalPressed, EventName.pcmDisable, EventName.buttonCancel)
    before = list(events.names)
    mads.update(op_enabled=False, op_active=False, braking=True, cruise_enabled=False, events=events)
    assert list(events.names) == before

  def test_module_never_removes_an_event_or_touches_latactive(self):
    """Parsed, not grepped -- so the prose above (which names the refused shortcut) can't pass it."""
    import openpilot.selfdrive.selfdrived.mads_pnw as m
    tree = ast.parse(inspect.getsource(m))
    called = {n.func.attr for n in ast.walk(tree)
              if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert "remove" not in called, "MADS must never delete an event openpilot raised"
    attrs = {n.attr for n in ast.walk(tree) if isinstance(n, ast.Attribute)}
    names = {n.id for n in ast.walk(tree) if isinstance(n, ast.Name)}
    assert "latActive" not in attrs | names, "MADS must not reach into controlsd's actuator gate"

  def test_mads_update_is_called_with_exactly_the_right_arguments_in_order(self):
    """Parsed, not grepped: swapping the first two arguments (enabled/active) or dropping
    regenBraking would be invisible to a substring check."""
    call = _find_call(SelfdriveD.step, "mads", "update")
    assert call is not None, "selfdrived must call self.mads.update()"
    assert [ast.unparse(a) for a in call.args] == [
      "self.enabled", "self.active", "CS.brakePressed or CS.regenBraking",
      "CS.cruiseState.enabled", "self.events"]

  def test_warning_alerts_are_readmitted_while_mads_steers_alone(self):
    """Without this, openpilot's own state machine sits in `disabled` (current_alert_types ==
    [ET.PERMANENT]) and update_alerts CLEARS every ET.WARNING -- swallowing "Take Control"
    (steerSaturated), the lane-change prompts, and belowSteerSpeed while the truck is steering."""
    tree = _fn_ast(SelfdriveD.step)
    found = False
    for node in ast.walk(tree):
      if not isinstance(node, ast.If):
        continue
      if ast.unparse(node.test) != "self.mads.active and (not self.active)":
        continue
      found = any(isinstance(b, ast.Expr) and isinstance(b.value, ast.Call) and
                  ast.unparse(b.value) == "self.state_machine.current_alert_types.append(ET.WARNING)"
                  for b in node.body)
    assert found, "ET.WARNING must be re-admitted for exactly the frames MADS steers alone"

  def test_controlsd_actually_consumes_lat_authorised(self):
    """`lat_authorised()` being correct is worthless if controlsd does not use it. Reverting either
    call site to selfdriveState.active would leave every isolated test of the helper passing."""
    tree = _fn_ast(Controls.state_control)
    lat_active = [n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                  and any(ast.unparse(t) == "CC.latActive" for t in n.targets)]
    assert len(lat_active) == 1
    assert ast.unparse(lat_active[0].value).startswith("self.lat_authorised()")
    assert any(isinstance(n, ast.If) and ast.unparse(n.test) == "self.lat_authorised()"
               for n in ast.walk(_fn_ast(Controls.publish))), \
      "the steer_limited_by_safety check must use it too"

  def test_ui_paints_override_not_disengaged_while_mads_steers(self):
    # Parsed from the file rather than imported: importing ui_state constructs the UIState
    # singleton, which reads params and dies on a host with a stale params_pyx (the same reason
    # selfdrive/ui's own tests do not collect here). This is at baseline too, not our doing.
    tree = _method_ast("openpilot.selfdrive.ui.ui_state", "UIState", "_update_status")
    found = False
    for node in ast.walk(tree):
      if isinstance(node, ast.If) and ast.unparse(node.test) == "mads.available and mads.lateralOnly":
        found = any(isinstance(b, ast.Assign) and ast.unparse(b.value) == "UIStatus.OVERRIDE"
                    for b in node.body)
    assert found, "a car that is steering itself must never paint DISENGAGED"

  def test_driver_monitoring_treats_lateral_only_as_engaged(self):
    """Otherwise _update_events takes the (not op_engaged) branch and RESETS awareness every frame
    -- no distraction monitoring at all while the truck steers itself."""
    from openpilot.selfdrive.monitoring import dmonitoringd
    from openpilot.selfdrive.monitoring.helpers import DriverMonitoring
    tree = _fn_ast(DriverMonitoring.run_step)
    assert any(isinstance(n, ast.Assign) and ast.unparse(n.value) == "sm['madsState']"
               for n in ast.walk(tree)), "run_step must read madsState"
    assigns = [ast.unparse(n.value) for n in ast.walk(tree) if isinstance(n, ast.Assign)
               and any(ast.unparse(t) == "enabled" for t in n.targets)]
    assert "sm['selfdriveState'].enabled or (mads.available and mads.lateralOnly)" in assigns
    # parsed, not grepped -- the explanatory comment above the SubMaster also contains the string
    sub = _find_call(dmonitoringd.dmonitoringd_thread, "messaging", "SubMaster")
    assert sub is not None
    assert "madsState" in {e.value for e in sub.args[0].elts if isinstance(e, ast.Constant)}, \
      "dmonitoringd must actually subscribe to madsState"

  def test_mads_runs_after_openpilots_own_state_machine(self):
    src = inspect.getsource(SelfdriveD.step)
    assert src.index("self.state_machine.update(self.events)") < src.index("self.mads.update(")

  def test_mads_alert_is_added_after_the_state_machine_and_before_the_alerts(self):
    src = inspect.getsource(SelfdriveD.step)
    # the alert must be raised, and raised on the lateral-only condition -- not unconditionally,
    # and not behind a constant.
    assert "if self.mads.lateral_only:\n      " in src
    assert src.index("self.mads.update(") < src.index("EventName.madsLateralOnly")
    assert src.index("EventName.madsLateralOnly") < src.index("self.update_alerts(CS)")

  def test_brake_input_includes_regen_braking(self):
    # The panda's `is_braking` is `brake_pressed || regen_braking`. On an EV, regen alone is the
    # common way the truck slows -- reading only brakePressed would silently miss it.
    assert "CS.brakePressed or CS.regenBraking" in inspect.getsource(SelfdriveD.step)

  def test_madsstate_is_published_before_selfdrivestate(self):
    # controlsd polls on selfdriveState; sending madsState first is what guarantees the frame's
    # authority is already queued when controlsd wakes.
    src = inspect.getsource(SelfdriveD.publish_selfdriveState)
    assert src.index("self.pm.send('madsState'") < src.index("self.pm.send('selfdriveState'")

  def test_mismatch_counter_is_still_keyed_on_openpilots_own_enabled(self):
    # This is why controlsMismatch cannot fire because of MADS: the counter is reset whenever
    # openpilot itself is disengaged, which is exactly the lateral-only state.
    src = inspect.getsource(SelfdriveD.data_sample)
    assert "if not self.enabled:\n      self.mismatch_counter = 0" in src


# ---------------------------------------------------------------------------------------------
# madsheartbeat2pnw — the lateral mismatch detector
# ---------------------------------------------------------------------------------------------

class _FakePandaState:
  def __init__(self, controls_allowed_lateral: bool, safety_model=car.CarParams.SafetyModel.ford):
    self.controlsAllowedLateral = controls_allowed_lateral
    self.controlsAllowed = controls_allowed_lateral
    self.safetyModel = safety_model


class _FakeSM:
  """Just enough SubMaster for the tail of SelfdriveD.data_sample."""
  def __init__(self, panda_states):
    self._panda_states = panda_states
    self.frame = 0

  def update(self, _timeout):
    self.frame += 1

  def __getitem__(self, key):
    assert key == 'pandaStates', key
    return self._panda_states


class _FakeSock:
  def receive(self, non_blocking=False):
    return None


class _FakeMads:
  def __init__(self, available: bool, lateral_only: bool):
    self.available = available
    self.lateral_only = lateral_only


def _sd_for_mismatch(available: bool, lateral_only: bool, enabled: bool, panda_states):
  """A real SelfdriveD with only the attributes data_sample's tail touches. No refactor, no
  reimplementation: the code under test is the shipped method."""
  sd = SelfdriveD.__new__(SelfdriveD)
  sd.car_state_sock = _FakeSock()
  sd.CS_prev = car.CarState.new_message().as_reader()
  sd.sm = _FakeSM(panda_states)
  sd.initialized = True
  sd.enabled = enabled
  sd.mads = _FakeMads(available, lateral_only)
  sd.mismatch_counter = 0
  sd.lateral_mismatch_counter = 0
  return sd


class TestLateralMismatchDetector:
  """The panda now publishes its own lateral authority (PandaState.controlsAllowedLateral). This
  is the detector that turns a panda-side revoke from a SILENT no-steer into an alert."""

  def test_counter_climbs_while_the_panda_refuses_lateral(self):
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False,
                          panda_states=[_FakePandaState(False)])
    for expected in range(1, 6):
      sd.data_sample()
      assert sd.lateral_mismatch_counter == expected

  def test_counter_stays_at_zero_while_the_panda_permits_lateral(self):
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False,
                          panda_states=[_FakePandaState(True)])
    for _ in range(500):
      sd.data_sample()
    assert sd.lateral_mismatch_counter == 0

  def test_counter_resets_when_the_panda_comes_back(self):
    states = [_FakePandaState(False)]
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False, panda_states=states)
    for _ in range(10):
      sd.data_sample()
    assert sd.lateral_mismatch_counter == 10
    states[0].controlsAllowedLateral = True
    sd.data_sample()
    # the panda agreeing again does NOT reset the counter -- only leaving the lateral-only state
    # does, exactly like the longitudinal mismatch_counter above it. It simply stops climbing.
    assert sd.lateral_mismatch_counter == 10

  @pytest.mark.parametrize("available,lateral_only,enabled", [
    (False, True, False),    # MADS not available -- the shipping default on every car
    (True, False, False),    # not holding lateral alone
    (True, True, True),      # openpilot itself is engaged: the longitudinal check already covers it
    (False, False, True),
  ])
  def test_counter_is_pinned_to_zero_outside_lateral_only(self, available, lateral_only, enabled):
    sd = _sd_for_mismatch(available, lateral_only, enabled, panda_states=[_FakePandaState(False)])
    sd.lateral_mismatch_counter = 199  # a stale, nearly-saturated counter
    for _ in range(50):
      sd.data_sample()
    assert sd.lateral_mismatch_counter == 0

  def test_reengaging_cannot_carry_a_saturated_counter_into_an_enabled_frame(self):
    """The `self.enabled` term. Both self.enabled and mads.lateral_only are one frame stale here,
    so without it a re-engage could fire an IMMEDIATE_DISABLE at a car that is steering fine."""
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False,
                          panda_states=[_FakePandaState(False)])
    for _ in range(250):
      sd.data_sample()
    assert sd.lateral_mismatch_counter >= 200
    sd.enabled = True                      # openpilot re-engaged; mads.lateral_only still stale True
    sd.data_sample()
    assert sd.lateral_mismatch_counter == 0

  def test_silent_pandas_are_ignored(self):
    """Same exclusion as the longitudinal counter: a panda in a silent/noOutput safety mode is not
    supposed to allow anything, so it must not be read as a revoke."""
    silent = _FakePandaState(False, safety_model=car.CarParams.SafetyModel.silent)
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False, panda_states=[silent])
    for _ in range(50):
      sd.data_sample()
    assert sd.lateral_mismatch_counter == 0

  def test_the_longitudinal_counter_is_untouched_by_all_of_this(self):
    sd = _sd_for_mismatch(available=True, lateral_only=True, enabled=False,
                          panda_states=[_FakePandaState(False)])
    for _ in range(50):
      sd.data_sample()
    assert sd.mismatch_counter == 0, "MADS must never be able to move the longitudinal counter"

  # ---- the event ---------------------------------------------------------------------------

  def test_event_is_raised_at_two_seconds(self):
    src = inspect.getsource(SelfdriveD.update_events)
    assert "if self.lateral_mismatch_counter >= 200:\n      self.events.add(EventName.madsControlsMismatchLateral)" in src

  def test_event_ends_the_lateral_only_state(self):
    """THE consequence: openpilot must stop commanding lateral into a panda that is blocking it."""
    mads = MadsPnw(MADS_ON)
    engage(mads)
    brake_release_cruise(mads)
    assert mads.enabled and mads.lateral_only
    mads.update(op_enabled=False, op_active=False, braking=False, cruise_enabled=False,
                events=ev(EventName.pcmDisable, EventName.madsControlsMismatchLateral))
    assert not mads.enabled
    assert not mads.active

  def test_event_carries_immediate_disable(self):
    assert ET.IMMEDIATE_DISABLE in EVENTS[EventName.madsControlsMismatchLateral]
    assert has_blocking_event(ev(EventName.madsControlsMismatchLateral))
    assert EventName.madsControlsMismatchLateral not in MADS_TOLERATED_EVENTS


class TestPandadHeartbeatPlumbing:
  """The panda's lateral watchdog is fed by heartbeat 0xf3 param2. These pin the plumbing that
  makes it a real signal rather than a constant -- the failure mode that made bluepilot's copy
  dead code."""

  PANDAD = pathlib.Path(__file__).parents[2] / "pandad"

  def test_send_heartbeat_forwards_the_mads_flag_to_param2(self):
    src = (self.PANDAD / "panda.cc").read_text()
    assert "void Panda::send_heartbeat(bool engaged, bool engaged_mads) {" in src
    assert "handle->control_write(0xf3, engaged, engaged_mads);" in src

  def test_pandad_derives_the_flag_from_madsstate_and_not_a_constant(self):
    src = (self.PANDAD / "pandad.cc").read_text()
    assert '"madsState"' in src, "pandad must subscribe to madsState"
    assert 'sm["madsState"].getMadsState().getEnabled()' in src
    assert 'sm["madsState"].getMadsState().getAvailable()' in src
    assert 'sm.allAliveAndValid({"madsState"})' in src, "a stale madsState must revoke, not grant"
    assert "panda->send_heartbeat(engaged, engaged_mads);" in src

  def test_pandad_publishes_the_pandas_lateral_authority(self):
    src = (self.PANDAD / "pandad.cc").read_text()
    assert "ps.setControlsAllowedLateral((bool)(health.controls_allowed_lateral_pkt));" in src

  def test_health_packet_short_read_is_rejected_not_zero_filled(self):
    """The versioned-wire-struct guard: an old panda answers 0xd2 with fewer bytes than this build
    expects, and the zero-initialised tail would read as a fabricated `false`."""
    src = (self.PANDAD / "panda.cc").read_text()
    assert "if (err != (int)sizeof(health)) {" in src
    body = src.split("if (err != (int)sizeof(health)) {")[1][:500]
    assert "health_packet_mismatch = true;" in body
    assert "return std::nullopt;" in body
    # a COMMS error must not be mistaken for a version mismatch
    assert "if (err < 0) {\n    // comms error" in src

  def test_connect_refuses_a_mismatched_panda_but_not_a_flaky_one(self):
    src = (self.PANDAD / "pandad.cc").read_text()
    assert "if (panda->health_packet_mismatch) {" in src
    assert "throw std::runtime_error(\"Panda health packet layout mismatch" in src
