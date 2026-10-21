"""World behavior, including independent control and state ownership."""
from collections.abc import Callable
import json
import math
from pathlib import Path
from typing import cast
import numpy as np
import pytest
from airtruck.testing_util import Pose, WorldControl, load_world
from airtruck.testing_util.mujoco_world import MujocoWorld
from airtruck.raw_controls.so101.io import ArmController
from airtruck.testing_util import simulation

ROOT = Path(__file__).resolve().parents[3]


def make_world(directory: Path) -> Path:
    model = ROOT / 'models/so101'
    (directory / 'scene.xml').write_text('''<mujoco>
      <option timestep="0.001" integrator="implicitfast"/>
      <worldbody>
        <body name="block" pos="0 0 0.5"><freejoint/>
          <geom type="box" size="0.02 0.02 0.02" mass="0.1"/>
        </body>
        <body name="target" mocap="true"><geom type="sphere" size="0.01" contype="0" conaffinity="0"/></body>
      </worldbody></mujoco>''')
    systems = [dict(name=name, model=str(model/'so101.xml'), root_body='base',
                    controller='so101', controller_config=str(model/'runtime/servo-model.json'),
                    pose={'position_m': [x, 0, 0]}, placement='fixed')
               for name, x in [('left', -0.4), ('right', 0.4)]]
    (directory/'world.json').write_text(json.dumps(dict(
        schema_version=1, scene='scene.xml', systems=systems,
        entities=[dict(name='block', body='block', placement='free'),
                  dict(name='target', body='target', placement='kinematic')])))
    return directory


@pytest.mark.sim
def test_two_arms_one_clock_and_independent_actuators(tmp_path: Path) -> None:
    with load_world(make_world(tmp_path)) as world:
        left = world.system('left', ArmController)
        right = world.system('right', ArmController)
        # The free joint before both arms makes qpos/dof/joint indices differ.
        world.advance(3)
        baseline = right.positions
        left.set_positions({'shoulder_pan': 0.1})
        assert right.targets['shoulder_pan'] == 0
        before = world.time_s
        world.step()
        assert world.time_s - before == pytest.approx(.001)
        world.advance(3)
        assert left.positions['shoulder_pan'] == pytest.approx(.1, abs=.01)
        assert right.positions['shoulder_pan'] == pytest.approx(baseline['shoulder_pan'], abs=.01)
        left.set_enabled(False)
        right.set_positions({'shoulder_pan': -.1})
        world.advance(3)
        assert right.positions['shoulder_pan'] == pytest.approx(-.1, abs=.01)
        li = world._model.actuator('left/shoulder_pan').id
        ri = world._model.actuator('right/shoulder_pan').id
        assert world._data.actuator_force[li] == 0
        assert world._model.actuator_gainprm[ri, 0] > 0
        world.reset()
        assert world.time_s == 0
        assert left.enabled and right.enabled
        assert all(v == 0 for v in (*left.positions.values(), *right.positions.values()))
        left.set_positions({'shoulder_pan': .1})
        world.advance(3)
        assert left.positions['shoulder_pan'] == pytest.approx(.1, abs=.01)


@pytest.mark.sim
def test_placement_snapshots_and_reset(tmp_path: Path) -> None:
    with load_world(make_world(tmp_path)) as world:
        block, target = world.entity('block'), world.entity('target')
        initial = world.pose(block)
        world.advance(.1)
        before = world.system('left', ArmController).positions
        pose = Pose((.2, .1, .3), (math.sqrt(.5), 0, 0, math.sqrt(.5)))
        world.place(block, pose)
        assert world.pose(block) == pose
        assert world.system('left', ArmController).positions == before
        np.testing.assert_array_equal(world._data.qvel[:6], 0)
        world.place(target, pose)
        assert world.pose(target) == pose
        with pytest.raises(ValueError, match='Fixed'):
            world.place(world.entity('left'), pose)
        snapshots = world.record(lambda: world.pose(block), .01, .005)
        assert len(snapshots) == 3 and snapshots[0].value == pose
        world.reset()
        assert world.pose(block) == initial
        assert world.pose(target) == Pose()
        # Free object falls under gravity, proving pose is refreshed after stepping.
        world.advance(.02)
        assert world.pose(block).position_m[2] < initial.position_m[2]
        world._data.qvel[:6] = 1
        world.place(block, pose, clear_velocity=False)
        np.testing.assert_array_equal(world._data.qvel[:6], 1)


@simulation()
def test_public_views_and_handle_lifetime(world: WorldControl, world_factory: Callable[[], WorldControl]) -> None:
    arm = world.system('arm', ArmController)
    assert not hasattr(arm, 'data') and not hasattr(arm, 'model') and not hasattr(arm, 'step')
    with pytest.raises(KeyError, match='Unknown system'):
        world.system('missing', ArmController)
    with pytest.raises(TypeError, match='does not implement'):
        world.system('arm', str)
    other = world_factory()
    with pytest.raises(ValueError, match='different world'):
        other.pose(world.entity('arm'))
    handle = world.entity('arm')
    world.reset()
    assert world.pose(handle) == Pose()
    values = {'x': 1.0}
    samples = world.record(lambda: values, .001)
    values['x'] = 2
    assert samples[0].value == {'x': 1.0}
    world.close()
    for operation in (lambda: arm.positions, lambda: arm.set_enabled(True),
                      lambda: world.pose(handle), lambda: world.reset()):
        with pytest.raises(RuntimeError, match='closed'):
            operation()


def test_pose_rejects_invalid_values() -> None:
    for p, q in [((float('nan'), 0, 0), (1, 0, 0, 0)), ((0, 0, 0), (0, 0, 0, 0))]:
        with pytest.raises(ValueError):
            Pose(p, q)


@pytest.mark.sim
@pytest.mark.parametrize('problem', ['duplicate', 'controller', 'placement', 'field', 'config', 'body'])
def test_bad_world_configuration(tmp_path: Path, problem: str) -> None:
    make_world(tmp_path)
    path = tmp_path/'world.json'
    data = json.loads(path.read_text())
    if problem == 'duplicate':
        data['systems'][1]['name'] = 'left'
    elif problem == 'controller':
        data['systems'][0]['controller'] = 'unknown'
    elif problem == 'placement':
        data['systems'][0]['placement'] = 'free'
    elif problem == 'field':
        data['systems'][0]['typo'] = 1
    elif problem == 'body':
        data['systems'][0]['root_body'] = 'absent'
    else:
        config = tmp_path/'invalid.json'
        config.write_text('{"datasheet": {"encoder_counts": 4096, "voltage_V": -1}}')
        data['systems'][0]['controller_config'] = str(config)
    path.write_text(json.dumps(data))
    with pytest.raises(ValueError):
        load_world(tmp_path)


@simulation()
def test_cleanup_attempts_every_adapter(world: WorldControl, monkeypatch: pytest.MonkeyPatch) -> None:
    backend = cast(MujocoWorld, world)
    original = backend._systems['arm']
    called: list[str] = []

    def fail() -> None:
        called.append('first')
        raise RuntimeError('adapter failure')

    class Last:
        def disable(self) -> None:
            called.append('last')

    monkeypatch.setattr(original, 'disable', fail)
    monkeypatch.setitem(backend._systems, 'last', Last())
    with pytest.raises(ExceptionGroup):
        world.close()
    assert called == ['first', 'last']
    world.close()


@pytest.mark.sim
def test_mixed_system_adapter_registry(tmp_path: Path) -> None:
    """A non-arm system gets a different typed interface in the same world."""
    from collections.abc import Mapping
    import mujoco
    from airtruck.testing_util.adapters import SystemAdapter

    class SlideControl:
        def __init__(self, command: Callable[[float], None]) -> None:
            self._command = command

        def set_force(self, force_n: float) -> None:
            self._command(force_n)

    class SlideAdapter:
        def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, prefix: str) -> None:
            self._data = data
            self._index = model.actuator(prefix + 'drive').id
            self._force = 0.0

        @property
        def actuator_ids(self) -> tuple[int, ...]:
            return (self._index,)

        def interface(self, require_open: Callable[[], None]) -> SlideControl:
            def command(force: float) -> None:
                require_open()
                if not math.isfinite(force):
                    raise ValueError('Force must be finite')
                self._force = force
            return SlideControl(command)

        def update_control(self) -> None:
            self._data.ctrl[self._index] = self._force

        def reset_control(self) -> None:
            self._force = 0

        def disable(self) -> None:
            self._force = 0
            self._data.ctrl[self._index] = 0

    def factory(model: mujoco.MjModel, data: mujoco.MjData, prefix: str,
                config: Mapping[str, object]) -> SystemAdapter:
        return SlideAdapter(model, data, prefix)

    make_world(tmp_path)
    (tmp_path/'slide.xml').write_text('''<mujoco><worldbody><body name="carriage">
      <joint name="slide" type="slide" axis="1 0 0"/>
      <geom type="sphere" size="0.02" mass="1" contype="0" conaffinity="0"/>
      </body></worldbody><actuator><motor name="drive" joint="slide"/></actuator></mujoco>''')
    (tmp_path/'slide.json').write_text('{}')
    path = tmp_path/'world.json'
    data = json.loads(path.read_text())
    data['systems'].append(dict(name='stage', model='slide.xml', root_body='carriage',
                                controller='slide', controller_config='slide.json'))
    path.write_text(json.dumps(data))
    with load_world(tmp_path, adapters={'slide': factory}) as world:
        arm = world.system('left', ArmController)
        slide = world.system('stage', SlideControl)
        slide.set_force(1)
        arm.set_positions({'shoulder_pan': .1})
        world.advance(.1)
        assert world.pose(world.entity('stage')).position_m[0] > 0
        assert arm.targets['shoulder_pan'] == pytest.approx(.1, abs=.001)
        with pytest.raises(TypeError):
            world.system('stage', ArmController)
        world.reset()
        assert world.pose(world.entity('stage')).position_m[0] == 0
        world.advance(.1)
        assert world.pose(world.entity('stage')).position_m[0] == 0


@pytest.mark.sim
def test_composition_preserves_so101_dynamics() -> None:
    import mujoco
    from airtruck.testing_util.adapters import SO101Adapter
    from airtruck.testing_util.loader import servo_spec
    from airtruck.raw_controls.so101.controller import SimSO101
    root = ROOT/'models/so101'
    model = mujoco.MjModel.from_xml_path(str(root/'so101.xml'))
    data = mujoco.MjData(model)
    controller = SimSO101(model, data, servo_spec=servo_spec(json.loads((root/'runtime/servo-model.json').read_text())))
    with MujocoWorld(model, data, {'arm': SO101Adapter(controller)}, {}) as original, load_world(ROOT/'worlds/so101_bench') as composed:
        for w in (original, composed):
            w.system('arm', ArmController).set_positions([.1, .1, .1, .1, .1, .1])
            w.advance(1)
        np.testing.assert_allclose(data.qpos, composed._data.qpos, rtol=0, atol=1e-12)
        np.testing.assert_allclose(model.body_mass, composed._model.body_mass, rtol=0, atol=1e-12)


@pytest.mark.sim
def test_duplicate_actuator_ownership_rejected() -> None:
    with load_world(ROOT/'worlds/so101_bench') as world:
        adapter = world._systems['arm']
        with pytest.raises(ValueError, match='share actuator'):
            MujocoWorld(world._model, world._data, {'a': adapter, 'b': adapter}, {})


@pytest.mark.sim
def test_loader_cleans_up_after_partial_binding_failure(tmp_path: Path) -> None:
    from collections.abc import Mapping
    import mujoco
    from airtruck.testing_util.loader import _so101
    from airtruck.testing_util.adapters import SystemAdapter
    make_world(tmp_path)
    called: list[str] = []

    def factory(model: mujoco.MjModel, data: mujoco.MjData, prefix: str,
                config: Mapping[str, object]) -> SystemAdapter:
        if prefix == 'right/':
            raise ValueError('failed second system')
        adapter = _so101(model, data, prefix, config)
        class Tracked:
            @property
            def actuator_ids(self) -> tuple[int, ...]:
                return adapter.actuator_ids
            def interface(self, require_open: Callable[[], None]) -> object:
                return adapter.interface(require_open)
            def update_control(self) -> None:
                adapter.update_control()
            def reset_control(self) -> None:
                adapter.reset_control()
            def disable(self) -> None:
                called.append(prefix)
                adapter.disable()
        return Tracked()

    with pytest.raises(ValueError, match='failed second system'):
        load_world(tmp_path, adapters={'so101': factory})
    assert called == ['left/']
