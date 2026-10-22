"""Fixture cleanup when viewer creation fails after a world was loaded."""
from pathlib import Path
from collections.abc import Callable, Generator
from types import SimpleNamespace
from typing import cast
import pytest
from airtruck.testing_util.pytest_plugin import world_factory
from airtruck.testing_util.mujoco_world import MujocoWorld
from airtruck.testing_util import WorldControl, load_world
from airtruck.raw_api.so101.io import ArmController


@pytest.mark.sim
def test_failed_viewer_startup_closes_world(monkeypatch: pytest.MonkeyPatch) -> None:
    root = Path(__file__).resolve().parents[3]
    session = load_world(root/'worlds/so101_bench')
    arm = session.system('arm', ArmController)

    def fail_open(self: MujocoWorld) -> None:
        raise RuntimeError('viewer startup failed')

    monkeypatch.setattr(MujocoWorld, 'open_window', fail_open)
    monkeypatch.setattr('airtruck.testing_util.pytest_plugin.load_world', lambda path: session)
    request = cast(pytest.FixtureRequest, SimpleNamespace(
        node=SimpleNamespace(get_closest_marker=lambda name: pytest.mark.sim_world('worlds/so101_bench').mark),
        config=SimpleNamespace(rootpath=root, getoption=lambda name: True),
    ))
    fixture_body = cast(Callable[[pytest.FixtureRequest], Generator[Callable[[], WorldControl], None, None]],
                        getattr(world_factory, "__wrapped__"))
    lifecycle = fixture_body(request)
    create = next(lifecycle)
    try:
        with pytest.raises(RuntimeError, match='viewer startup failed'):
            create()
    finally:
        lifecycle.close()
    with pytest.raises(RuntimeError, match='closed'):
        arm.set_enabled(True)
