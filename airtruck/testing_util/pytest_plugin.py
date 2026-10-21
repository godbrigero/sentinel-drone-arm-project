"""Explicit opt-in pytest worlds. Ordinary tests create no simulation."""
from collections.abc import Callable, Iterator
from pathlib import Path
import pytest
from airtruck.testing_util import WorldControl, load_world
from airtruck.testing_util.mujoco_world import MujocoWorld


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("airtruck simulation")
    group.addoption("--sim-gui", action="store_true", help="Show MuJoCo windows (macOS: use mjpython).")
    group.addoption("--sim-world-dir", default="worlds/so101_bench", help="Default world directory, relative to pytest root.")


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line("markers", "sim: explicitly selected simulation test")
    config.addinivalue_line("markers", "sim_world(directory): select a world directory")


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    for item in items:
        if item.get_closest_marker("sim_world") is not None:
            item.add_marker(pytest.mark.sim)


@pytest.fixture
def world_factory(request: pytest.FixtureRequest) -> Iterator[Callable[[], WorldControl]]:
    marker = request.node.get_closest_marker("sim_world")
    if marker is None:
        pytest.fail('World fixtures require @simulation("worlds/so101_bench")', pytrace=False)
    if len(marker.args) > 1 or marker.kwargs:
        pytest.fail("@simulation accepts one optional directory", pytrace=False)
    selected = marker.args[0] if marker.args else None
    if selected is None:
        selected = request.config.getoption("--sim-world-dir")
    if not isinstance(selected, str) or not selected:
        pytest.fail("World directory must be a nonempty string", pytrace=False)
    directory = Path(selected)
    if not directory.is_absolute():
        directory = request.config.rootpath / directory
    sessions: list[MujocoWorld] = []

    def create() -> WorldControl:
        session = load_world(directory)
        sessions.append(session)
        if request.config.getoption("--sim-gui"):
            session.open_window()
        return session

    try:
        yield create
    finally:
        errors: list[Exception] = []
        for session in reversed(sessions):
            try:
                session.close()
            except Exception as error:
                errors.append(error)
        if errors:
            raise ExceptionGroup("World cleanup failed", errors)


@pytest.fixture
def world(world_factory: Callable[[], WorldControl]) -> WorldControl:
    return world_factory()
