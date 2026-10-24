"""Pytest opt-in marker; world loading occurs only at fixture setup."""
import pytest


def simulation(world_dir: str | None = None) -> "pytest.MarkDecorator":
    """Select a world directory; loading occurs only at fixture setup."""
    return pytest.mark.sim_world(world_dir)
