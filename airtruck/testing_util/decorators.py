"""Pytest opt-in; importing world utilities does not require pytest."""
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import pytest


def simulation(world_dir: str | None = None) -> "pytest.MarkDecorator":
    """Select a world directory; loading occurs only at fixture setup."""
    import pytest
    return pytest.mark.sim_world(world_dir)
