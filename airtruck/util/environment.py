from enum import Enum


class RunMode(Enum):
    """Select the follower backend; the application always reads a physical leader."""

    SIMULATION = "simulation"
    """Drive a simulated follower from the physical leader."""
    REAL = "real"
    """Drive a physical follower from the physical leader."""
