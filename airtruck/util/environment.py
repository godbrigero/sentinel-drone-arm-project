from enum import Enum


class RunMode(Enum):
    """The mode in which the application is running."""

    SIMULATION = "simulation"
    """Run the application in simulation mode."""
    REAL = "real"
    """Run the application in real mode."""
