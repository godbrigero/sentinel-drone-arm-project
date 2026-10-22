from pathlib import Path

from airtruck.util.environment import RunMode

RUN_MODE: RunMode = RunMode.SIMULATION
PORT: str = ""  # Set your USB servo adapter path before using real mode.
CALIBRATION: Path = Path(__file__).resolve().parents[1] / "models/so101/runtime/calibration.json"
