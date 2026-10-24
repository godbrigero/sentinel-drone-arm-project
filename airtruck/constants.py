from pathlib import Path

from airtruck.util.environment import RunMode

RUN_MODE: RunMode = RunMode.REAL

LEADER_PORT: str = "/dev/tty.usbmodem5B7A0002911"
FOLLOWER_PORT: str = "/dev/tty.usbmodem5B7A0002901"
LEADER_BAUDRATE: int = 1_000_000
FOLLOWER_BAUDRATE: int = 1_000_000
LEADER_CALIBRATION: Path = (
    Path(__file__).resolve().parents[1] / "models/so101/runtime/leader-calibration.json"
)
FOLLOWER_CALIBRATION: Path = (
    Path(__file__).resolve().parents[1] / "models/so101/runtime/calibration.json"
)
