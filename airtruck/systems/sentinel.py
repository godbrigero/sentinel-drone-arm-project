"""The Sentinel quadrotor: one object for simulation or the real PX4 aircraft."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from airtruck.systems.base import ROOT
from airtruck.systems.drone import Drone

MODEL = ROOT / "models/sentinel-quad/sentinel.xml"
# Body-origin height that puts the landing feet on a floor at z = 0.
SPAWN_HEIGHT_M = float(
    json.loads((ROOT / "models/sentinel-quad/information.json").read_text())[
        "simulation"
    ]["spawn_height_m"]
)


class SentinelDrone(Drone):
    """Sentinel with its built-in model, cameras and PX4 ROS 2 backend.

    `hardware=False` (default) flies the MuJoCo model; `hardware=True` flies the
    aircraft through PX4 with the same controls, state, flight commands, cameras
    and tag detections. Switch `drone.hardware` between driver steps, as with arms.

    Nothing connects at construction. The PX4 backend opens when the drone's
    `SystemDriver` enters its `with` block (or at its first reset/step), so no
    connect call is required; connecting never arms or changes flight mode.
    """

    def __init__(
        self,
        name: str,
        *,
        hardware: bool = False,
        model: str | Path = MODEL,
        position: tuple[float, float, float] = (0, 0, SPAWN_HEIGHT_M),
        **kwargs: Any,
    ) -> None:
        super().__init__(
            name, model=model, hardware=hardware, position=position, **kwargs
        )
