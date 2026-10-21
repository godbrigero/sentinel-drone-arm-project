# Airtruck

`airtruck` is a namespace package with no `__init__.py` files. Import controls directly from `airtruck.raw_controls.so101.controller` and interfaces from `airtruck.raw_controls.so101.io`:

```python
from airtruck.raw_controls.so101.io import ArmController  # Shared typed control interface
from airtruck.raw_controls.so101.controller import SimSO101  # Explicit simulation backend
from airtruck.raw_controls.so101.controller import RealSO101 # Explicit physical backend
```

All paths below are relative to the repository root.

| Folder | Responsibility |
|---|---|
| `airtruck/raw_controls/so101/controller.py` | `SimSO101`, `RealSO101`, and hardware joint calibration |
| `airtruck/raw_controls/so101/io.py` | `ArmController` abstract base class and `ServoTransport` protocol |
| `models/so101/` | MJCF, mesh assets, exporter configuration and motor parameters |
| `scripts/so101/` | Model rebuild, verification and viewer commands |
| `airtruck/raw_controls/so101/tests/` | Simulation and fake-transport regression tests |
| `docs/` | Usage, hardware and motor-model documentation |
| `typings/` | MuJoCo stubs for static tooling only |
| `global-info-images/` | Existing user-provided reference images |

Device controls live under `airtruck/raw_controls/<device>/`; SO-101 simulation and hardware implementations share `controller.py`, with their interfaces alongside in `io.py`. Future drone controls can live under `airtruck/raw_controls/drone/`.

The controller receives its dependencies explicitly: `SimSO101(model, data, servo_spec=spec)`. Scripts and test fixtures load the model and configuration; the controller has no filesystem or environment knowledge.

Both backends inherit `ArmController` from `airtruck.raw_controls.so101.io`. Its abstract methods and properties must be implemented before a subclass can be instantiated. Both backends expose `set_positions`, `positions`, `targets`, `limits`, `enabled` and `set_enabled`. Simulation advances only when `step()` is called. Physical motors run asynchronously through the supplied driver; the physical adapter deliberately has no simulated `step()` or automatic reset-to-zero.

```python
from airtruck.raw_controls.so101.io import ArmController

def move_pan(arm: ArmController) -> None:
    arm.set_positions({"shoulder_pan": 0.0872665})
```

Run `uv sync --locked` to create `.venv/` with the locked dependencies. The [SO-101 instructions](so101/README.md) describe model commands and the [physical adapter instructions](hardware/README.md) explain the hardware boundary.

## Static typing

The root `pyrightconfig.json` selects `typings/` for Pyright/Pylance. Select `.venv/bin/python` in your editor. The stubs describe MuJoCo 3.16.0 and are not runtime modules.

```sh
uv run mypy
```

`mypy` and `pytest` are in the default development dependency group.

## Run and verify

Run these commands from the repository root:

```sh
./scripts/so101/run.command --demo
uv run pytest
uv run python scripts/so101/verify.py
```

Verification prints its report to the terminal. Model inputs remain under `models/so101/config/`; changing the controller code does not require rebuilding the XML. See the [SO-101 guide](so101/README.md) for rebuilding and recording commands.

## Dependencies

`pyproject.toml` declares dependencies; `uv.lock` pins their resolved versions.
`uv sync --locked` creates `.venv/` and installs runtime and development dependencies.
`uv run` uses this environment automatically; activation is optional:

```sh
source .venv/bin/activate
```

The global `requirements.txt` is a generated compatibility export. After dependency changes, refresh it with:

```sh
uv export --locked --no-hashes --output-file requirements.txt
```

Commit `pyproject.toml`, `uv.lock`, and the exported requirements; keep `.venv/` ignored.
Pytest explicitly runs the two control behavior files listed in `pyproject.toml`.

## What the supporting files do

`models/so101/config/servos.json` is runtime controller configuration.
`cad-export.json` is required to rebuild the model and verify native CAD geometry;
`electrical-fit.json` and `config.json` support the build/verification workflow.
`requirements.txt` is a compatibility export; uv itself uses `pyproject.toml` and
`uv.lock`. Pytest and mypy configuration live in `pyproject.toml`.
