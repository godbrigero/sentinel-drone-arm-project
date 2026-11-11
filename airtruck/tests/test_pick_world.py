"""The yellow-cube world and its scan inputs are self-contained in the repository."""

import hashlib
import json

import mujoco

from airtruck.examples.so101_pick import MODEL, SOURCE, WORLD, make_driver
from airtruck.systems.base import ROOT


def test_scan_sources_match_record() -> None:
    record = json.loads((MODEL / "information.json").read_text())
    scan = ROOT / record["source"]
    assert scan.is_relative_to(ROOT) and scan.parent == SOURCE
    assert hashlib.sha256(scan.read_bytes()).hexdigest() == record["sha256"]
    assert (ROOT / record["constants_source"]).is_file()


def test_world_compiles_from_repository_assets() -> None:
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    assert model.nmesh == 6 and model.body("cube").id > 0
    driver, arm = make_driver()
    try:
        assert driver.model.camera("front").id >= 0
        assert driver.model.camera(f"{arm.name}/wrist").id >= 0
    finally:
        driver.close()
