"""Small analytic fixtures exercise the controller API, not Sentinel geometry."""

from pathlib import Path

import pytest


@pytest.fixture
def drone_model(tmp_path: Path) -> Path:
    model = tmp_path / "free_body.xml"
    model.write_text("""<mujoco model="analytic_free_body">
    <compiler angle="radian" inertiafromgeom="false"/>
    <option timestep="0.0005" integrator="implicitfast"/>
    <worldbody><body name="base"><freejoint/>
      <inertial mass=".1" pos="0 0 0" diaginertia=".00006 .00006 .00006"/>
      <geom type="box" size=".03 .03 .03"/>
      <site name="wrench" size=".003"/>
    </body></worldbody>
    <actuator>
      <motor name="thrust_n" site="wrench" gear="0 0 1 0 0 0" ctrlrange="0 20"/>
      <motor name="roll_torque_nm" site="wrench" gear="0 0 0 1 0 0" ctrlrange="-1 1"/>
      <motor name="pitch_torque_nm" site="wrench" gear="0 0 0 0 1 0" ctrlrange="-1 1"/>
      <motor name="yaw_torque_nm" site="wrench" gear="0 0 0 0 0 1" ctrlrange="-1 1"/>
    </actuator></mujoco>""")
    return model
