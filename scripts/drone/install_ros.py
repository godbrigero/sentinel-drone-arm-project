"""Install native ROS/PX4 bindings and expose them to the persistent uv environment."""

import os
import subprocess
import sys
import tempfile
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def main() -> None:
    if Path(sys.prefix).resolve() != (ROOT / ".venv").resolve():
        raise RuntimeError(
            "Run with uv run --locked python scripts/drone/install_ros.py"
        )
    config = tomllib.loads((ROOT / "pyproject.toml").read_text())["tool"]["airtruck"][
        "ros"
    ]
    subprocess.run(
        [
            "pixi",
            "global",
            "install",
            "--environment",
            config["environment"],
            "--expose",
            "airtruck-ros-python=python",
            "--channel",
            "https://prefix.dev/robostack-" + config["distribution"],
            "--channel",
            "conda-forge",
            *config["packages"],
        ],
        check=True,
    )
    prefix = (
        Path(os.environ.get("PIXI_HOME", str(Path.home() / ".pixi")))
        / "envs"
        / config["environment"]
    )
    env = os.environ.copy()
    env.update(
        PATH=str(prefix / "bin") + os.pathsep + env["PATH"],
        ROS_DISTRO=config["distribution"],
        AMENT_PREFIX_PATH=str(prefix),
        CMAKE_PREFIX_PATH=str(prefix),
    )
    with tempfile.TemporaryDirectory(prefix="airtruck-px4-build-") as temporary:
        source, build = Path(temporary) / "px4_msgs", Path(temporary) / "build"
        subprocess.run(
            [
                "git",
                "clone",
                "--no-checkout",
                "https://github.com/PX4/px4_msgs.git",
                str(source),
            ],
            check=True,
        )
        subprocess.run(
            ["git", "-C", str(source), "checkout", "--detach", config["px4_revision"]],
            check=True,
        )
        subprocess.run(
            [
                str(prefix / "bin/cmake"),
                "-S",
                str(source),
                "-B",
                str(build),
                "-G",
                "Ninja",
                "-DCMAKE_INSTALL_PREFIX=" + str(prefix),
                "-DCMAKE_INSTALL_RPATH=" + str(prefix / "lib"),
                "-DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON",
                "-DPython3_EXECUTABLE=" + str(prefix / "bin/python"),
                "-DPYTHON_EXECUTABLE=" + str(prefix / "bin/python"),
            ],
            env=env,
            check=True,
        )
        subprocess.run(
            [str(prefix / "bin/cmake"), "--build", str(build), "--parallel", "6"],
            env=env,
            check=True,
        )
        subprocess.run(
            [str(prefix / "bin/cmake"), "--install", str(build)], env=env, check=True
        )
    site = Path(sys.prefix) / "lib/python3.12/site-packages"
    binding_path = str(prefix / "lib/python3.12/site-packages")
    content = (
        binding_path
        + "\n"
        + f"import os; os.environ.setdefault('AMENT_PREFIX_PATH', {str(prefix)!r}); os.environ.setdefault('ROS_DISTRO', {config['distribution']!r})\n"
    )
    destination = site / "airtruck_ros.pth"
    destination.write_text(content)
    if destination.read_text() != content:
        raise RuntimeError("ROS path registration did not verify")
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import rclpy; from rclpy.serialization import serialize_message; from px4_msgs.msg import TrajectorySetpoint; print(rclpy.__file__); print('PX4 native serialization bytes:', len(serialize_message(TrajectorySetpoint())))",
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
