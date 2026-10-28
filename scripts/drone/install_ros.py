"""Install native ROS/PX4 bindings and expose them to the persistent uv environment."""

import os
import shutil
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
    prefix = (
        Path(os.environ.get("PIXI_HOME", str(Path.home() / ".pixi")))
        / "envs"
        / config["environment"]
    )
    # pixi global install merges into an existing environment's specs, so recreate
    # it to apply exactly the pinned packages (and drop any other ROS distribution).
    if prefix.exists():
        subprocess.run(
            ["pixi", "global", "uninstall", config["environment"]], check=True
        )
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
    env = os.environ.copy()
    env.update(
        PATH=str(prefix / "bin") + os.pathsep + env["PATH"],
        ROS_DISTRO=config["distribution"],
        AMENT_PREFIX_PATH=str(prefix),
        CMAKE_PREFIX_PATH=str(prefix),
        # rcutils exports a bare -latomic, which the system linker resolves here.
        LIBRARY_PATH=str(prefix / "lib"),
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
                # px4_msgs requests linter tests; the SDK ships no ament linters.
                "-DBUILD_TESTING=OFF",
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
        # Remove a previous px4_msgs revision so no stale message types remain.
        for pattern in (
            "include/px4_msgs",
            "share/px4_msgs",
            "share/ament_index/resource_index/*/px4_msgs",
            "lib/libpx4_msgs__*",
            "lib/python3.12/site-packages/px4_msgs*",
        ):
            for path in prefix.glob(pattern):
                if path.is_dir() and not path.is_symlink():
                    shutil.rmtree(path)
                else:
                    path.unlink()
        subprocess.run(
            [str(prefix / "bin/cmake"), "--install", str(build)], env=env, check=True
        )
    site = Path(sys.prefix) / "lib/python3.12/site-packages"
    binding_path = str(prefix / "lib/python3.12/site-packages")
    content = (
        binding_path
        + "\n"
        + f"import os; os.environ.setdefault('AMENT_PREFIX_PATH', {str(prefix)!r}); os.environ.setdefault('ROS_DISTRO', {config['distribution']!r}); os.environ.setdefault('RMW_IMPLEMENTATION', {config['rmw_implementation']!r})\n"
    )
    destination = site / "airtruck_ros.pth"
    destination.write_text(content)
    if destination.read_text() != content:
        raise RuntimeError("ROS path registration did not verify")
    subprocess.run(
        [
            sys.executable,
            "-c",
            "import rclpy; from rclpy.serialization import serialize_message; from rclpy.utilities import get_rmw_implementation_identifier; from px4_msgs.msg import TrajectorySetpoint; from geometry_msgs.msg import PoseStamped; print(rclpy.__file__); print('RMW:', get_rmw_implementation_identifier()); print('Tag pose bytes:', len(serialize_message(PoseStamped()))); print('PX4 native serialization bytes:', len(serialize_message(TrajectorySetpoint())))",
        ],
        check=True,
    )


if __name__ == "__main__":
    main()
