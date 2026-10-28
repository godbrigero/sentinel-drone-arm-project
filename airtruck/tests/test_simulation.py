"""Shared-world scheduling, mixed control schemas and standard Gymnasium behavior."""

from pathlib import Path
import xml.etree.ElementTree as ET

import gymnasium as gym
import mujoco
import numpy as np
import pytest
from gymnasium import spaces
from gymnasium.utils.env_checker import check_env

from airtruck.examples.two_arms import motion, two_arms
from airtruck.runtime.driver import SystemDriver
from airtruck.runtime.gymnasium import GymnasiumAdapter, feature_names
from airtruck.systems.drone import Drone
from airtruck.systems.so101 import SO101Follower, SO101Leader


def single_env() -> gym.Env:
    return SystemDriver([SO101Follower("arm")]).as_gymnasium(flatten=True)


def test_object_controls_share_one_clock_and_hold_other_targets() -> None:
    left, right = two_arms()
    with SystemDriver([left, right], collisions=False) as driver:
        driver.reset()
        untouched = right.controls["gripper.pos"]
        left.controls["shoulder_pan.pos"] = 20
        right.controls["shoulder_pan.pos"] = -10
        assert driver.data.time == 0
        driver.step()
        assert driver.data.time == pytest.approx(0.02)
        assert right.controls["gripper.pos"] == untouched
        for _ in range(99):
            driver.step()
        assert left.state["shoulder_pan.pos"] > 15
        assert right.state["shoulder_pan.pos"] < -5
        assert not np.any(driver.data.warning.number)


@pytest.mark.parametrize(
    "bad",
    ["unknown", "missing", "shape", "nan", "range", "precision", "complex", "gripper"],
)
def test_invalid_controls_cannot_partially_advance_world(bad: str) -> None:
    arms = two_arms()
    with SystemDriver(arms) as driver:
        driver.reset()
        arms[0].controls["shoulder_pan.pos"] = 20
        if bad == "unknown":
            arms[1].controls["unknown"] = 1
        elif bad == "missing":
            del arms[1].controls["gripper.pos"]
        elif bad == "shape":
            arms[1].controls["shoulder_pan.pos"] = [1, 2]
        elif bad == "nan":
            arms[1].controls["shoulder_pan.pos"] = np.nan
        elif bad == "range":
            arms[1].controls["shoulder_pan.pos"] = 101
        elif bad == "precision":
            arms[1].controls["shoulder_pan.pos"] = 100.0000001
        elif bad == "complex":
            arms[1].controls["shoulder_pan.pos"] = np.complex64(0.5 + 1j)
        else:
            arms[1].controls["gripper.pos"] = -1
        before = driver.data.ctrl.copy()
        with pytest.raises(ValueError):
            driver.step()
        np.testing.assert_array_equal(driver.data.ctrl, before)
        assert driver.data.time == 0


def test_gymnasium_native_flattening_and_horizon() -> None:
    arm = SO101Follower("arm")
    with SystemDriver([arm]) as driver:
        adapter = GymnasiumAdapter(driver)
        check_env(adapter, skip_render_check=True)
        env = driver.as_gymnasium(flatten=True, max_episode_steps=1)
        observation, _ = env.reset(seed=42)
        assert observation.shape == (6,)
        action = np.asarray(spaces.flatten(adapter.action_space, {"arm": arm.controls}))
        _, reward, terminated, truncated, _ = env.step(action)
        assert reward == 0 and not terminated and truncated
        assert feature_names(adapter.action_space)[0] == "arm.shoulder_pan.pos"
        observation, _ = env.reset(seed=42)
        assert driver.data.time == 0
        assert env.observation_space.contains(observation)


@pytest.mark.parametrize("asynchronous", [False, True])
def test_standard_vector_workers_have_independent_worlds(asynchronous: bool) -> None:
    vector = gym.vector.AsyncVectorEnv if asynchronous else gym.vector.SyncVectorEnv
    env = vector([single_env, single_env])
    try:
        observation, _ = env.reset(seed=42)
        action = observation.copy()
        action[0, 0] = 20
        for _ in range(30):
            observation, _, _, _, _ = env.step(action)
        assert observation[0, 0] > observation[1, 0] + 5
    finally:
        env.close()


def test_cross_arm_contacts_change_the_trajectory() -> None:
    runs = []
    for enabled in (True, False):
        arms = two_arms()
        with SystemDriver(arms, collisions=enabled) as driver:
            driver.reset()
            initial = {arm.name: dict(arm.controls) for arm in arms}
            trajectory, samples, peak = [], 0, 0.0
            for _ in range(400):
                motion(arms, initial, float(driver.data.time))
                driver.step()
                contacts = [
                    c
                    for c in driver.info["contacts"]
                    if {c["body_a"].split("/")[0], c["body_b"].split("/")[0]}
                    == {"left", "right"}
                ]
                samples += len(contacts)
                peak = max([peak] + [c["normal_force_n"] for c in contacts])
                trajectory.append(driver.data.qpos.copy())
            assert not np.any(driver.data.warning.number)
            runs.append((np.stack(trajectory), samples, peak))
    assert runs[0][1] > 10 and runs[0][2] > 0.1
    assert runs[1][1] == 0
    assert np.max(np.linalg.norm(runs[0][0] - runs[1][0], axis=1)) > 0.005


def test_arm_and_free_body_use_different_controls_in_one_world(
    drone_model: Path,
) -> None:
    arm = SO101Follower("arm")
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.5))
    with SystemDriver([arm, drone]) as driver:
        driver.reset()
        initial_z = drone.state["position"][2]
        arm.controls["shoulder_pan.pos"] = 20
        drone.takeoff(0.5)
        drone.controls["yaw"] = 0.3
        driver.step()
        assert driver.model.nq == 13 and driver.model.nu == 10
        assert driver.data.time == pytest.approx(0.02)
        assert drone.state["position"][2] < initial_z
        assert abs(drone.state["quaternion"][3]) > 0
        assert arm.state["shoulder_pan.pos"] > 0
        adapter = GymnasiumAdapter(driver)
        assert spaces.flatdim(adapter.action_space) == 10
        assert spaces.flatdim(adapter.observation_space) == 22
        assert not np.any(driver.data.warning.number)


def test_disarmed_drone_falls_and_makes_native_floor_contact(drone_model: Path) -> None:
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.2))
    with SystemDriver([drone]) as driver:
        driver.reset()
        drone.controls["position"] = np.array([0.4, 0, -2.0])
        for _ in range(50):
            driver.step()
        assert drone.state["position"][2] == pytest.approx(-0.03, abs=0.003)
        assert drone.state["landed"] and not drone.state["armed"]
        assert driver.info["contacts"]


def test_leader_is_an_input_and_immediate_commands_do_not_advance_time() -> None:
    leader = SO101Leader("leader", position=(-0.25, 0, 0), follower="follower")
    follower = SO101Follower("follower", position=(0.25, 0, 0))
    with SystemDriver([leader, follower]) as driver:
        driver.reset()
        leader.controls["shoulder_pan.pos"] = 20
        for _ in range(30):
            driver.step()
        assert leader.state["shoulder_pan.pos"] > 15
        assert follower.state["shoulder_pan.pos"] > 10
        assert tuple(GymnasiumAdapter(driver).action_space.spaces) == ("follower",)
        before = driver.data.time
        follower.send_action({"shoulder_pan.pos": 10})
        assert driver.data.time == before


def test_leader_follower_link_is_validated_and_callback_overrides_it() -> None:
    with pytest.raises(TypeError, match="SO101Follower or its system name"):
        SO101Leader("leader", follower=SO101Leader("other"))
    outside = SO101Follower("follower", position=(0.25, 0, 0))
    with pytest.raises(ValueError, match="'follower' is not in this driver"):
        SystemDriver([SO101Leader("leader", follower=outside)])
    with pytest.raises(ValueError, match="'follower' is not in this driver"):
        SystemDriver(
            [
                SO101Leader("leader", follower=outside),
                SO101Follower("follower", position=(0.25, 0, 0)),
            ]
        )
    with pytest.raises(ValueError, match="SO101Follower named 'missing'"):
        SystemDriver([SO101Leader("leader", follower="missing")])
    with pytest.raises(ValueError, match="SO101Follower named 'other'"):
        SystemDriver(
            [
                SO101Leader("leader", follower="other"),
                SO101Leader("other", position=(0.25, 0, 0)),
            ]
        )
    for by_object in (False, True):
        follower = SO101Follower("follower", position=(0.25, 0, 0))
        leader = SO101Leader(
            "leader",
            position=(-0.25, 0, 0),
            follower=follower if by_object else "follower",
        )
        with SystemDriver([follower, leader]) as driver:
            driver.reset()
            leader.controls["shoulder_pan.pos"] = 20
            linked: list[bool] = []

            def update() -> None:
                linked.append(follower.controls == leader.state)
                follower.controls["shoulder_pan.pos"] = -20

            for _ in range(3):
                driver.step(update=update)
            assert linked == [True] * 3 and leader.state["shoulder_pan.pos"] > 5
            assert follower.controls["shoulder_pan.pos"] == -20


def test_run_reset_and_lifetime() -> None:
    arm = SO101Follower("arm")
    driver = SystemDriver([arm])
    with pytest.raises(gym.error.ResetNeeded):
        driver.step()
    driver.reset(seed=42)
    initial = dict(arm.state)
    count = 0

    def update() -> None:
        nonlocal count
        count += 1
        arm.controls["shoulder_pan.pos"] = 20

    driver.run(update=update, steps=3, realtime=False)
    assert count == 3 and driver.data.time == pytest.approx(0.06)
    driver.reset(seed=42)
    assert arm.state == initial
    driver.close()
    driver.close()
    assert not arm.is_connected
    with pytest.raises(RuntimeError, match="closed"):
        driver.reset()


def test_driver_rejects_duplicate_systems_and_second_owner() -> None:
    with pytest.raises(ValueError, match="unique"):
        SystemDriver([SO101Follower("arm"), SO101Follower("arm")])
    arm = SO101Follower("arm")
    with SystemDriver([arm]):
        with pytest.raises(ValueError, match="only one"):
            SystemDriver([arm])


def test_reset_seeds_object_owned_randomization() -> None:
    class RandomArm(SO101Follower):
        def reset(self) -> None:
            self.initial_qpos["shoulder_pan"] = float(self.rng.uniform(-0.1, 0.1))
            super().reset()

    arm = RandomArm("arm")
    with SystemDriver([arm]) as driver:
        driver.reset(seed=42)
        first = dict(arm.state)
        driver.reset(seed=42)
        assert arm.state == first
        driver.reset(seed=43)
        assert arm.state != first


def test_drone_velocity_matches_pose_origin_with_offset_center_of_mass(
    drone_model: Path,
) -> None:
    xml = ET.parse(drone_model)
    inertia, site = xml.find("worldbody/body/inertial"), xml.find("worldbody/body/site")
    assert inertia is not None and site is not None
    inertia.set("pos", ".01 0 0")
    site.set("pos", ".01 0 0")
    xml.write(drone_model)
    drone = Drone("drone", model=drone_model, position=(0.4, 0, 0.5))
    with SystemDriver([drone]) as driver:
        driver.reset()
        driver.data.qvel[5] = 1
        mujoco.mj_forward(driver.model, driver.data)
        drone.observe()
        np.testing.assert_allclose(
            drone.state["linear_velocity_m_s"], [0, 0, 0], atol=1e-8
        )
        np.testing.assert_allclose(drone.state["angular_velocity_rad_s"], [0, 0, -1])
