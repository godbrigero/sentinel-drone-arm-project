from airtruck.runtime.driver import SystemDriver
from airtruck.systems.sentinel import SentinelDrone
from airtruck.examples.constants import IS_HARDWARE


def main() -> None:
    drone = SentinelDrone("drone", hardware=IS_HARDWARE)

    with SystemDriver([drone], control_dt=0.02, render_mode="human") as driver:
        driver.reset()

        drone.takeoff(0.5)
        driver.run(steps=250, realtime=True)
        drone.controls["position"] = [0.0, 0.5, -0.5]
        driver.run(steps=300, realtime=True)
        drone.land()
        driver.run(steps=250, realtime=True)


if __name__ == "__main__":
    main()
