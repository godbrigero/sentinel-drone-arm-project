from airtruck.runtime.driver import SystemDriver
from airtruck.systems.sentinel import SentinelDrone


def main():
    drone = SentinelDrone("drone")

    with SystemDriver([drone], control_dt=0.02, render_mode="human") as driver:
        driver.reset()

        drone.takeoff(1.0)
        driver.run(steps=250, realtime=True)
        drone.controls["position"] = [1.0, 0.0, -1.0]
        driver.run(steps=300, realtime=True)
        drone.land()
        driver.run(steps=250, realtime=True)


if __name__ == "__main__":
    main()
