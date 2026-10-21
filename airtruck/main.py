from airtruck.constants import *


def main() -> None:
    if RUN_MODE == RunMode.SIMULATION:
        print("Running in simulation mode")
    else:
        print("Running in real mode")
