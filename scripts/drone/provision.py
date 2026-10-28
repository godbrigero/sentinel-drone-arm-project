"""Prepare the Sentinel's VOXL 2 for flight over SSH, using factory SoftAP defaults."""

import paramiko

HOST = "192.168.8.1"
USER = "root"
PASSWORD = "oelinux123"  # VOXL factory default.
# macOS Wi-Fi allows at most 1500; larger DDS fragments never reach the Mac, so ROS
# finds the aircraft but no topics. Not persistent: re-run after every boot.
MTU = 1500
COMMANDS = (
    "voxl-bind-spektrum -s",
    f"ip link set dev wlan0 mtu {MTU}",
    "ip link show wlan0 | grep mtu",
)


def run(client: paramiko.SSHClient, command: str) -> str:
    print(f"$ {command}", flush=True)
    _, stdout, _ = client.exec_command(command, timeout=60, get_pty=True)
    output = stdout.read().decode()
    print(output, end="", flush=True)
    status = stdout.channel.recv_exit_status()
    if status != 0:
        raise RuntimeError(f"{command!r} exited with status {status}")
    return output


def main() -> None:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        client.connect(
            HOST,
            username=USER,
            password=PASSWORD,
            timeout=10,
            look_for_keys=False,
            allow_agent=False,
        )
    except OSError as error:
        raise SystemExit(
            f"Cannot reach {HOST} ({error}); join the aircraft's VOXL-… Wi-Fi network."
        ) from error
    with client:
        outputs = [run(client, command) for command in COMMANDS]
    if f" mtu {MTU} " not in outputs[-1]:
        raise RuntimeError(f"wlan0 MTU is not {MTU}")
    print(f"Ready: Spektrum bind started, wlan0 MTU {MTU}.")


if __name__ == "__main__":
    main()
