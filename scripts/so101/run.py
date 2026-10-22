"""Small motion demonstration, six independent controls, and MuJoCo render recording."""

from pathlib import Path
import argparse, json, time, subprocess
import numpy as np
import mujoco
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from airtruck.raw_api.so101.controller import JOINTS
from airtruck.raw_api.so101.io import ArmController
from airtruck.testing_util import load_world



def camera():
    c = mujoco.MjvCamera()
    c.lookat[:] = [0, -0.06, 0.13]
    c.distance = 0.65
    # MuJoCo camera fields require degrees; joint controls use radians.
    c.azimuth = 145
    c.elevation = -23
    return c


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--demo", action="store_true")
    p.add_argument("--record", type=Path)
    p.add_argument("--seconds", type=float, default=18)
    args = p.parse_args()
    with load_world(Path(__file__).resolve().parents[2] / "worlds/so101_bench") as world:
        run(args, world)


def run(args, world):
    bus = world.system("arm", ArmController)
    # Rendering is backend integration; application commands use ArmController.
    m, d = world._model, world._data
    # Warm-up at the saved CAD pose with actual dynamics and gravity.
    world.step(3000)
    selected = [0]

    def key(code):
        if 49 <= code <= 54:
            selected[0] = code - 49
            print("Selected", JOINTS[selected[0]], flush=True)
        elif code in (91, 93):
            i = selected[0]
            bus.set_positions({JOINTS[i]: bus.targets[JOINTS[i]] + (np.pi / 36 if code == 93 else -np.pi / 36)})
        elif code in (82, 114):
            for n in JOINTS:
                bus.set_positions({n: 0})

    print(
        "MuJoCo SO-101: keys 1–6 select motor; [ / ] move target by 0.0872665 rad; R returns to CAD pose.",
        flush=True,
    )
    rows = []
    cam = camera()
    frame = 0
    start = d.time

    def advance():
        nonlocal frame
        elapsed = d.time - start
        if args.demo:
            # One joint at a time, small positive then return. Elbow's positive range is only 0.1192 rad.
            idx = min(int(elapsed // 3), 5)
            phase = elapsed % 3
            for i, n in enumerate(JOINTS):
                bus.set_positions({n: np.pi / 36 if i == idx and 0.3 < phase < 1.6 else 0})
        world.step(33)
        if args.record:
            rows.append(
                {
                    "time_s": float(d.time - start),
                    "target_rad": list(bus.targets.values()),
                    "actual_rad": d.qpos.tolist(),
                    "voltage_V": d.ctrl.tolist(),
                }
            )
        frame += 1

    if args.record:
        args.record.parent.mkdir(parents=True, exist_ok=True)
        proc = subprocess.Popen(
            [
                "ffmpeg",
                "-y",
                "-loglevel",
                "error",
                "-f",
                "rawvideo",
                "-pix_fmt",
                "rgb24",
                "-s",
                "960x720",
                "-r",
                "30",
                "-i",
                "-",
                "-an",
                "-c:v",
                "libx264",
                "-pix_fmt",
                "yuv420p",
                str(args.record),
            ],
            stdin=subprocess.PIPE,
        )
        with mujoco.Renderer(m, height=720, width=960) as render:
            while d.time - start < args.seconds:
                advance()
                render.update_scene(d, camera=cam)
                pixels = render.render()
                from PIL import Image, ImageDraw

                img = Image.fromarray(pixels)
                draw = ImageDraw.Draw(img)
                draw.rectangle((0, 0, 960, 98), fill=(18, 24, 32))
                draw.text(
                    (20, 12),
                    "SO-101 | MuJoCo | six independent servo position commands",
                    fill="white",
                    font_size=22,
                )
                active = min(int((d.time - start) // 3), 5)
                draw.text(
                    (20, 45),
                    f"Motor {active + 1}: {JOINTS[active]}   target {bus.targets[JOINTS[active]]:.4f} rad   actual {d.qpos[active]:.4f} rad",
                    fill=(94, 220, 190),
                    font_size=20,
                )
                draw.text(
                    (20, 74),
                    "Native Onshape geometry | 7.4 V servo approximation | fixed base",
                    fill=(175, 185, 200),
                    font_size=15,
                )
                pixels = np.array(img)
                proc.stdin.write(pixels.tobytes())
                if frame in (1, 30):
                    from PIL import Image

                    Image.fromarray(pixels).save(
                        args.record.parent
                        / ("mujoco-start.png" if frame == 1 else "mujoco-moved.png")
                    )
        proc.stdin.close()
        assert proc.wait() == 0
        (args.record.parent / "motion-trace.json").write_text(
            json.dumps({"joints": JOINTS, "frames": rows}, indent=2) + "\n"
        )
    else:
        from mujoco import viewer as mjviewer

        with mjviewer.launch_passive(
            m, d, key_callback=key, show_right_ui=False
        ) as viewer:
            viewer.cam.lookat[:] = cam.lookat
            viewer.cam.distance = cam.distance
            viewer.cam.azimuth = cam.azimuth
            viewer.cam.elevation = cam.elevation
            while viewer.is_running():
                tick = time.monotonic()
                with viewer.lock():
                    advance()
                    viewer.opt.geomgroup[2] = 1
                viewer.sync()
                time.sleep(max(0, 0.033 - (time.monotonic() - tick)))
                if args.demo and d.time - start > args.seconds:
                    args.demo = False
                    print("Demo complete; manual controls active.", flush=True)


if __name__ == "__main__":
    main()
