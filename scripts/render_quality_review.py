"""Capture PyBullet DIRECT/TinyRenderer views used for material review."""
from pathlib import Path
import sys

import numpy as np
import pybullet as p
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from engine import config
from engine.premium_environment import PremiumEnvironment
from scripts.diff_drive_robot import DiffDriveRobot


def main():
    output = Path(__file__).resolve().parents[1] / "visual_review"
    output.mkdir(exist_ok=True)
    client = p.connect(p.DIRECT)
    try:
        env = PremiumEnvironment(1234).build()
        sx, sy, yaw = env.robot_start
        robot = DiffDriveRobot(str(Path(__file__).resolve().parents[1] /
                                   "urdf" / "diff_drive_robot.urdf"),
                               [sx, sy, config.ROBOT_BASE_Z], yaw)

        def save_view(name, eye, target, fov=55, up=(0, 0, 1)):
            view = p.computeViewMatrix(eye, target, up)
            projection = p.computeProjectionMatrixFOV(fov, 1280 / 960, 0.01, 20)
            _, _, rgba, _, _ = p.getCameraImage(1280, 960, view, projection,
                                                renderer=p.ER_TINY_RENDERER)
            pixels = np.asarray(rgba, dtype=np.uint8).reshape(960, 1280, 4)
            Image.fromarray(pixels).convert("RGB").save(output / f"{name}.png")

        save_view("01_full_environment", [8, -9, 8], [0, 0, 0.25])
        rgb, _ = robot.get_camera_image()
        Image.fromarray(rgb, "RGB").save(output / "02_onboard_camera.png")
        save_view("03_floor_closeup", [0, 0, 2.8], [0, 0, 0], 48, (0, 1, 0))
        save_view("04_wall_detail", [0, 1.35, 0.7], [0, 2.9, 0.55], 60)
        save_view("05_drop_zone", [*env.drop_center, 2.5], [*env.drop_center, 0], 48, (0, 1, 0))
        print(f"Captured 5 PyBullet views to {output}; walls={len(env.wall_ids)}, "
              f"objects={len(env.object_ids)}")
    finally:
        p.disconnect(client)


if __name__ == "__main__":
    main()
