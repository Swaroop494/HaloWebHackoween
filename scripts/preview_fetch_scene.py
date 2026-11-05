"""Open the Fetch room in PyBullet's GUI without starting any AI provider."""
from __future__ import annotations

import argparse
import os
import sys
import time

import pybullet as p
import pybullet_data

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from engine import config
from engine.fetch_environment import FetchEnvironment
from scripts.diff_drive_robot import DiffDriveRobot


def main():
    parser = argparse.ArgumentParser(description="Preview the Fetch scene in the PyBullet GUI")
    parser.add_argument("--seed", type=int, default=1234)
    args = parser.parse_args()
    client = p.connect(p.GUI)
    try:
        # Avoid a blank/unresponsive ExampleBrowser while the scene's many
        # visual bodies are being created; enable rendering once it is ready.
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        env = FetchEnvironment(args.seed).build()
        start = env.robot_start
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        robot = DiffDriveRobot(os.path.join(root, "urdf", "diff_drive_robot.urdf"),
                               [start[0], start[1], config.ROBOT_BASE_Z], start[2])
        p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        p.resetDebugVisualizerCamera(cameraDistance=7, cameraYaw=45, cameraPitch=-45,
                                     cameraTargetPosition=[0, 0, 0])
        print("Fetch scene preview is open. Close the PyBullet window or press Ctrl+C to exit.")
        while p.isConnected(client):
            p.getCameraImage(1, 1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        if p.isConnected(client):
            p.disconnect(client)


if __name__ == "__main__":
    main()
