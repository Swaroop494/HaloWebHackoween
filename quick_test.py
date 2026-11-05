#!/usr/bin/env python3
"""Quick visual test to verify rendering."""
import pybullet as p
import pybullet_data
from engine.fetch_environment import FetchEnvironment
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetDebugVisualizerCamera(7, 45, -25, [0, 0, 0.5])

print("Building factory...")
env = FetchEnvironment(seed=42)
env.build()

print(f"Floor: {env.floor_id}")
print(f"Walls: {len(env.wall_ids)}")
print(f"Objects: {len(env.object_ids)}")
print(f"Drop zone: {env.drop_id}")

# Load robot
robot = p.loadURDF("urdf/diff_drive_robot.urdf", [0, 0, 0.1], [0, 0, 0, 1])
print(f"Robot: {robot}")

# Settle
for _ in range(240):
    p.stepSimulation()

print("\nRendering for 5 seconds...")
time.sleep(5)

p.disconnect()
print("Complete!")