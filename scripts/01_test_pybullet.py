#!/usr/bin/env python3
"""Test PyBullet with differential drive robot."""
import pybullet as p
import pybullet_data
import time
import numpy as np
import os
import sys

# Add parent directory to path to import diff_drive_robot
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot

def main():
    # Connect to PyBullet (GUI mode)
    physics_client = p.connect(p.GUI)
    
    # Set up simulation
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Camera setup
    p.resetDebugVisualizerCamera(
        cameraDistance=5,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")
    
    # Load robot
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    print("Robot loaded successfully!")
    print(f"Left wheel joint: {robot.left_wheel_joint}")
    print(f"Right wheel joint: {robot.right_wheel_joint}")
    print("\nRunning simulation... Press Ctrl+C to exit")
    
    # Simple motion test
    try:
        for step in range(10000):
            # Apply wheel velocities
            if step < 2000:
                # Move forward
                v = 0.3
                w = 0.0
            elif step < 4000:
                # Turn right
                v = 0.2
                w = -1.0
            elif step < 6000:
                # Turn left
                v = 0.2
                w = 1.0
            else:
                # Move forward again
                v = 0.3
                w = 0.0
            
            # Set velocity
            robot.set_velocity(v, w)
            
            # Step simulation
            p.stepSimulation()
            
            # Get robot position
            pose = robot.get_pose()
            
            if step % 500 == 0:
                print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), yaw={np.degrees(pose[2]):.1f}°")
            
            time.sleep(1/240)
    
    except KeyboardInterrupt:
        print("\nSimulation stopped.")
    
    p.disconnect()

if __name__ == "__main__":
    main()

