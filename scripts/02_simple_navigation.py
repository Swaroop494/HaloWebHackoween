#!/usr/bin/env python3
"""Simple navigation test - robot moves to a goal position."""
import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot

class SimpleNavigationController:
    """Simple proportional controller for goal navigation."""
    
    def __init__(self, goal_threshold=0.3):
        self.goal_threshold = goal_threshold
        self.kp_linear = 0.5
        self.kp_angular = 2.0
        self.max_linear_vel = 0.5
        self.max_angular_vel = 2.0
    
    def compute_control(self, robot_pose, goal):
        """
        Compute velocity commands to reach goal.
        
        Args:
            robot_pose: [x, y, theta] current pose
            goal: [x, y] goal position
            
        Returns:
            v, w, reached: linear velocity, angular velocity, goal reached flag
        """
        # Calculate distance and angle to goal
        dx = goal[0] - robot_pose[0]
        dy = goal[1] - robot_pose[1]
        distance_to_goal = np.sqrt(dx**2 + dy**2)
        
        if distance_to_goal < self.goal_threshold:
            return 0.0, 0.0, True  # Goal reached
        
        # Desired heading to goal
        desired_heading = np.arctan2(dy, dx)
        heading_error = desired_heading - robot_pose[2]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Proportional control
        v = min(self.max_linear_vel, self.kp_linear * distance_to_goal)
        w = np.clip(self.kp_angular * heading_error, -self.max_angular_vel, self.max_angular_vel)
        
        # Slow down when turning sharply
        if abs(heading_error) > 0.5:
            v *= 0.5
        
        return v, w, False

def main():
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Camera setup
    p.resetDebugVisualizerCamera(
        cameraDistance=8,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[2, 2, 0]
    )
    
    # Load environment
    plane_id = p.loadURDF("plane.urdf")
    
    # Create goal marker
    goal = [4, 4]
    goal_visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.2,
        rgbaColor=[0.2, 0.8, 0.2, 0.7]
    )
    goal_marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=goal_visual,
        basePosition=[goal[0], goal[1], 0.2]
    )
    
    # Create robot
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    # Controller
    controller = SimpleNavigationController()
    
    print("Starting simple navigation test...")
    print(f"Goal: ({goal[0]}, {goal[1]})")
    print("Robot will navigate to the green sphere")
    
    step = 0
    max_steps = 5000
    goal_reached = False
    
    try:
        while step < max_steps and not goal_reached:
            # Get robot state
            pose = robot.get_pose()
            
            # Compute control
            v, w, reached = controller.compute_control(pose, goal)
            goal_reached = reached
            
            if reached:
                print(f"\n✓ Goal reached at step {step}!")
                print(f"Final position: ({pose[0]:.2f}, {pose[1]:.2f})")
                break
            
            # Apply control
            robot.set_velocity(v, w)
            
            # Step simulation
            for _ in range(10):  # 10 physics steps per control step
                p.stepSimulation()
            
            step += 1
            
            # Print status
            if step % 100 == 0:
                dist = np.sqrt((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)
                print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                      f"dist_to_goal={dist:.2f}m, v={v:.2f}, w={w:.2f}")
            
            time.sleep(1/60)
        
        if not goal_reached:
            print(f"\nMax steps reached ({max_steps}). Stopping.")
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    p.disconnect()

if __name__ == "__main__":
    main()

