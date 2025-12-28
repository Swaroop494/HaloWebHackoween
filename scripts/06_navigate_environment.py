#!/usr/bin/env python3
"""Navigate robot through warehouse/street environments with obstacle avoidance."""
import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot
from scripts.05_environment_builder import EnvironmentBuilder

class NavigationController:
    """Advanced navigation controller with obstacle avoidance."""
    
    def __init__(self, goal_threshold=0.4):
        self.goal_threshold = goal_threshold
        self.kp_linear = 0.8
        self.kp_angular = 2.5
        self.max_linear_vel = 0.5
        self.max_angular_vel = 2.0
        
        # Obstacle avoidance parameters
        self.safety_distance = 0.5
        self.obstacle_repulsion = 1.5
        self.min_turn_distance = 0.3
    
    def compute_control(self, robot_pose, goal, lidar_scan):
        """
        Compute velocity commands with obstacle avoidance.
        
        Args:
            robot_pose: [x, y, theta] current pose
            goal: [x, y] goal position
            lidar_scan: Array of lidar distances
            
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
        
        # Obstacle avoidance using lidar
        min_lidar = np.min(lidar_scan)
        num_rays = len(lidar_scan)
        
        # Find obstacles in front and sides
        front_obstacle = False
        left_obstacle = False
        right_obstacle = False
        
        for i, dist in enumerate(lidar_scan):
            if dist < self.safety_distance:
                # Ray angle relative to robot heading
                ray_angle = (2*np.pi * i / num_rays)
                ray_angle_rel = ray_angle - np.pi  # -pi to pi, 0 is front
                
                # Normalize to -pi to pi
                ray_angle_rel = np.arctan2(np.sin(ray_angle_rel), np.cos(ray_angle_rel))
                
                # Check front (within 60 degrees)
                if abs(ray_angle_rel) < np.pi/3:
                    front_obstacle = True
                # Check left
                elif ray_angle_rel > np.pi/6 and ray_angle_rel < np.pi/2:
                    left_obstacle = True
                # Check right
                elif ray_angle_rel < -np.pi/6 and ray_angle_rel > -np.pi/2:
                    right_obstacle = True
        
        # Obstacle avoidance behavior
        if front_obstacle:
            # Obstacle in front - turn away
            if left_obstacle and not right_obstacle:
                # Turn right
                w = self.max_angular_vel
                v = 0.1  # Slow down
            elif right_obstacle and not left_obstacle:
                # Turn left
                w = -self.max_angular_vel
                v = 0.1
            else:
                # Turn in direction with more space
                left_space = np.mean(lidar_scan[:num_rays//2])
                right_space = np.mean(lidar_scan[num_rays//2:])
                if left_space > right_space:
                    w = -self.max_angular_vel * 0.8
                else:
                    w = self.max_angular_vel * 0.8
                v = 0.15
        else:
            # No immediate obstacle - navigate to goal
            v = min(self.max_linear_vel, self.kp_linear * distance_to_goal)
            w = np.clip(self.kp_angular * heading_error, -self.max_angular_vel, self.max_angular_vel)
            
            # Slow down if obstacle is close
            if min_lidar < self.safety_distance * 1.5:
                v *= (min_lidar / (self.safety_distance * 1.5))
            
            # Slow down when turning sharply
            if abs(heading_error) > 0.5:
                v *= 0.6
        
        return v, w, False

def main():
    """Main navigation test in environment."""
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Camera setup
    p.resetDebugVisualizerCamera(
        cameraDistance=15,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Load ground
    plane_id = p.loadURDF("plane.urdf")
    
    # Create environment
    builder = EnvironmentBuilder(physics_client)
    
    # Choose environment type
    env_type = sys.argv[1] if len(sys.argv) > 1 else "warehouse"
    
    print(f"Creating {env_type} environment...")
    if env_type == "warehouse":
        obstacles = builder.create_warehouse_environment()
    elif env_type == "street":
        obstacles = builder.create_street_environment()
    elif env_type == "maze":
        obstacles = builder.create_simple_maze()
    else:
        print(f"Unknown environment: {env_type}, using warehouse")
        obstacles = builder.create_warehouse_environment()
    
    # Create robot at start position
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    # Set goal position (far from start, avoiding obstacles)
    goal = [6, 6]
    
    # Create goal marker
    goal_visual = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.3,
        rgbaColor=[0.2, 0.8, 0.2, 0.8]
    )
    goal_marker = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=goal_visual,
        basePosition=[goal[0], goal[1], 0.3]
    )
    
    # Create navigation controller
    controller = NavigationController()
    
    print(f"\nNavigation Test in {env_type} environment")
    print(f"Goal: ({goal[0]}, {goal[1]})")
    print("Robot will navigate to the green sphere")
    print("Press Ctrl+C to stop\n")
    
    step = 0
    max_steps = 10000
    goal_reached = False
    collision_count = 0
    
    try:
        while step < max_steps and not goal_reached:
            # Get robot state
            pose = robot.get_pose()
            lidar = robot.get_lidar_scan()
            
            # Check for collision
            if robot.check_collision():
                collision_count += 1
                if collision_count > 10:  # Stuck in collision
                    print("Robot stuck in collision, resetting...")
                    robot.reset([0, 0], 0)
                    collision_count = 0
            
            # Compute control
            v, w, reached = controller.compute_control(pose, goal, lidar)
            goal_reached = reached
            
            if reached:
                print(f"\n✓ Goal reached at step {step}!")
                print(f"Final position: ({pose[0]:.2f}, {pose[1]:.2f})")
                break
            
            # Apply control
            robot.set_velocity(v, w)
            
            # Step simulation
            for _ in range(10):
                p.stepSimulation()
            
            step += 1
            
            # Print status
            if step % 200 == 0:
                dist = np.sqrt((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)
                min_lidar = np.min(lidar)
                print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                      f"dist_to_goal={dist:.2f}m, min_obstacle={min_lidar:.2f}m, "
                      f"v={v:.2f}, w={w:.2f}")
            
            time.sleep(1/60)
        
        if not goal_reached:
            print(f"\nMax steps reached ({max_steps}).")
            final_dist = np.sqrt((goal[0]-pose[0])**2 + (goal[1]-pose[1])**2)
            print(f"Final distance to goal: {final_dist:.2f}m")
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    p.disconnect()

if __name__ == "__main__":
    main()

