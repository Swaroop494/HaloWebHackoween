#!/usr/bin/env python3
"""Navigate robot through warehouse/street environments with obstacle avoidance."""
import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time
import math

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot
# Import environment builder using importlib since filename starts with number
import importlib.util
env_builder_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                "scripts", "05_environment_builder.py")
spec = importlib.util.spec_from_file_location("environment_builder", env_builder_path)
env_builder_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(env_builder_module)
EnvironmentBuilder = env_builder_module.EnvironmentBuilder

class NavigationController:
    """Advanced navigation controller with obstacle avoidance."""
    
    def __init__(self, goal_threshold=0.4):
        self.goal_threshold = goal_threshold
        self.kp_linear = 0.8
        self.kp_angular = 2.5
        self.max_linear_vel = 0.5
        self.max_angular_vel = 2.0
        
        # Obstacle avoidance parameters
        self.safety_distance = 0.6  # Increased safety distance
        self.critical_distance = 0.4  # Very close - stop and turn
        self.obstacle_repulsion = 1.5
        self.min_turn_distance = 0.3
        
        # State memory to prevent oscillation
        self.last_turn_direction = 0  # -1 for left, 1 for right, 0 for none
        self.stuck_counter = 0
        self.last_position = None
        self.stuck_threshold = 0.1  # Consider stuck if moved less than this in 50 steps
        self.position_history = []
    
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
        
        # Check if robot is stuck (oscillating)
        current_pos = np.array([robot_pose[0], robot_pose[1]])
        self.position_history.append(current_pos)
        if len(self.position_history) > 50:
            self.position_history.pop(0)
        
        if len(self.position_history) >= 20:
            # Check if robot has moved significantly
            recent_positions = np.array(self.position_history[-20:])
            movement = np.max(np.linalg.norm(recent_positions - recent_positions[0], axis=1))
            if movement < self.stuck_threshold:
                self.stuck_counter += 1
                # When stuck, commit to a direction and turn more aggressively
                if self.last_turn_direction == 0:
                    # Choose initial direction based on space
                    left_space = np.mean(lidar_scan[:num_rays//2])
                    right_space = np.mean(lidar_scan[num_rays//2:])
                    self.last_turn_direction = -1 if left_space > right_space else 1
                
                # Turn in committed direction
                v = 0.1
                w = self.last_turn_direction * self.max_angular_vel * 0.8
                return v, w, False
            else:
                self.stuck_counter = 0
                self.last_turn_direction = 0  # Reset when moving
        else:
            self.stuck_counter = 0
        
        # Check for critical obstacle (very close)
        if min_lidar < self.critical_distance:
            # Stop and turn away from obstacle
            v = 0.0
            
            # Find direction with most space
            left_space = np.mean(lidar_scan[:num_rays//2])
            right_space = np.mean(lidar_scan[num_rays//2:])
            
            if left_space > right_space:
                w = -self.max_angular_vel * 0.9  # Turn left
                self.last_turn_direction = -1
            else:
                w = self.max_angular_vel * 0.9   # Turn right
                self.last_turn_direction = 1
        elif front_obstacle:
            # Obstacle in front - turn away
            v = 0.2  # Slow down significantly
            
            # Find direction with most space
            left_space = np.mean(lidar_scan[:num_rays//2])
            right_space = np.mean(lidar_scan[num_rays//2:])
            
            # Use state memory to prevent oscillation
            if abs(left_space - right_space) < 0.3:  # Similar space on both sides
                # Use last turn direction to maintain consistency
                if self.last_turn_direction == 0:
                    self.last_turn_direction = -1 if left_space > right_space else 1
                w = self.last_turn_direction * self.max_angular_vel * 0.7
            else:
                # Clear preference - turn toward more space
                if left_space > right_space:
                    w = -self.max_angular_vel * 0.7
                    self.last_turn_direction = -1
                else:
                    w = self.max_angular_vel * 0.7
                    self.last_turn_direction = 1
        else:
            # No immediate obstacle - navigate to goal
            v = min(self.max_linear_vel, self.kp_linear * distance_to_goal)
            w = np.clip(self.kp_angular * heading_error, -self.max_angular_vel, self.max_angular_vel)
            
            # Reset turn direction memory when clear path
            if min_lidar > self.safety_distance * 1.5:
                self.last_turn_direction = 0
            
            # Slow down if obstacle is close
            if min_lidar < self.safety_distance * 1.5:
                v *= max(0.3, min_lidar / (self.safety_distance * 1.5))
            
            # Slow down when turning sharply
            if abs(heading_error) > 0.5:
                v *= 0.5
        
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
    debug_items = []  # Track debug items for cleanup
    
    try:
        while step < max_steps and not goal_reached:
            # Get robot state
            pose = robot.get_pose()
            lidar = robot.get_lidar_scan()
            imu_data = robot.get_imu_data()
            odom_data = robot.get_odometry()
            
            # Visualize sensors in PyBullet GUI
            # Clear previous debug items
            for item in debug_items:
                try:
                    p.removeUserDebugItem(item)
                except:
                    pass
            debug_items.clear()
            
            # Draw lidar rays (every 3rd ray to reduce clutter)
            num_rays = len(lidar)
            ray_from = []
            ray_to = []
            colors = []
            
            for i in range(0, num_rays, 3):  # Every 3rd ray
                dist = lidar[i]
                angle = pose[2] + (2*np.pi * i / num_rays)
                
                start = [pose[0], pose[1], 0.11]
                end = [
                    pose[0] + dist * np.cos(angle),
                    pose[1] + dist * np.sin(angle),
                    0.11
                ]
                
                ray_from.append(start)
                ray_to.append(end)
                
                # Color based on distance (red=close, green=far)
                intensity = min(1.0, dist / 5.0)
                colors.append([1.0 - intensity, intensity, 0.0])
            
            # Draw rays
            if ray_from:
                line_id = p.addUserDebugLines(
                    ray_from, ray_to, 
                    lineColorsRGB=colors, 
                    lineWidth=2, 
                    lifeTime=0.15
                )
                debug_items.append(line_id)
            
            # Add text overlay with sensor data (above robot)
            min_lidar = np.min(lidar)
            text = f"Min Lidar: {min_lidar:.2f}m | Vel: {odom_data['velocity'][0]:.2f}m/s | Ang: {odom_data['velocity'][2]:.2f}rad/s"
            text_id = p.addUserDebugText(
                text,
                textPosition=[pose[0], pose[1], 0.6],
                textColorRGB=[1, 1, 0],
                textSize=1.2,
                lifeTime=0.15
            )
            debug_items.append(text_id)
            
            # Check for collision
            collision_detected = robot.check_collision()
            if collision_detected:
                collision_count += 1
                # If stuck in collision, back up and turn
                if collision_count > 5:
                    print(f"Collision detected at step {step}, backing up...")
                    # Back up
                    robot.set_velocity(-0.2, 0)
                    for _ in range(20):  # Back up for a bit
                        p.stepSimulation()
                    # Turn away
                    left_space = np.mean(lidar[:len(lidar)//2])
                    right_space = np.mean(lidar[len(lidar)//2:])
                    turn_vel = -0.8 if left_space > right_space else 0.8
                    robot.set_velocity(0, turn_vel)
                    for _ in range(30):  # Turn for a bit
                        p.stepSimulation()
                    collision_count = 0
                    continue  # Skip this step's control
            else:
                collision_count = 0  # Reset counter if no collision
            
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

