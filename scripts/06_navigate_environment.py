#!/usr/bin/env python3
"""Navigate robot through warehouse/street environments with obstacle avoidance."""
import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time
import math

# Dynamic import for module with numeric name
import importlib.util
spec = importlib.util.spec_from_file_location("env_builder", 
    os.path.join(os.path.dirname(__file__), "05_environment_builder.py"))
env_builder_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(env_builder_module)
EnvironmentBuilder = env_builder_module.EnvironmentBuilder

from scripts.diff_drive_robot import DiffDriveRobot

class PIDNavigationController:
    """Simple PID-based navigation controller with obstacle avoidance."""
    
    def __init__(self, goal_threshold=0.4):
        self.goal_threshold = goal_threshold
        
        # PID gains for goal navigation
        self.kp_linear = 1.0
        self.kp_angular = 2.0
        self.ki_angular = 0.0  # Can add integral term if needed
        self.kd_angular = 0.5  # Derivative term for smoother turns
        
        # Velocity limits
        self.max_linear_vel = 0.5
        self.max_angular_vel = 2.0
        
        # Obstacle avoidance
        self.safety_distance = 0.6
        self.critical_distance = 0.4
        
        # PID state
        self.last_heading_error = 0.0
        
    def compute_control(self, robot_pose, goal, lidar_scan):
        """
        Simple PID controller with obstacle avoidance.
        
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
        
        # Obstacle avoidance - modify desired heading if obstacle detected
        min_lidar = np.min(lidar_scan)
        num_rays = len(lidar_scan)
        
        # Find obstacles in front (within 90 degrees)
        front_obstacle_dist = 5.0  # Max range
        front_obstacle_angle = 0.0
        
        for i, dist in enumerate(lidar_scan):
            if dist < self.safety_distance:
                # Ray angle relative to robot heading
                ray_angle = (2*np.pi * i / num_rays)
                ray_angle_rel = ray_angle - np.pi  # -pi to pi, 0 is front
                ray_angle_rel = np.arctan2(np.sin(ray_angle_rel), np.cos(ray_angle_rel))
                
                # Check front (within 90 degrees)
                if abs(ray_angle_rel) < np.pi/2:
                    if dist < front_obstacle_dist:
                        front_obstacle_dist = dist
                        front_obstacle_angle = ray_angle_rel
        
        # Modify heading if obstacle in front
        if front_obstacle_dist < self.safety_distance:
            # Calculate avoidance direction
            # Find direction with most space
            left_space = np.mean(lidar_scan[:num_rays//2])
            right_space = np.mean(lidar_scan[num_rays//2:])
            
            # Calculate avoidance angle (turn away from obstacle)
            if left_space > right_space:
                # More space on left, turn left
                avoidance_angle = np.pi/2  # 90 degrees left
            else:
                # More space on right, turn right
                avoidance_angle = -np.pi/2  # 90 degrees right
            
            # Blend goal heading with avoidance heading
            # Weight based on obstacle proximity
            obstacle_weight = (self.safety_distance - front_obstacle_dist) / self.safety_distance
            obstacle_weight = min(1.0, obstacle_weight * 1.5)  # Scale up for more aggressive avoidance
            
            # Modify desired heading
            desired_heading = desired_heading + obstacle_weight * avoidance_angle
            heading_error = desired_heading - robot_pose[2]
            heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # PID control for angular velocity
        # Proportional term
        p_term = self.kp_angular * heading_error
        
        # Derivative term (rate of change of error)
        d_error = heading_error - self.last_heading_error
        d_term = self.kd_angular * d_error
        
        # Angular velocity command
        w = p_term + d_term
        w = np.clip(w, -self.max_angular_vel, self.max_angular_vel)
        
        # Update for next iteration
        self.last_heading_error = heading_error
        
        # Linear velocity - proportional to distance, reduced when turning or near obstacles
        v = self.kp_linear * distance_to_goal
        v = min(v, self.max_linear_vel)
        
        # Reduce speed when turning sharply
        if abs(heading_error) > np.pi/4:  # More than 45 degrees
            v *= 0.6
        
        # Reduce speed when obstacle is close
        if min_lidar < self.safety_distance:
            v *= max(0.3, min_lidar / self.safety_distance)
        
        # Very close to obstacle - slow down more
        if min_lidar < self.critical_distance:
            v *= 0.2
        
        return v, w, False

def main():
    """Main navigation test in environment."""
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Configure debug visualizer to show sensor panels
    # Enable RGB, depth, and segmentation views in GUI
    p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 1)
    
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
    
    # Create navigation controller (simple PID-based)
    controller = PIDNavigationController()
    
    print(f"\nNavigation Test in {env_type} environment")
    print(f"Goal: ({goal[0]}, {goal[1]})")
    print("Robot will navigate to the green sphere")
    print("\n📹 SENSOR PANELS:")
    print("   In the PyBullet GUI, you can view sensor data in the tabs:")
    print("   - RGB: Shows camera RGB view from robot's perspective")
    print("   - Depth: Shows depth map visualization")
    print("   - Segmentation: Shows object segmentation")
    print("   - Lidar: Visualized as colored rays in 3D view")
    print("\nPress Ctrl+C to stop\n")
    
    step = 0
    max_steps = 10000
    goal_reached = False
    collision_count = 0
    debug_items = []  # Track debug items for cleanup
    
    try:
        while step < max_steps and not goal_reached:
            # Get robot state
            try:
                pose = robot.get_pose()
                lidar = robot.get_lidar_scan()
                imu_data = robot.get_imu_data()
                odom_data = robot.get_odometry()
            except:
                # Robot may have been deleted, skip this iteration
                continue
            
            # Update camera view to robot's perspective for sensor panels
            # This feeds the RGB/Depth/Segmentation panels in the GUI
            try:
                pos, orn = p.getBasePositionAndOrientation(robot.robot_id)
                
                # Calculate camera position behind and above robot
                rot_matrix = p.getMatrixFromQuaternion(orn)
                rot_matrix = np.array(rot_matrix).reshape(3, 3)
                camera_offset = rot_matrix @ np.array([-0.3, 0, 0.2])  # Behind and above
                camera_pos = np.array(pos) + camera_offset
                camera_target = np.array(pos) + rot_matrix @ np.array([1.0, 0, 0.05])  # Look ahead
                
                # Get camera images for sensor panels (RGB, Depth, Segmentation)
                # Only update every few frames to improve performance
                if step % 3 == 0:  # Update camera every 3 frames (reduce computation)
                    view_matrix = p.computeViewMatrix(
                        cameraEyePosition=camera_pos.tolist(),
                        cameraTargetPosition=camera_target.tolist(),
                        cameraUpVector=[0, 0, 1]
                    )
                    proj_matrix = p.computeProjectionMatrixFOV(
                        fov=60,
                        aspect=1.0,
                        nearVal=0.01,
                        farVal=10.0
                    )
                    
                    # Get camera image - this feeds the RGB/Depth/Segmentation panels
                    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
                        width=320,
                        height=240,
                        viewMatrix=view_matrix,
                        projectionMatrix=proj_matrix,
                        renderer=p.ER_BULLET_HARDWARE_OPENGL
                    )
            except:
                pass  # Skip camera update if robot not available
            
            # Optional: Draw lidar visualization in 3D view
            # Only draw every few frames to improve performance
            if step % 2 == 0:  # Update every 2 frames
                # Clear previous debug items
                for item in debug_items:
                    try:
                        p.removeUserDebugItem(item)
                    except:
                        pass
                debug_items.clear()
                
                # Draw lidar rays (every 6th ray to reduce clutter and improve performance)
                num_rays = len(lidar)
                for i in range(0, num_rays, 6):  # Every 6th ray
                    dist = lidar[i]
                    angle = pose[2] + (2*np.pi * i / num_rays)
                    
                    start = [pose[0], pose[1], 0.11]
                    end = [
                        pose[0] + dist * np.cos(angle),
                        pose[1] + dist * np.sin(angle),
                        0.11
                    ]
                    
                    # Color based on distance (red=close, green=far)
                    intensity = min(1.0, dist / 5.0)
                    color = [1.0 - intensity, intensity, 0.0]
                    
                    # Draw individual line
                    line_id = p.addUserDebugLine(
                        start, end,
                        lineColorRGB=color,
                        lineWidth=1.5,
                        lifeTime=0.2
                    )
                    debug_items.append(line_id)
            
            # Check for collision - simple recovery
            try:
                collision_detected = robot.check_collision()
                if collision_detected:
                    collision_count += 1
                    # If stuck in collision, back up and turn
                    if collision_count > 3:
                        print(f"Collision detected at step {step}, backing up...")
                        # Back up
                        robot.set_velocity(-0.15, 0)
                        for _ in range(15):  # Back up for a bit
                            p.stepSimulation()
                        # Turn away based on lidar
                        left_space = np.mean(lidar[:len(lidar)//2])
                        right_space = np.mean(lidar[len(lidar)//2:])
                        turn_vel = -0.6 if left_space > right_space else 0.6
                        robot.set_velocity(0, turn_vel)
                        for _ in range(20):  # Turn for a bit
                            p.stepSimulation()
                        collision_count = 0
                        continue  # Skip this step's control
                else:
                    collision_count = 0  # Reset counter if no collision
            except:
                # If robot was deleted, skip this iteration
                collision_count = 0
                continue
            
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
