#!/usr/bin/env python3
"""Real-time sensor stream visualization for PyBullet robot."""
import pybullet as p
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle
import cv2
import os
import sys
import time

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot

class SensorStreamVisualizer:
    """Real-time visualization of robot sensor streams."""
    
    def __init__(self, robot):
        self.robot = robot
        self.fig = None
        self.axes = None
        self.setup_plots()
        
        # Data storage for plotting
        self.lidar_history = []
        self.pose_history = []
        self.time_history = []
        self.start_time = time.time()
    
    def setup_plots(self):
        """Setup matplotlib figure with subplots for different sensors."""
        self.fig = plt.figure(figsize=(16, 10))
        # Make window non-blocking and allow it to be moved to background
        self.fig.canvas.manager.set_window_title('Sensor Streams - Can be minimized')
        gs = self.fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)
        
        # Lidar visualization (top left, large)
        self.ax_lidar = self.fig.add_subplot(gs[0:2, 0:2])
        self.ax_lidar.set_xlim(-6, 6)
        self.ax_lidar.set_ylim(-6, 6)
        self.ax_lidar.set_aspect('equal')
        self.ax_lidar.set_title('Lidar Scan (2D Top View)', fontsize=12, fontweight='bold')
        self.ax_lidar.grid(True, alpha=0.3)
        self.ax_lidar.set_xlabel('X (m)')
        self.ax_lidar.set_ylabel('Y (m)')
        
        # Camera RGB (top right)
        self.ax_rgb = self.fig.add_subplot(gs[0, 2])
        self.ax_rgb.set_title('RGB Camera', fontsize=10, fontweight='bold')
        self.ax_rgb.axis('off')
        self.img_rgb = None
        
        # Camera Depth (middle right)
        self.ax_depth = self.fig.add_subplot(gs[1, 2])
        self.ax_depth.set_title('Depth Camera', fontsize=10, fontweight='bold')
        self.ax_depth.axis('off')
        self.img_depth = None
        
        # IMU data (bottom left)
        self.ax_imu = self.fig.add_subplot(gs[2, 0])
        self.ax_imu.set_title('IMU Data', fontsize=10, fontweight='bold')
        self.ax_imu.set_ylabel('Value')
        self.ax_imu.grid(True, alpha=0.3)
        
        # Odometry (bottom middle)
        self.ax_odom = self.fig.add_subplot(gs[2, 1])
        self.ax_odom.set_title('Odometry', fontsize=10, fontweight='bold')
        self.ax_odom.set_ylabel('Value')
        self.ax_odom.grid(True, alpha=0.3)
        
        # Trajectory (bottom right)
        self.ax_traj = self.fig.add_subplot(gs[2, 2])
        self.ax_traj.set_title('Robot Trajectory', fontsize=10, fontweight='bold')
        self.ax_traj.set_xlabel('X (m)')
        self.ax_traj.set_ylabel('Y (m)')
        self.ax_traj.set_aspect('equal')
        self.ax_traj.grid(True, alpha=0.3)
        
        plt.tight_layout()
    
    def update_lidar_plot(self, lidar_scan, pose):
        """Update lidar visualization."""
        self.ax_lidar.clear()
        self.ax_lidar.set_xlim(-6, 6)
        self.ax_lidar.set_ylim(-6, 6)
        self.ax_lidar.set_aspect('equal')
        self.ax_lidar.set_title('Lidar Scan (2D Top View)', fontsize=12, fontweight='bold')
        self.ax_lidar.grid(True, alpha=0.3)
        self.ax_lidar.set_xlabel('X (m)')
        self.ax_lidar.set_ylabel('Y (m)')
        
        # Draw robot position
        robot_circle = Circle((pose[0], pose[1]), 0.15, color='blue', alpha=0.7)
        self.ax_lidar.add_patch(robot_circle)
        
        # Draw robot heading
        heading_length = 0.3
        dx = heading_length * np.cos(pose[2])
        dy = heading_length * np.sin(pose[2])
        self.ax_lidar.arrow(pose[0], pose[1], dx, dy, 
                           head_width=0.1, head_length=0.1, fc='blue', ec='blue')
        
        # Draw lidar rays
        num_rays = len(lidar_scan)
        for i, dist in enumerate(lidar_scan):
            angle = pose[2] + (2*np.pi * i / num_rays)
            
            # Ray endpoint
            end_x = pose[0] + dist * np.cos(angle)
            end_y = pose[1] + dist * np.sin(angle)
            
            # Color based on distance (closer = red, farther = green)
            color_intensity = min(1.0, dist / 5.0)
            color = (1.0 - color_intensity, color_intensity, 0.0)
            
            self.ax_lidar.plot([pose[0], end_x], [pose[1], end_y], 
                             color=color, linewidth=1.5, alpha=0.6)
            
            # Draw hit point
            if dist < 5.0:  # Only draw if not max range
                self.ax_lidar.plot(end_x, end_y, 'ro', markersize=3, alpha=0.7)
    
    def update_camera_plots(self, rgb_image, depth_image):
        """Update camera visualizations."""
        # RGB image
        self.ax_rgb.clear()
        self.ax_rgb.axis('off')
        self.ax_rgb.set_title('RGB Camera', fontsize=10, fontweight='bold')
        if rgb_image is not None:
            self.ax_rgb.imshow(rgb_image)
        
        # Depth image
        self.ax_depth.clear()
        self.ax_depth.axis('off')
        self.ax_depth.set_title('Depth Camera', fontsize=10, fontweight='bold')
        if depth_image is not None:
            # Normalize depth for visualization
            depth_vis = depth_image.copy()
            depth_vis[depth_vis > 10.0] = 10.0
            depth_vis = depth_vis / 10.0
            self.ax_depth.imshow(depth_vis, cmap='jet')
    
    def update_imu_plot(self, imu_data):
        """Update IMU data visualization."""
        self.ax_imu.clear()
        self.ax_imu.set_title('IMU Data', fontsize=10, fontweight='bold')
        self.ax_imu.set_ylabel('Value')
        self.ax_imu.grid(True, alpha=0.3)
        
        labels = ['Accel X', 'Accel Y', 'Accel Z', 'Gyro X', 'Gyro Y', 'Gyro Z']
        values = np.concatenate([imu_data['accel'], imu_data['gyro']])
        colors = ['red', 'green', 'blue', 'orange', 'purple', 'brown']
        
        bars = self.ax_imu.bar(labels, values, color=colors, alpha=0.7)
        self.ax_imu.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        self.ax_imu.set_ylim(-15, 15)
        
        # Add value labels on bars
        for bar, val in zip(bars, values):
            height = bar.get_height()
            self.ax_imu.text(bar.get_x() + bar.get_width()/2., height,
                           f'{val:.2f}', ha='center', va='bottom' if height >= 0 else 'top',
                           fontsize=8)
    
    def update_odometry_plot(self, odom_data):
        """Update odometry visualization."""
        self.ax_odom.clear()
        self.ax_odom.set_title('Odometry', fontsize=10, fontweight='bold')
        self.ax_odom.set_ylabel('Value')
        self.ax_odom.grid(True, alpha=0.3)
        
        labels = ['Left Wheel\nVel (rad/s)', 'Right Wheel\nVel (rad/s)', 
                 'Linear Vel\n(m/s)', 'Angular Vel\n(rad/s)']
        values = [
            odom_data['left_wheel_vel'],
            odom_data['right_wheel_vel'],
            odom_data['velocity'][0],  # vx
            odom_data['velocity'][2]   # w (angular)
        ]
        colors = ['blue', 'red', 'green', 'orange']
        
        bars = self.ax_odom.bar(labels, values, color=colors, alpha=0.7)
        self.ax_odom.axhline(y=0, color='black', linestyle='-', linewidth=0.5)
        
        # Add value labels
        for bar, val in zip(bars, values):
            height = bar.get_height()
            self.ax_odom.text(bar.get_x() + bar.get_width()/2., height,
                            f'{val:.2f}', ha='center', va='bottom' if height >= 0 else 'top',
                            fontsize=8)
    
    def update_trajectory_plot(self, pose):
        """Update robot trajectory visualization."""
        self.pose_history.append(pose.copy())
        
        if len(self.pose_history) > 1:
            poses = np.array(self.pose_history)
            self.ax_traj.clear()
            self.ax_traj.set_title('Robot Trajectory', fontsize=10, fontweight='bold')
            self.ax_traj.set_xlabel('X (m)')
            self.ax_traj.set_ylabel('Y (m)')
            self.ax_traj.set_aspect('equal')
            self.ax_traj.grid(True, alpha=0.3)
            
            # Plot trajectory
            self.ax_traj.plot(poses[:, 0], poses[:, 1], 'b-', linewidth=2, alpha=0.7, label='Path')
            
            # Plot start
            if len(poses) > 0:
                self.ax_traj.plot(poses[0, 0], poses[0, 1], 'go', markersize=10, label='Start')
            
            # Plot current position
            self.ax_traj.plot(pose[0], pose[1], 'ro', markersize=10, label='Current')
            
            # Draw heading
            heading_length = 0.2
            dx = heading_length * np.cos(pose[2])
            dy = heading_length * np.sin(pose[2])
            self.ax_traj.arrow(pose[0], pose[1], dx, dy,
                              head_width=0.1, head_length=0.1, fc='red', ec='red')
            
            self.ax_traj.legend(loc='upper right', fontsize=8)
    
    def update(self, lidar_scan, pose, rgb_image, depth_image, imu_data, odom_data):
        """Update all visualizations."""
        self.update_lidar_plot(lidar_scan, pose)
        self.update_camera_plots(rgb_image, depth_image)
        self.update_imu_plot(imu_data)
        self.update_odometry_plot(odom_data)
        self.update_trajectory_plot(pose)
        
        # Use non-blocking draw to allow PyBullet window to stay on top
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.01)

def main():
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Camera setup for PyBullet GUI
    p.resetDebugVisualizerCamera(
        cameraDistance=8,
        cameraYaw=45,
        cameraPitch=-45,
        cameraTargetPosition=[0, 0, 0]
    )
    
    # Load environment
    plane_id = p.loadURDF("plane.urdf")
    
    # Create some obstacles for visualization
    for i in range(3):
        x = np.random.uniform(-3, 3)
        y = np.random.uniform(-3, 3)
        if np.sqrt(x**2 + y**2) > 1.0:
            visual = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=0.2,
                length=0.5,
                rgbaColor=[0.8, 0.2, 0.2, 1]
            )
            collision = p.createCollisionShape(
                p.GEOM_CYLINDER,
                radius=0.2,
                height=0.5
            )
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=collision,
                baseVisualShapeIndex=visual,
                basePosition=[x, y, 0.25]
            )
    
    # Create robot
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    # Create visualizer
    visualizer = SensorStreamVisualizer(robot)
    
    print("Sensor stream visualization started!")
    print("Close the matplotlib window or press Ctrl+C to stop")
    
    step = 0
    try:
        while True:
            # Get sensor data
            pose = robot.get_pose()
            lidar_scan = robot.get_lidar_scan()
            rgb_image, depth_image = robot.get_camera_image()
            imu_data = robot.get_imu_data()
            odom_data = robot.get_odometry()
            
            # Simple motion for demonstration
            if step < 500:
                v, w = 0.3, 0.0
            elif step < 1000:
                v, w = 0.2, -1.0
            elif step < 1500:
                v, w = 0.2, 1.0
            else:
                v, w = 0.3, 0.0
            
            robot.set_velocity(v, w)
            
            # Step simulation
            for _ in range(10):
                p.stepSimulation()
            
            # Update visualizations
            visualizer.update(lidar_scan, pose, rgb_image, depth_image, imu_data, odom_data)
            
            step += 1
            
            if step % 100 == 0:
                print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                      f"min_lidar={np.min(lidar_scan):.2f}m")
            
            time.sleep(1/30)  # ~30 FPS for visualization
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    
    plt.close('all')
    p.disconnect()

if __name__ == "__main__":
    main()

