#!/usr/bin/env python3
"""Sensor data logger for recording robot sensor streams."""
import pybullet as p
import pybullet_data
import numpy as np
import json
import os
import sys
import time
from datetime import datetime

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scripts.diff_drive_robot import DiffDriveRobot

class SensorDataLogger:
    """Log sensor data to files for later analysis."""
    
    def __init__(self, log_dir="logs/sensor_data"):
        """Initialize logger with output directory."""
        self.log_dir = log_dir
        os.makedirs(log_dir, exist_ok=True)
        
        # Create timestamped log file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(log_dir, f"sensor_log_{timestamp}.json")
        
        # Data storage
        self.data = {
            'metadata': {
                'timestamp': timestamp,
                'description': 'Robot sensor data log'
            },
            'samples': []
        }
        
        self.sample_count = 0
    
    def log_sample(self, timestamp, pose, lidar_scan, imu_data, odom_data, 
                   rgb_image=None, depth_image=None, action=None):
        """
        Log a single sensor sample.
        
        Args:
            timestamp: Time since start (seconds)
            pose: Robot pose [x, y, theta]
            lidar_scan: Lidar distance array
            imu_data: IMU data dict
            odom_data: Odometry data dict
            rgb_image: Optional RGB image (will be saved separately)
            depth_image: Optional depth image (will be saved separately)
            action: Optional action taken [v, w]
        """
        sample = {
            'timestamp': float(timestamp),
            'pose': {
                'x': float(pose[0]),
                'y': float(pose[1]),
                'theta': float(pose[2])
            },
            'lidar': {
                'distances': lidar_scan.tolist(),
                'num_rays': len(lidar_scan),
                'range': float(np.max(lidar_scan))
            },
            'imu': {
                'accel': {
                    'x': float(imu_data['accel'][0]),
                    'y': float(imu_data['accel'][1]),
                    'z': float(imu_data['accel'][2])
                },
                'gyro': {
                    'x': float(imu_data['gyro'][0]),
                    'y': float(imu_data['gyro'][1]),
                    'z': float(imu_data['gyro'][2])
                },
                'orientation': {
                    'roll': float(imu_data['orientation'][0]),
                    'pitch': float(imu_data['orientation'][1]),
                    'yaw': float(imu_data['orientation'][2])
                }
            },
            'odometry': {
                'left_wheel_vel': float(odom_data['left_wheel_vel']),
                'right_wheel_vel': float(odom_data['right_wheel_vel']),
                'linear_velocity': {
                    'vx': float(odom_data['velocity'][0]),
                    'vy': float(odom_data['velocity'][1]),
                    'vz': float(odom_data['velocity'][2])
                },
                'angular_velocity': float(odom_data['velocity'][2])
            }
        }
        
        if action is not None:
            sample['action'] = {
                'linear_vel': float(action[0]),
                'angular_vel': float(action[1])
            }
        
        # Save images separately if provided
        if rgb_image is not None:
            img_path = os.path.join(self.log_dir, f"rgb_{self.sample_count:06d}.png")
            import cv2
            cv2.imwrite(img_path, cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR))
            sample['rgb_image'] = os.path.basename(img_path)
        
        if depth_image is not None:
            img_path = os.path.join(self.log_dir, f"depth_{self.sample_count:06d}.png")
            import cv2
            # Normalize depth for saving
            depth_normalized = (depth_image / depth_image.max() * 255).astype(np.uint8)
            cv2.imwrite(img_path, depth_normalized)
            sample['depth_image'] = os.path.basename(img_path)
        
        self.data['samples'].append(sample)
        self.sample_count += 1
    
    def save(self):
        """Save logged data to JSON file."""
        with open(self.log_file, 'w') as f:
            json.dump(self.data, f, indent=2)
        print(f"Saved {self.sample_count} samples to {self.log_file}")
        return self.log_file
    
    def get_summary(self):
        """Get summary statistics of logged data."""
        if len(self.data['samples']) == 0:
            return "No samples logged"
        
        poses = np.array([[s['pose']['x'], s['pose']['y']] for s in self.data['samples']])
        timestamps = [s['timestamp'] for s in self.data['samples']]
        
        summary = {
            'total_samples': len(self.data['samples']),
            'duration': max(timestamps) - min(timestamps) if len(timestamps) > 1 else 0,
            'distance_traveled': float(np.sum(np.linalg.norm(np.diff(poses, axis=0), axis=1))),
            'start_position': {'x': float(poses[0, 0]), 'y': float(poses[0, 1])},
            'end_position': {'x': float(poses[-1, 0]), 'y': float(poses[-1, 1])}
        }
        
        return summary

def main():
    """Example usage of sensor data logger."""
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Load environment
    plane_id = p.loadURDF("plane.urdf")
    
    # Create robot
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    # Create logger
    logger = SensorDataLogger()
    
    print("Recording sensor data...")
    print("Press Ctrl+C to stop and save")
    
    start_time = time.time()
    step = 0
    log_interval = 5  # Log every N simulation steps
    
    try:
        while step < 2000:  # Record for ~2000 steps
            # Get sensor data
            pose = robot.get_pose()
            lidar_scan = robot.get_lidar_scan()
            imu_data = robot.get_imu_data()
            odom_data = robot.get_odometry()
            
            # Simple motion
            if step < 500:
                v, w = 0.3, 0.0
            elif step < 1000:
                v, w = 0.2, -1.0
            else:
                v, w = 0.2, 1.0
            
            robot.set_velocity(v, w)
            
            # Step simulation
            for _ in range(10):
                p.stepSimulation()
            
            # Log data periodically
            if step % log_interval == 0:
                current_time = time.time() - start_time
                logger.log_sample(
                    timestamp=current_time,
                    pose=pose,
                    lidar_scan=lidar_scan,
                    imu_data=imu_data,
                    odom_data=odom_data,
                    action=[v, w]
                )
            
            step += 1
            
            if step % 200 == 0:
                print(f"Step {step}: Logged {logger.sample_count} samples")
            
            time.sleep(1/60)
    
    except KeyboardInterrupt:
        print("\nStopping recording...")
    
    # Save logged data
    log_file = logger.save()
    summary = logger.get_summary()
    
    print("\n" + "="*50)
    print("Recording Summary:")
    print("="*50)
    print(f"Total samples: {summary['total_samples']}")
    print(f"Duration: {summary['duration']:.2f} seconds")
    print(f"Distance traveled: {summary['distance_traveled']:.2f} m")
    print(f"Start position: ({summary['start_position']['x']:.2f}, {summary['start_position']['y']:.2f})")
    print(f"End position: ({summary['end_position']['x']:.2f}, {summary['end_position']['y']:.2f})")
    print(f"\nData saved to: {log_file}")
    
    p.disconnect()

if __name__ == "__main__":
    main()

