#!/usr/bin/env python3
"""Gymnasium environment for PyBullet differential drive navigation."""
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import pybullet as p
import pybullet_data
import os
import importlib.util
from typing import Dict, Tuple, Optional

# Dynamic import for modules with numeric names
def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

# Load required modules
env_builder_module = load_module("env_builder", 
    os.path.join(os.path.dirname(__file__), "05_environment_builder.py"))
EnvironmentBuilder = env_builder_module.EnvironmentBuilder

from scripts.diff_drive_robot import DiffDriveRobot


class PyBulletNavEnv(gym.Env):
    """Gymnasium environment for PyBullet differential drive navigation."""
    
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}
    
    def __init__(self, render_mode=None, env_type="warehouse", num_obstacles=None):
        """
        Initialize navigation environment.
        
        Args:
            render_mode: "human" (GUI) or None (headless)
            env_type: "warehouse", "street", or "maze"
            num_obstacles: Number of obstacles (None = default for env_type)
        """
        super().__init__()
        
        self.render_mode = render_mode
        self.env_type = env_type
        self.num_obstacles = num_obstacles
        
        # PyBullet setup
        if render_mode == "human":
            self.physics_client = p.connect(p.GUI)
        else:
            self.physics_client = p.connect(p.DIRECT)
        
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1/240)
        
        # Action space: [v, w] - linear and angular velocity
        self.action_space = spaces.Box(
            low=np.array([-0.5, -2.0], dtype=np.float32),
            high=np.array([0.7, 2.0], dtype=np.float32),
            dtype=np.float32
        )
        
        # Observation space:
        # [dx, dy, dtheta, v, w, lidar_readings (36 values), min_lidar]
        obs_dim = 5 + 36 + 1  # relative goal + velocities + lidar + min_lidar
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(obs_dim,),
            dtype=np.float32
        )
        
        # Environment parameters
        self.arena_size = 10.0
        self.goal_threshold = 0.4
        self.max_steps = 1500  # Increased from 1000 to give more time
        self.time_step = 1/24  # Control rate
        
        # State
        self.robot = None
        self.goal = None
        self.obstacles = []
        self.builder = None
        self.step_count = 0
        self.prev_distance = None
        self.start_pos = None
        
    def _create_environment(self):
        """Create or reset the simulation environment."""
        # Clear existing objects
        p.resetSimulation()
        p.setGravity(0, 0, -9.81)
        
        # Load ground
        p.loadURDF("plane.urdf")
        
        # Create environment
        if self.builder is None:
            self.builder = EnvironmentBuilder(self.physics_client)
        
        # Create obstacles based on environment type
        if self.env_type == "warehouse":
            self.obstacles = self.builder.create_warehouse_environment()
        elif self.env_type == "street":
            self.obstacles = self.builder.create_street_environment()
        elif self.env_type == "maze":
            self.obstacles = self.builder.create_simple_maze()
        else:
            # Default: simple obstacles
            self.obstacles = self.builder.create_warehouse_environment()
        
        # Random goal position (far from start)
        while True:
            gx = np.random.uniform(-self.arena_size/2 + 1, self.arena_size/2 - 1)
            gy = np.random.uniform(-self.arena_size/2 + 1, self.arena_size/2 - 1)
            if np.sqrt(gx**2 + gy**2) > 2.0:  # Far from start
                break
        self.goal = np.array([gx, gy])
        
        # Create robot at origin
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                                 "urdf", "diff_drive_robot.urdf")
        start_yaw = np.random.uniform(-np.pi, np.pi)
        self.start_pos = [0, 0, 0.1]
        self.robot = DiffDriveRobot(urdf_path, start_pos=self.start_pos, start_yaw=start_yaw)
        
        # Goal marker (for visualization)
        if self.render_mode == "human":
            goal_visual = p.createVisualShape(
                p.GEOM_SPHERE,
                radius=0.3,
                rgbaColor=[0.2, 0.8, 0.2, 0.8]
            )
            self.goal_marker = p.createMultiBody(
                baseMass=0,
                baseVisualShapeIndex=goal_visual,
                basePosition=[gx, gy, 0.3]
            )
    
    def _get_obs(self) -> np.ndarray:
        """Compute observation."""
        pose = self.robot.get_pose()
        
        # Relative goal position
        dx = self.goal[0] - pose[0]
        dy = self.goal[1] - pose[1]
        distance_to_goal = np.sqrt(dx**2 + dy**2)
        
        # Heading to goal
        desired_heading = np.arctan2(dy, dx)
        dtheta = desired_heading - pose[2]
        dtheta = np.arctan2(np.sin(dtheta), np.cos(dtheta))
        
        # Normalize relative goal (scale by arena size)
        dx_norm = dx / self.arena_size
        dy_norm = dy / self.arena_size
        dtheta_norm = dtheta / np.pi  # Normalize to [-1, 1]
        
        # Velocities
        v = self.robot.last_v
        w = self.robot.last_w
        
        # Normalize velocities
        v_norm = v / 0.7  # Max linear velocity
        w_norm = w / 2.0  # Max angular velocity
        
        # Lidar scan
        lidar = self.robot.get_lidar_scan()
        min_lidar = np.min(lidar)
        
        # Normalize lidar (by max range 5.0m)
        lidar_norm = lidar / 5.0
        min_lidar_norm = min_lidar / 5.0
        
        # Combine observation
        obs = np.concatenate([
            [dx_norm, dy_norm, dtheta_norm, v_norm, w_norm],
            lidar_norm,
            [min_lidar_norm]
        ]).astype(np.float32)
        
        return obs
    
    def _get_info(self) -> Dict:
        """Additional info for debugging."""
        pose = self.robot.get_pose()
        dist = np.sqrt((self.goal[0]-pose[0])**2 + (self.goal[1]-pose[1])**2)
        return {
            "distance_to_goal": dist,
            "steps": self.step_count,
            "collision": self.robot.check_collision() if self.robot else False
        }
    
    def reset(self, seed=None, options=None) -> Tuple[np.ndarray, Dict]:
        """Reset environment."""
        super().reset(seed=seed)
        
        self._create_environment()
        self.step_count = 0
        
        # Initial distance for reward shaping
        pose = self.robot.get_pose()
        self.prev_distance = np.sqrt(
            (self.goal[0]-pose[0])**2 + (self.goal[1]-pose[1])**2
        )
        
        obs = self._get_obs()
        info = self._get_info()
        
        return obs, info
    
    def step(self, action: np.ndarray) -> Tuple[np.ndarray, float, bool, bool, Dict]:
        """Execute action."""
        # Apply action (clamp to action space)
        v, w = action[0], action[1]
        v = np.clip(v, -0.5, 0.7)
        w = np.clip(w, -2.0, 2.0)
        
        self.robot.set_velocity(v, w)
        
        # Step physics
        num_physics_steps = int(self.time_step / (1/240))
        for _ in range(num_physics_steps):
            p.stepSimulation()
        
        self.step_count += 1
        
        # Get state
        obs = self._get_obs()
        info = self._get_info()
        
        # Compute reward
        current_distance = info["distance_to_goal"]
        progress = self.prev_distance - current_distance
        self.prev_distance = current_distance
        
        reward = 0.0
        
        # Shaped reward: distance-based (encourages getting closer)
        # Larger reward when closer to goal
        distance_reward = 1.0 / (1.0 + current_distance)  # 1.0 at goal, ~0.1 at 9m
        reward += 5.0 * distance_reward
        
        # Progress reward (encourage moving toward goal) - increased weight
        reward += 20.0 * progress  # Increased from 10.0
        
        # Bonus for making progress (even small)
        if progress > 0.01:  # Moved at least 1cm closer
            reward += 1.0
        
        # Goal reached reward (large bonus)
        if current_distance < self.goal_threshold:
            reward += 200.0  # Increased from 100.0
        
        # Time penalty (reduced - less harsh)
        reward -= 0.01  # Reduced from 0.1 (10x less harsh)
        
        # Collision penalty
        if info["collision"]:
            reward -= 30.0  # Reduced from 50.0
        
        # Obstacle penalty (based on lidar) - only for very close
        min_lidar = np.min(obs[5:41])  # Lidar readings in observation
        if min_lidar < 0.2:  # Very close to obstacle (reduced threshold)
            reward -= 3.0 * (0.2 - min_lidar)  # Reduced penalty
        
        # Penalty for getting stuck (not making progress for a while)
        # This encourages exploration and escaping local minima
        if abs(progress) < 0.01 and current_distance > 0.5:  # Stuck and not at goal
            reward -= 0.5
        
        # Termination conditions (ensure Python bool, not numpy bool)
        terminated = bool(current_distance < self.goal_threshold)
        truncated = bool(self.step_count >= self.max_steps)
        
        # Truncate on collision (severe collision)
        if info["collision"]:
            truncated = True
        
        return obs, reward, terminated, truncated, info
    
    def render(self):
        """Render (handled by PyBullet GUI)."""
        if self.render_mode == "human":
            # Camera follows robot
            try:
                pose = self.robot.get_pose()
                p.resetDebugVisualizerCamera(
                    cameraDistance=8,
                    cameraYaw=np.degrees(pose[2]) + 45,
                    cameraPitch=-45,
                    cameraTargetPosition=[pose[0], pose[1], 0]
                )
            except:
                pass
    
    def close(self):
        """Cleanup."""
        if self.physics_client is not None:
            p.disconnect(self.physics_client)
            self.physics_client = None

