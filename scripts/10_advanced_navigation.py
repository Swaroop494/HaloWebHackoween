#!/usr/bin/env python3
"""Advanced navigation with path planning (A*, RRT) and multi-goal navigation."""
import pybullet as p
import pybullet_data
import numpy as np
import os
import sys
import time
import importlib.util
from typing import List, Tuple, Optional

# Dynamic import for module with numeric name
spec = importlib.util.spec_from_file_location("env_builder", 
    os.path.join(os.path.dirname(__file__), "05_environment_builder.py"))
env_builder_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(env_builder_module)
EnvironmentBuilder = env_builder_module.EnvironmentBuilder

from scripts.diff_drive_robot import DiffDriveRobot
from scripts.path_planner import AStarPlanner, RRTPlanner, PathFollower
from scripts.localization_tracker import LocalizationTracker, PerformanceLogger


class DynamicObstacleAvoidance:
    """Dynamic obstacle avoidance with path replanning."""
    
    def __init__(self, planner_type='astar', replan_threshold=0.4):
        """
        Initialize dynamic obstacle avoidance.
        
        Args:
            planner_type: 'astar' or 'rrt'
            replan_threshold: Distance to obstacle that triggers replanning
        """
        self.planner_type = planner_type
        self.replan_threshold = replan_threshold
        
        if planner_type == 'astar':
            self.planner = AStarPlanner(grid_size=0.2, obstacle_threshold=0.5)
        else:
            self.planner = RRTPlanner(step_size=0.5, max_iterations=3000)
        
        self.path_follower = PathFollower(lookahead_distance=0.5, waypoint_threshold=0.3)
        self.current_path = None
        self.last_replan_time = 0
        self.replan_interval = 1.0  # Minimum time between replans (seconds)
    
    def check_obstacle_ahead(self, robot_pose: np.ndarray, lidar_scan: np.ndarray) -> bool:
        """Check if obstacle is ahead on current path."""
        min_lidar = np.min(lidar_scan)
        return min_lidar < self.replan_threshold
    
    def create_obstacle_checker(self, robot_pose: np.ndarray, lidar_scan: np.ndarray):
        """Create obstacle checker function from lidar scan."""
        num_rays = len(lidar_scan)
        lidar_data = lidar_scan.copy()
        robot_pos = robot_pose[:2]
        robot_theta = robot_pose[2]
        
        def obstacle_checker(x: float, y: float) -> bool:
            """Check if point (x, y) is in obstacle."""
            # Convert to robot frame
            dx = x - robot_pos[0]
            dy = y - robot_pos[1]
            
            # Rotate to robot frame
            cos_t = np.cos(-robot_theta)
            sin_t = np.sin(-robot_theta)
            local_x = dx * cos_t - dy * sin_t
            local_y = dx * sin_t + dy * cos_t
            
            # Check if point is within lidar range
            dist = np.sqrt(local_x**2 + local_y**2)
            if dist > 5.0:  # Lidar range
                return False
            
            # Get angle
            angle = np.arctan2(local_y, local_x)
            # Normalize to [0, 2π]
            angle = (angle + np.pi) % (2 * np.pi)
            
            # Find corresponding lidar ray
            ray_idx = int((angle / (2 * np.pi)) * num_rays) % num_rays
            lidar_dist = lidar_data[ray_idx]
            
            # Check if point is closer than lidar reading (obstacle)
            return dist < lidar_dist * 0.9  # 10% margin
        
        return obstacle_checker
    
    def plan_path(self, start: Tuple[float, float], goal: Tuple[float, float],
                  robot_pose: np.ndarray, lidar_scan: np.ndarray, 
                  perf_logger=None) -> Optional[List[Tuple[float, float]]]:
        """
        Plan path from start to goal.
        
        Args:
            start: (x, y) start position
            goal: (x, y) goal position
            robot_pose: Current robot pose
            lidar_scan: Current lidar scan
            perf_logger: Performance logger (optional)
            
        Returns:
            List of waypoints, or None if no path found
        """
        if perf_logger:
            timing_context = perf_logger.time_operation('path_planning')
            timing_context.__enter__()
        
        try:
            if self.planner_type == 'astar':
                # Create grid from lidar
                grid, origin = self.planner.create_grid_from_lidar(robot_pose, lidar_scan)
                path = self.planner.plan(start, goal, grid, origin)
            else:
                # RRT planning
                bounds = ((-10, 10), (-10, 10))  # Workspace bounds
                obstacle_checker = self.create_obstacle_checker(robot_pose, lidar_scan)
                path = self.planner.plan(start, goal, bounds, obstacle_checker)
        finally:
            if perf_logger:
                timing_context.__exit__(None, None, None)
        
        return path
    
    def update(self, robot_pose: np.ndarray, goal: Tuple[float, float],
               lidar_scan: np.ndarray, current_time: float, perf_logger=None) -> Tuple[float, float, bool]:
        """
        Update navigation with dynamic replanning.
        
        Args:
            robot_pose: [x, y, theta] current robot pose
            goal: (x, y) goal position
            lidar_scan: Current lidar scan
            current_time: Current simulation time
            
        Returns:
            (v, w, goal_reached) velocity commands
        """
        start = (robot_pose[0], robot_pose[1])
        
        # Check if goal reached
        dist_to_goal = np.sqrt((goal[0] - robot_pose[0])**2 + (goal[1] - robot_pose[1])**2)
        if dist_to_goal < 0.4:
            return 0.0, 0.0, True
        
        # Check if replanning needed
        needs_replan = False
        
        if self.current_path is None:
            needs_replan = True
        elif self.check_obstacle_ahead(robot_pose, lidar_scan):
            # Obstacle detected ahead
            if current_time - self.last_replan_time > self.replan_interval:
                needs_replan = True
        
        # Replan if needed
        if needs_replan:
            path = self.plan_path(start, goal, robot_pose, lidar_scan, perf_logger=perf_logger)
            if path is not None:
                self.current_path = path
                self.path_follower.set_path(path)
                self.last_replan_time = current_time
            elif self.current_path is None:
                # No path found, use simple obstacle avoidance
                return self.simple_avoidance(robot_pose, goal, lidar_scan)
        
        # Follow path
        if self.current_path is not None:
            v, w, path_complete = self.path_follower.compute_control(robot_pose)
            if path_complete:
                # Path complete, check if at goal
                if dist_to_goal < 0.4:
                    return 0.0, 0.0, True
                else:
                    # Replan to goal
                    path = self.plan_path(start, goal, robot_pose, lidar_scan, perf_logger=perf_logger)
                    if path is not None:
                        self.current_path = path
                        self.path_follower.set_path(path)
            return v, w, False
        
        # Fallback: simple avoidance
        return self.simple_avoidance(robot_pose, goal, lidar_scan)
    
    def simple_avoidance(self, robot_pose: np.ndarray, goal: Tuple[float, float],
                       lidar_scan: np.ndarray) -> Tuple[float, float, bool]:
        """Simple obstacle avoidance fallback."""
        min_lidar = np.min(lidar_scan)
        num_rays = len(lidar_scan)
        
        # Find direction with most space
        left_space = np.mean(lidar_scan[:num_rays//2])
        right_space = np.mean(lidar_scan[num_rays//2:])
        
        # Calculate heading to goal
        dx = goal[0] - robot_pose[0]
        dy = goal[1] - robot_pose[1]
        desired_heading = np.arctan2(dy, dx)
        heading_error = desired_heading - robot_pose[2]
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        
        # Modify heading if obstacle close
        if min_lidar < 0.6:
            if left_space > right_space:
                heading_error += np.pi/3  # Turn left
            else:
                heading_error -= np.pi/3  # Turn right
        
        w = 2.0 * heading_error
        w = np.clip(w, -2.0, 2.0)
        
        v = 0.5  # Increased from 0.3
        if min_lidar < 0.5:
            v *= 0.6  # Less aggressive slowdown
        
        return v, w, False


class MultiGoalNavigator:
    """Navigate through multiple goals/waypoints."""
    
    def __init__(self, planner_type='astar'):
        """
        Initialize multi-goal navigator.
        
        Args:
            planner_type: 'astar' or 'rrt'
        """
        self.planner_type = planner_type
        self.avoidance = DynamicObstacleAvoidance(planner_type=planner_type)
        self.goals = []
        self.current_goal_idx = 0
        self.start_time = time.time()
    
    def set_goals(self, goals: List[Tuple[float, float]]):
        """Set list of goals to visit in order."""
        self.goals = goals
        self.current_goal_idx = 0
    
    def update(self, robot_pose: np.ndarray, lidar_scan: np.ndarray) -> Tuple[float, float, bool]:
        """
        Update navigation to current goal.
        
        Args:
            robot_pose: [x, y, theta] current robot pose
            lidar_scan: Current lidar scan
            
        Returns:
            (v, w, all_goals_reached)
        """
        if self.current_goal_idx >= len(self.goals):
            return 0.0, 0.0, True  # All goals reached
        
        current_goal = self.goals[self.current_goal_idx]
        current_time = time.time() - self.start_time
        
        v, w, goal_reached = self.avoidance.update(robot_pose, current_goal, lidar_scan, current_time)
        
        if goal_reached:
            self.current_goal_idx += 1
            # Clear path for next goal
            self.avoidance.current_path = None
            self.avoidance.path_follower.current_path = None
        
        all_goals_reached = (self.current_goal_idx >= len(self.goals))
        return v, w, all_goals_reached


def main():
    """Main advanced navigation test."""
    # Parse arguments
    planner_type = sys.argv[1] if len(sys.argv) > 1 else 'astar'
    multi_goal = '--multi' in sys.argv or '-m' in sys.argv
    
    if planner_type not in ['astar', 'rrt']:
        print(f"Unknown planner type: {planner_type}, using 'astar'")
        planner_type = 'astar'
    
    # Connect to PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1/240)
    
    # Configure debug visualizer
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
    env_type = 'warehouse'
    print(f"Creating {env_type} environment...")
    obstacles = builder.create_warehouse_environment()
    
    # Create robot
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                             "urdf", "diff_drive_robot.urdf")
    robot = DiffDriveRobot(urdf_path, start_pos=[0, 0, 0.1])
    
    # Set goals
    if multi_goal:
        goals = [(3, 3), (6, 3), (6, 6), (3, 6), (0, 0)]
        navigator = MultiGoalNavigator(planner_type=planner_type)
        navigator.set_goals(goals)
        print(f"\nMulti-goal navigation with {planner_type.upper()} planner")
        print(f"Goals: {goals}")
    else:
        goals = [(6, 6)]
        navigator = DynamicObstacleAvoidance(planner_type=planner_type)
        print(f"\nSingle goal navigation with {planner_type.upper()} planner")
        print(f"Goal: {goals[0]}")
    
    # Create goal markers
    goal_markers = []
    for i, goal in enumerate(goals):
        color = [0.2, 0.8, 0.2, 0.8] if i == 0 else [0.8, 0.2, 0.2, 0.8]
        goal_visual = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=0.3,
            rgbaColor=color
        )
        goal_marker = p.createMultiBody(
            baseMass=0,
            baseVisualShapeIndex=goal_visual,
            basePosition=[goal[0], goal[1], 0.3]
        )
        goal_markers.append(goal_marker)
    
    print("\nPress Ctrl+C to stop\n")
    
    # Initialize tracking and performance logging
    tracker = LocalizationTracker(save_trajectory=True)
    tracker.start()
    perf_logger = PerformanceLogger(log_interval=200)
    
    step = 0
    max_steps = 20000
    all_goals_reached = False
    debug_items = []
    path_lines = []
    
    try:
        while step < max_steps and not all_goals_reached:
            perf_logger.start_cycle()
            
            # Get robot state
            try:
                with perf_logger.time_operation('sensor_read'):
                    pose = robot.get_pose()
                    lidar = robot.get_lidar_scan()
                    odom_data = robot.get_odometry()
            except:
                continue
            
            # Update localization tracker
            velocity = np.array([odom_data['velocity'][0], odom_data['velocity'][1], odom_data['velocity'][2]])
            tracker.update(pose, velocity)
            
            # Update camera view to robot's perspective for sensor panels
            # This feeds the RGB/Depth/Segmentation panels in the GUI
            with perf_logger.time_operation('visualization'):
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
                    if step % 3 == 0:  # Update camera every 3 frames
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
                
                # Draw lidar visualization in 3D view
                if step % 2 == 0:  # Update every 2 frames
                    # Clear previous debug items (except path lines)
                    for item in debug_items:
                        try:
                            p.removeUserDebugItem(item)
                        except:
                            pass
                    debug_items.clear()
                    
                    # Draw lidar rays (every 6th ray to reduce clutter)
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
                        try:
                            line_id = p.addUserDebugLine(
                                start, end,
                                lineColorRGB=color,
                                lineWidth=1.5,
                                lifeTime=0.2
                            )
                            debug_items.append(line_id)
                        except:
                            pass
            
            # Update navigation
            with perf_logger.time_operation('navigation_update'):
                if multi_goal:
                    v, w, all_goals_reached = navigator.update(pose, lidar)
                else:
                    v, w, goal_reached = navigator.update(pose, goals[0], lidar, time.time(), perf_logger=perf_logger)
                    all_goals_reached = goal_reached
            
            if all_goals_reached:
                print(f"\n✓ All goals reached at step {step}!")
                # Print final statistics
                perf_logger.print_statistics()
                tracker.print_statistics()
                tracker.save_trajectory_to_file()
                break
            
            # Visualize path (only update periodically to reduce computation)
            if step % 5 == 0:  # Update path visualization every 5 frames
                if hasattr(navigator, 'avoidance'):
                    path = navigator.avoidance.current_path
                else:
                    path = navigator.current_path
                
                # Clear previous path visualization
                for line_id in path_lines:
                    try:
                        p.removeUserDebugItem(line_id)
                    except:
                        pass
                path_lines.clear()
                
                # Draw path
                if path and len(path) > 1:
                    for i in range(len(path) - 1):
                        start = [path[i][0], path[i][1], 0.15]
                        end = [path[i+1][0], path[i+1][1], 0.15]
                        try:
                            line_id = p.addUserDebugLine(
                                start, end,
                                lineColorRGB=[0, 0, 1],  # Blue path
                                lineWidth=3,
                                lifeTime=0.3
                            )
                            path_lines.append(line_id)
                        except:
                            pass  # Skip if disconnected
            
            # Apply control
            with perf_logger.time_operation('control_apply'):
                robot.set_velocity(v, w)
            
            # Step simulation
            with perf_logger.time_operation('physics_step'):
                for _ in range(10):
                    p.stepSimulation()
            
            perf_logger.end_cycle()
            step += 1
            
            # Print status with performance info
            if step % 200 == 0:
                stats = perf_logger.get_statistics()
                cycle_time = stats['total_cycle']['mean'] * 1000 if stats['total_cycle'] else 0
                fps = 1.0 / stats['total_cycle']['mean'] if stats['total_cycle'] and stats['total_cycle']['mean'] > 0 else 0
                
                if multi_goal:
                    current_goal = navigator.goals[navigator.current_goal_idx] if navigator.current_goal_idx < len(navigator.goals) else None
                    dist = np.sqrt((current_goal[0]-pose[0])**2 + (current_goal[1]-pose[1])**2) if current_goal else 0
                    print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                          f"goal {navigator.current_goal_idx+1}/{len(navigator.goals)}, "
                          f"dist={dist:.2f}m | "
                          f"Cycle: {cycle_time:.1f}ms ({fps:.1f} FPS)")
                else:
                    dist = np.sqrt((goals[0][0]-pose[0])**2 + (goals[1]-pose[1])**2)
                    print(f"Step {step}: pos=({pose[0]:.2f}, {pose[1]:.2f}), "
                          f"dist_to_goal={dist:.2f}m | "
                          f"Cycle: {cycle_time:.1f}ms ({fps:.1f} FPS)")
                
                # Print detailed performance stats periodically
                if perf_logger.should_log():
                    perf_logger.print_statistics()
                    tracker_stats = tracker.get_statistics()
                    print(f"\nDistance traveled: {tracker_stats['total_distance']:.2f}m, "
                          f"Avg speed: {tracker_stats['avg_velocity']:.3f} m/s")
            
            time.sleep(1/60)
        
        if not all_goals_reached:
            print(f"\nMax steps reached ({max_steps}).")
        
        # Print final statistics
        perf_logger.print_statistics()
        tracker.print_statistics()
        tracker.save_trajectory_to_file()
    
    except KeyboardInterrupt:
        print("\nStopped by user")
        # Print statistics even on interrupt
        perf_logger.print_statistics()
        tracker.print_statistics()
        tracker.save_trajectory_to_file()
    except Exception as e:
        print(f"\nError: {e}")
        perf_logger.print_statistics()
        tracker.print_statistics()
    
    try:
        p.disconnect()
    except:
        pass


if __name__ == "__main__":
    main()

