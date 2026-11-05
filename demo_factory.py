#!/usr/bin/env python3
"""Demo script for premium sci-fi factory environment."""
import pybullet as p
import pybullet_data
from engine.fetch_environment import FetchEnvironment
from engine import config
import time
import math

def main():
    # Connect with GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Configure viewer
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
    
    # Set initial camera view
    p.resetDebugVisualizerCamera(
        cameraDistance=7,
        cameraYaw=45,
        cameraPitch=-25,
        cameraTargetPosition=[0, 0, 0.5]
    )
    
    print("=" * 60)
    print("PREMIUM SCI-FI ROBOTICS R&D FACILITY")
    print("=" * 60)
    print("\nBuilding environment...")
    
    # Build environment
    env = FetchEnvironment(seed=42)
    env.build()
    
    print(f"\n✓ Floor created with panel grid and safety markings")
    print(f"✓ {len(env.wall_ids)} walls with metal trims and panel seams")
    print(f"✓ Ceiling with LED strips and structural beams")
    print(f"✓ Drop zone at ({env.drop_center[0]:.2f}, {env.drop_center[1]:.2f})")
    print(f"✓ {len(env.object_ids)} task objects placed")
    print(f"✓ Factory obstacles: crates, charging stations, barriers")
    
    # Load robot
    robot_urdf = "urdf/diff_drive_robot.urdf"
    robot_id = p.loadURDF(
        robot_urdf,
        [env.robot_start[0], env.robot_start[1], config.ROBOT_BASE_Z],
        p.getQuaternionFromEuler([0, 0, env.robot_start[2]])
    )
    print(f"✓ Robot spawned at ({env.robot_start[0]:.2f}, {env.robot_start[1]:.2f})")
    
    # List objects
    print("\nTask Objects:")
    for obj_id in env.object_ids:
        meta = env.object_meta[obj_id]
        print(f"  • {meta['colour_name']} {meta['shape']} at ({meta['position'][0]:.2f}, {meta['position'][1]:.2f})")
    
    print("\n" + "=" * 60)
    print("DEMO RUNNING - Press Ctrl+C to exit")
    print("=" * 60)
    
    # Run physics simulation
    step = 0
    try:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
            
            # Rotate camera view every 500 steps
            if step % 500 == 0:
                yaw = (step // 500) * 90
                p.resetDebugVisualizerCamera(
                    cameraDistance=7,
                    cameraYaw=yaw,
                    cameraPitch=-25,
                    cameraTargetPosition=[0, 0, 0.5]
                )
            
            step += 1
            
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    
    p.disconnect()
    print("Demo complete!")

if __name__ == "__main__":
    main()
