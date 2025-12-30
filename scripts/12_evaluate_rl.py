#!/usr/bin/env python3
"""Evaluate trained RL agent for PyBullet navigation."""
import os
import sys
import argparse
import numpy as np
import time
import pybullet as p
from stable_baselines3 import PPO, SAC, TD3

# Add parent directory to path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from scripts.pybullet_nav_env import PyBulletNavEnv


def evaluate(args):
    """Evaluate trained model."""
    # Determine model path
    if args.model_path:
        model_path = args.model_path
    else:
        # Try to find a compatible model - check both best and final
        # First, check which models exist and which one matches the current observation space
        test_env = PyBulletNavEnv(render_mode=None, env_type=args.env_type)
        env_obs_space = test_env.observation_space
        test_env.close()
        
        model_path = None
        compatible_model = None
        
        # Check final model first (most recent)
        final_model_path = f'models/{args.algorithm}_{args.env_type}_final'
        if os.path.exists(final_model_path + '.zip'):
            try:
                if args.algorithm.lower() == 'ppo':
                    test_model = PPO.load(final_model_path)
                elif args.algorithm.lower() == 'sac':
                    test_model = SAC.load(final_model_path)
                elif args.algorithm.lower() == 'td3':
                    test_model = TD3.load(final_model_path)
                else:
                    test_model = None
                
                if test_model and test_model.observation_space.shape == env_obs_space.shape:
                    compatible_model = final_model_path
                    print(f"✓ Found compatible final model: {final_model_path}")
            except Exception as e:
                print(f"Could not load final model: {e}")
        
        # Check best model
        best_model_path = 'models/best_model'
        if os.path.exists(best_model_path + '.zip'):
            try:
                if args.algorithm.lower() == 'ppo':
                    test_model = PPO.load(best_model_path)
                elif args.algorithm.lower() == 'sac':
                    test_model = SAC.load(best_model_path)
                elif args.algorithm.lower() == 'td3':
                    test_model = TD3.load(best_model_path)
                else:
                    test_model = None
                
                if test_model and test_model.observation_space.shape == env_obs_space.shape:
                    if compatible_model is None:
                        compatible_model = best_model_path
                        print(f"✓ Found compatible best model: {best_model_path}")
            except Exception as e:
                print(f"Could not load best model: {e}")
        
        # Use compatible model if found, otherwise use final model (will show error later)
        if compatible_model:
            model_path = compatible_model
        elif os.path.exists(final_model_path + '.zip'):
            model_path = final_model_path
        elif os.path.exists(best_model_path + '.zip'):
            model_path = best_model_path
        else:
            print(f"Error: No trained model found.")
            print(f"Expected: {final_model_path}.zip or {best_model_path}.zip")
            print(f"Run training first: python scripts/11_train_rl.py")
            return
    
    print(f"Loading model from {model_path}...")
    
    # Load model based on algorithm
    try:
        if args.algorithm.lower() == 'ppo':
            model = PPO.load(model_path)
        elif args.algorithm.lower() == 'sac':
            model = SAC.load(model_path)
        elif args.algorithm.lower() == 'td3':
            model = TD3.load(model_path)
        else:
            raise ValueError(f"Unknown algorithm: {args.algorithm}")
        
        print("✓ Model loaded successfully")
        
        # Check observation space compatibility - CRITICAL CHECK
        # Create a test env to check observation space
        test_env = PyBulletNavEnv(render_mode=None, env_type=args.env_type)
        model_obs_space = model.observation_space
        env_obs_space = test_env.observation_space
        
        # #region agent log - observation space check
        try:
            import json
            with open('/Users/abhi/Desktop/Github_Projects/pybullet_sim/.cursor/debug.log', 'a') as f:
                log_entry = json.dumps({
                    "sessionId": "debug-session",
                    "runId": "run1",
                    "hypothesisId": "G",
                    "location": "12_evaluate_rl.py:evaluate",
                    "message": "Observation space check",
                    "data": {
                        "model_path": str(model_path),
                        "model_obs_shape": list(model_obs_space.shape),
                        "env_obs_shape": list(env_obs_space.shape),
                        "match": bool(model_obs_space.shape == env_obs_space.shape)
                    },
                    "timestamp": int(time.time() * 1000)
                }) + "\n"
                f.write(log_entry)
        except: pass
        # #endregion
        
        if model_obs_space.shape != env_obs_space.shape:
            print(f"\n{'='*60}")
            print(f"⚠️  CRITICAL ERROR: Observation space mismatch!")
            print(f"{'='*60}")
            print(f"   Model was trained with: {model_obs_space.shape[0]} dimensions")
            print(f"   Environment now provides: {env_obs_space.shape[0]} dimensions")
            print(f"\n   This causes the model to read WRONG values from observations!")
            print(f"   Example: Model reads 'distance' as 'velocity', 'velocity' as 'angular velocity', etc.")
            print(f"   This completely breaks the model's behavior - it will get stuck immediately!")
            
            # Try to find a compatible model
            print(f"\n   Attempting to find a compatible model...")
            final_model_path = f'models/{args.algorithm}_{args.env_type}_final'
            best_model_path = 'models/best_model'
            compatible_model = None
            
            # Check final model
            if os.path.exists(final_model_path + '.zip') and final_model_path != model_path:
                try:
                    if args.algorithm.lower() == 'ppo':
                        test_model = PPO.load(final_model_path)
                    elif args.algorithm.lower() == 'sac':
                        test_model = SAC.load(final_model_path)
                    elif args.algorithm.lower() == 'td3':
                        test_model = TD3.load(final_model_path)
                    else:
                        test_model = None
                    
                    if test_model and test_model.observation_space.shape == env_obs_space.shape:
                        compatible_model = final_model_path
                        print(f"   ✓ Found compatible final model: {final_model_path}")
                except:
                    pass
            
            # Check best model
            if compatible_model is None and os.path.exists(best_model_path + '.zip') and best_model_path != model_path:
                try:
                    if args.algorithm.lower() == 'ppo':
                        test_model = PPO.load(best_model_path)
                    elif args.algorithm.lower() == 'sac':
                        test_model = SAC.load(best_model_path)
                    elif args.algorithm.lower() == 'td3':
                        test_model = TD3.load(best_model_path)
                    else:
                        test_model = None
                    
                    if test_model and test_model.observation_space.shape == env_obs_space.shape:
                        compatible_model = best_model_path
                        print(f"   ✓ Found compatible best model: {best_model_path}")
                except:
                    pass
            
            if compatible_model:
                print(f"\n   Using compatible model: {compatible_model}")
                test_env.close()
                # Recursively call with the compatible model
                args.model_path = compatible_model
                return evaluate(args)
            else:
                print(f"\n   No compatible model found. You MUST retrain the model with the new observation space.")
                print(f"   Run: python scripts/11_train_rl.py --algorithm {args.algorithm} --timesteps 1000000 --num-envs 4")
                print(f"{'='*60}\n")
                test_env.close()
                return
        else:
            print(f"✓ Observation space matches: {model_obs_space.shape}")
        test_env.close()
    except Exception as e:
        print(f"Error loading model: {e}")
        return
    
    # Create environment with rendering
    env = PyBulletNavEnv(render_mode='human' if args.render else None, 
                        env_type=args.env_type)
    
    # Run episodes
    print(f"\n{'='*60}")
    print(f"Running {args.num_episodes} evaluation episodes")
    print(f"{'='*60}\n")
    
    success_count = 0
    total_rewards = []
    episode_lengths = []
    distances_traveled = []
    
    for ep in range(args.num_episodes):
        obs, info = env.reset()
        done = False
        total_reward = 0
        steps = 0
        start_pos = env.robot.get_pose()[:2]
        
        print(f"--- Episode {ep+1}/{args.num_episodes} ---")
        print(f"Initial distance to goal: {info['distance_to_goal']:.2f}m")
        
        while not done:
            # Get action from model
            action, _ = model.predict(obs, deterministic=args.deterministic)
            
            # #region agent log - only log every 10th step during evaluation to reduce volume
            if steps % 10 == 0:  # Only log every 10th step
                try:
                    import json
                    pose = env.robot.get_pose()
                    with open('/Users/abhi/Desktop/Github_Projects/pybullet_sim/.cursor/debug.log', 'a') as f:
                        log_entry = json.dumps({
                            "sessionId": "debug-session",
                            "runId": "run1",
                            "hypothesisId": "D",
                            "location": "12_evaluate_rl.py:evaluate",
                            "message": "Action predicted",
                            "data": {
                                "episode": int(ep + 1),
                                "step": int(steps),
                                "robot_pos": [float(pose[0]), float(pose[1]), float(pose[2])],
                                "goal_pos": [float(env.goal[0]), float(env.goal[1])],
                                "distance_to_goal": float(info['distance_to_goal']),
                                "action": [float(action[0]), float(action[1])],
                                "obs_dx_norm": float(obs[0]),
                                "obs_dy_norm": float(obs[1]),
                                "obs_dtheta_norm": float(obs[2]),
                                "obs_distance_norm": float(obs[3])
                            },
                            "timestamp": int(time.time() * 1000)
                        }) + "\n"
                        f.write(log_entry)
                except: pass
            # #endregion
            
            # Step environment
            try:
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                steps += 1
                done = terminated or truncated
                
                # Render
                if args.render:
                    env.render()
                    time.sleep(1/30)  # ~30 FPS
                
                # Print progress
                if steps % 50 == 0:
                    print(f"  Step {steps}: dist={info['distance_to_goal']:.2f}m, "
                          f"reward={total_reward:.1f}")
            except p.error as e:
                if "Not connected to physics server" in str(e):
                    print(f"\n  Warning: Physics server disconnected at step {steps}")
                    break
                else:
                    raise
        
        # Calculate distance traveled
        final_pos = env.robot.get_pose()[:2]
        distance_traveled = np.linalg.norm(np.array(final_pos) - np.array(start_pos))
        distances_traveled.append(distance_traveled)
        
        # Check success
        success = info['distance_to_goal'] < env.goal_threshold
        success_count += int(success)
        total_rewards.append(total_reward)
        episode_lengths.append(steps)
        
        status = "✓ SUCCESS" if success else "✗ FAILED"
        print(f"{status} | Steps: {steps} | Reward: {total_reward:.1f} | "
              f"Final distance: {info['distance_to_goal']:.2f}m | "
              f"Distance traveled: {distance_traveled:.2f}m")
        print()
    
    # Print summary
    print("="*60)
    print("EVALUATION RESULTS")
    print("="*60)
    print(f"Success rate: {success_count}/{args.num_episodes} "
          f"({100*success_count/args.num_episodes:.1f}%)")
    print(f"Average reward: {np.mean(total_rewards):.2f} ± {np.std(total_rewards):.2f}")
    print(f"Average episode length: {np.mean(episode_lengths):.1f} ± {np.std(episode_lengths):.1f} steps")
    print(f"Average distance traveled: {np.mean(distances_traveled):.2f}m ± {np.std(distances_traveled):.2f}m")
    print("="*60)
    
    env.close()


def main():
    parser = argparse.ArgumentParser(description='Evaluate trained RL agent')
    parser.add_argument('--algorithm', type=str, default='ppo',
                       choices=['ppo', 'sac', 'td3'],
                       help='RL algorithm used (default: ppo)')
    parser.add_argument('--env-type', type=str, default='warehouse',
                       choices=['warehouse', 'street', 'maze'],
                       help='Environment type (default: warehouse)')
    parser.add_argument('--model-path', type=str, default=None,
                       help='Path to model file (default: auto-detect)')
    parser.add_argument('--num-episodes', type=int, default=10,
                       help='Number of evaluation episodes (default: 10)')
    parser.add_argument('--render', action='store_true',
                       help='Render evaluation episodes')
    parser.add_argument('--deterministic', action='store_true', default=True,
                       help='Use deterministic policy (default: True)')
    
    args = parser.parse_args()
    
    evaluate(args)


if __name__ == "__main__":
    main()

