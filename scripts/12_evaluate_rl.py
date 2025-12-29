#!/usr/bin/env python3
"""Evaluate trained RL agent for PyBullet navigation."""
import os
import sys
import argparse
import numpy as np
import time
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
        # Try to find best model
        model_path = 'models/best_model'
        if not os.path.exists(model_path + '.zip'):
            # Try final model
            model_path = f'models/{args.algorithm}_{args.env_type}_final'
            if not os.path.exists(model_path + '.zip'):
                print(f"Error: No trained model found.")
                print(f"Expected: {model_path}.zip")
                print(f"Run training first: python scripts/11_train_rl.py")
                return
    
    print(f"Loading model from {model_path}...")
    
    # Load model based on algorithm
    if args.algorithm.lower() == 'ppo':
        model = PPO.load(model_path)
    elif args.algorithm.lower() == 'sac':
        model = SAC.load(model_path)
    elif args.algorithm.lower() == 'td3':
        model = TD3.load(model_path)
    else:
        raise ValueError(f"Unknown algorithm: {args.algorithm}")
    
    print("✓ Model loaded successfully")
    
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
            
            # Step environment
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

