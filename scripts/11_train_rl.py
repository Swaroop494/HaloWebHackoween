#!/usr/bin/env python3
"""Train RL agent for PyBullet navigation using Stable-Baselines3."""
import os
import sys
import argparse
from stable_baselines3 import PPO, SAC, TD3
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
from stable_baselines3.common.utils import set_random_seed

# Add parent directory to path
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from scripts.pybullet_nav_env import PyBulletNavEnv


def make_env(rank=0, seed=0, env_type="warehouse", render_mode=None):
    """Create environment for vectorized training."""
    def _init():
        env = PyBulletNavEnv(render_mode=render_mode, env_type=env_type)
        env = Monitor(env, filename=None, allow_early_resets=True)
        env.reset(seed=seed + rank)
        return env
    set_random_seed(seed + rank)
    return _init


def train(args):
    """Main training function."""
    print("="*60)
    print("PyBullet Navigation RL Training")
    print("="*60)
    print(f"Algorithm: {args.algorithm}")
    print(f"Environment: {args.env_type}")
    print(f"Total timesteps: {args.timesteps:,}")
    print(f"Number of parallel environments: {args.num_envs}")
    print("="*60)
    
    # Create directories
    os.makedirs('logs/tensorboard', exist_ok=True)
    os.makedirs('models', exist_ok=True)
    os.makedirs('models/checkpoints', exist_ok=True)
    
    # Test environment
    print("\nCreating test environment...")
    test_env = PyBulletNavEnv(render_mode=None, env_type=args.env_type)
    print("Checking environment...")
    check_env(test_env, warn=True)
    print("✓ Environment check passed!")
    test_env.close()
    
    # Create vectorized environments
    print(f"\nCreating {args.num_envs} parallel environments...")
    env = SubprocVecEnv([make_env(i, args.seed, args.env_type) 
                        for i in range(args.num_envs)])
    
    # Evaluation environment (single, with optional rendering)
    eval_render_mode = "human" if args.render_eval else None
    eval_env = Monitor(PyBulletNavEnv(render_mode=eval_render_mode, env_type=args.env_type))
    
    # Callbacks
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path='models/',
        log_path='logs/',
        eval_freq=max(10000 // args.num_envs, 1000),
        deterministic=True,
        render=False,
        n_eval_episodes=5
    )
    
    checkpoint_callback = CheckpointCallback(
        save_freq=max(50000 // args.num_envs, 5000),
        save_path='models/checkpoints/',
        name_prefix=f'{args.algorithm}_{args.env_type}'
    )
    
    # Create model
    print(f"\nCreating {args.algorithm} model...")
    
    if args.algorithm.lower() == 'ppo':
        model = PPO(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=args.learning_rate,
            n_steps=2048,
            batch_size=256,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            vf_coef=0.5,
            tensorboard_log='logs/tensorboard',
            device=args.device,
            seed=args.seed
        )
    elif args.algorithm.lower() == 'sac':
        model = SAC(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=args.learning_rate,
            buffer_size=100000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            ent_coef='auto',
            tensorboard_log='logs/tensorboard',
            device=args.device,
            seed=args.seed
        )
    elif args.algorithm.lower() == 'td3':
        model = TD3(
            "MlpPolicy",
            env,
            verbose=1,
            learning_rate=args.learning_rate,
            buffer_size=100000,
            learning_starts=1000,
            batch_size=256,
            tau=0.005,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            tensorboard_log='logs/tensorboard',
            device=args.device,
            seed=args.seed
        )
    else:
        raise ValueError(f"Unknown algorithm: {args.algorithm}")
    
    # Train
    print(f"\n{'='*60}")
    print("Starting training...")
    print(f"Monitor with: tensorboard --logdir logs/tensorboard")
    print(f"{'='*60}\n")
    
    model.learn(
        total_timesteps=args.timesteps,
        callback=[eval_callback, checkpoint_callback],
        progress_bar=True
    )
    
    # Save final model
    final_model_path = f'models/{args.algorithm}_{args.env_type}_final'
    model.save(final_model_path)
    print(f"\n✓ Training complete!")
    print(f"✓ Final model saved to: {final_model_path}")
    print(f"✓ Best model saved to: models/best_model")
    
    env.close()
    eval_env.close()


def main():
    parser = argparse.ArgumentParser(description='Train RL agent for PyBullet navigation')
    parser.add_argument('--algorithm', type=str, default='ppo',
                       choices=['ppo', 'sac', 'td3'],
                       help='RL algorithm to use (default: ppo)')
    parser.add_argument('--env-type', type=str, default='warehouse',
                       choices=['warehouse', 'street', 'maze'],
                       help='Environment type (default: warehouse)')
    parser.add_argument('--timesteps', type=int, default=500000,
                       help='Total training timesteps (default: 500000)')
    parser.add_argument('--num-envs', type=int, default=4,
                       help='Number of parallel environments (default: 4)')
    parser.add_argument('--learning-rate', type=float, default=3e-4,
                       help='Learning rate (default: 3e-4)')
    parser.add_argument('--device', type=str, default='cpu',
                       choices=['cpu', 'cuda'],
                       help='Device to use (default: cpu)')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed (default: 42)')
    parser.add_argument('--render-eval', action='store_true',
                       help='Render evaluation episodes')
    
    args = parser.parse_args()
    
    train(args)


if __name__ == "__main__":
    main()

