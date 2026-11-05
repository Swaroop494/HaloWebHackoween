#!/usr/bin/env python3
"""Run the premium robotics facility environment."""
import subprocess
import sys
import time
import os


def run_server(port: int, env_var: str = "PREMIUM_GUI", name: str = "Premium"):
    """Run a PyBullet server."""
    env = os.environ.copy()
    env[env_var] = "true" if "--gui" in sys.argv else "false"

    cmd = [
        sys.executable,
        "-m",
        f"engine.premium_server" if port == 8081 else "engine.fetch_server",
    ]

    print(f"Starting {name} server on port {port}...")
    proc = subprocess.Popen(cmd, env=env)

    # Wait for server to start
    time.sleep(2)

    return proc


def main():
    """Main entry point."""
    print("=" * 60)
    print("Premium Robotics Facility Environment")
    print("=" * 60)
    print()
    print("This script starts both the original and premium servers.")
    print()

    processes = []

    try:
        # Start original server
        orig_proc = run_server(8080, "FETCH_GUI", "Original")
        processes.append(orig_proc)
        print("Original server started on port 8080")
        print()

        # Start premium server
        premium_proc = run_server(8081, "PREMIUM_GUI", "Premium")
        processes.append(premium_proc)
        print("Premium server started on port 8081")
        print()

        print("Both servers are running.")
        print("Original API: http://localhost:8080")
        print("Premium API: http://localhost:8081")
        print()
        print("Press Ctrl+C to stop all servers.")

        # Wait for both processes
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nStopping servers...")
        for proc in processes:
            proc.terminate()

        for proc in processes:
            proc.wait()

        print("All servers stopped.")


if __name__ == "__main__":
    main()
