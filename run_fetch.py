"""Single-command launcher for the membrane fetch demo."""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description="Run the PyBullet fetch simulation and agent")
    parser.add_argument("--dummy", action="store_true", help="run a no-cost one-action plumbing check")
    parser.add_argument("--provider", choices=("groq", "claude"), default="groq",
                        help="vision model provider (default: groq)")
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--max-steps", type=int, default=60)
    parser.add_argument("--command", default="Fetch the requested object and deliver it to the marked zone.")
    args = parser.parse_args()
    if not 1 <= args.max_steps <= 60:
        parser.error("--max-steps must be between 1 and 60")
    env = os.environ.copy()
    env["FETCH_SEED"] = str(args.seed)
    project_dir = Path(__file__).resolve().parent
    server = subprocess.Popen([sys.executable, "-m", "uvicorn", "api.fetch_server:app", "--host", "127.0.0.1", "--port", str(args.port)], env=env, cwd=project_dir)
    try:
        import httpx
        ready = False
        for _ in range(100):
            if server.poll() is not None:
                raise RuntimeError("Fetch server exited during startup")
            try:
                response = httpx.get(f"http://127.0.0.1:{args.port}/health", timeout=0.5)
                if response.status_code == 200 and response.json().get("task") == "fetch" and response.json().get("ready"):
                    ready = True
                    break
            except (httpx.HTTPError, ValueError):
                time.sleep(0.2)
        if not ready:
            raise RuntimeError(f"Fetch API did not become ready on port {args.port}")
        if not args.dummy:
            key_name = "GROQ_API_KEY" if args.provider == "groq" else "ANTHROPIC_API_KEY"
            if not env.get(key_name):
                raise RuntimeError(f"Set {key_name} or use --dummy")
            counter_path = project_dir / "logs" / "run_count.txt"
            count = int(counter_path.read_text(encoding="utf-8").strip()) if counter_path.exists() else 0
            if count >= 120:
                raise RuntimeError("The 120-run development limit has been reached")
            counter_path.parent.mkdir(parents=True, exist_ok=True)
            counter_path.write_text(f"{count + 1}\n", encoding="utf-8")
        from agent.claude_agent import run_agent
        result = run_agent(args.command, base_url=f"http://127.0.0.1:{args.port}", dummy=args.dummy,
                           max_steps=args.max_steps, provider=args.provider)
        print(result)
    finally:
        server.terminate()
        try:
            server.wait(timeout=5)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()


if __name__ == "__main__":
    main()
