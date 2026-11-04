"""JSONL decision logging and per-decision PNG frames."""
from __future__ import annotations

import json
import os
import subprocess
from datetime import datetime, timezone
from io import BytesIO
from pathlib import Path

from PIL import Image


class FetchLogger:
    def __init__(self, run_id: str, command: str, root: str | None = None):
        if root is None:
            root = str(Path(__file__).resolve().parents[1] / "logs")
        self.directory = Path(root) / run_id
        self.frames = self.directory / "frames"
        self.frames.mkdir(parents=True, exist_ok=True)
        self.path = self.directory / "log.jsonl"
        self.command = command
        self.run_id = run_id
        self.step = 0

    def record(self, tool: str, args: dict, result: dict, image: bytes,
               state: dict, model: str = "dummy", tokens_in: int = 0,
               tokens_out: int = 0, cost_usd: float | None = None) -> None:
        frame_rel = Path("frames") / f"step_{self.step:03d}.png"
        Image.open(BytesIO(image)).convert("RGB").save(self.directory / frame_rel)
        entry = {"run_id": self.run_id, "step": self.step,
                 "timestamp": datetime.now(timezone.utc).astimezone().isoformat(),
                 "command": self.command, "tool": tool, "args": args,
                 "result": result, "frame_path": frame_rel.as_posix(),
                 "model": model, "tokens_in": tokens_in, "tokens_out": tokens_out,
                 "cost_usd": cost_usd, "engine_state": state.get("state"),
                 "collided": bool(state.get("collided", False))}
        with self.path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(entry, ensure_ascii=False) + "\n")
        self.step += 1

    def finalize(self, seed: int | None, outcome: str, total_tokens_in: int,
                 total_tokens_out: int, total_cost_usd: float | None = None) -> None:
        """Write the run summary without engine-private scene information."""
        root = Path(__file__).resolve().parents[1]
        try:
            commit = subprocess.run(["git", "rev-parse", "HEAD"], cwd=root,
                                    capture_output=True, text=True, check=True).stdout.strip()
        except (OSError, subprocess.CalledProcessError):
            commit = None
        metadata = {
            "run_id": self.run_id,
            "seed": seed,
            "command": self.command,
            "outcome": outcome,
            "step_count": self.step,
            "total_tokens_in": int(total_tokens_in),
            "total_tokens_out": int(total_tokens_out),
            "total_cost_usd": None if total_cost_usd is None else float(total_cost_usd),
            "git_commit": commit,
        }
        (self.directory / "meta.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

