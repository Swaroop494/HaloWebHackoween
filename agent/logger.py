import json
import os
from datetime import datetime
from PIL import Image
import io

class RunLogger:
    def __init__(self, run_id, seed, command):
        self.run_dir = os.path.join('logs', run_id)
        self.frames_dir = os.path.join(self.run_dir, 'frames')
        os.makedirs(self.frames_dir, exist_ok=True)
        self.log_path = os.path.join(self.run_dir, 'log.jsonl')
        self.meta_path = os.path.join(self.run_dir, 'meta.json')
        self.run_id = run_id
        self.seed = seed
        self.command = command
        self.started = datetime.now().isoformat()
        self.total_cost = 0.0
        meta = {
            'run_id': run_id,
            'seed': seed,
            'command': command,
            'started': self.started,
        }
        with open(self.meta_path, 'w') as f:
            json.dump(meta, f)

    def log_step(self, step, tool, args, result, frame_bytes, engine_state, collided, model='claude-sonnet-5-5', tokens_in=0, tokens_out=0, cost_usd=0.0):
        frame_path = os.path.join(self.frames_dir, f'step_{step:03d}.png')
        # save frame
        try:
            img = Image.open(io.BytesIO(frame_bytes))
            img.save(frame_path)
        except Exception:
            pass
        entry = {
            'run_id': self.run_id,
            'step': step,
            'timestamp': datetime.now().isoformat(),
            'command': self.command,
            'tool': tool,
            'args': args,
            'result': result,
            'frame_path': frame_path.replace('\\\\', '/'),
            'model': model,
            'tokens_in': tokens_in,
            'tokens_out': tokens_out,
            'cost_usd': cost_usd,
            'engine_state': engine_state,
            'collided': collided,
        }
        with open(self.log_path, 'a') as f:
            f.write(json.dumps(entry) + '\n')
        self.total_cost += cost_usd
        return frame_path

    def finish(self, outcome, steps):
        meta = {
            'run_id': self.run_id,
            'seed': self.seed,
            'command': self.command,
            'started': self.started,
            'ended': datetime.now().isoformat(),
            'outcome': outcome,
            'steps': steps,
            'total_cost_usd': self.total_cost,
            'git_commit': '',
        }
        with open(self.meta_path, 'w') as f:
            json.dump(meta, f)
