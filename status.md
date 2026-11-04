# Status

## Membrane Fetch implementation

- Reuses `scripts/diff_drive_robot.py` and `urdf/diff_drive_robot.urdf`; loaded driven wheel joint IDs are left `0` and right `1`.
- Added a seeded six-object room, four walls, floor, marked drop zone, fixed-constraint pickup, internal delivery validation, and the `IDLE` through `DONE` / `BLOCKED` state machine with three-retry collision recovery.
- Added the single-owner-thread FastAPI server and routes in `api/fetch_server.py` / `engine/fetch_server.py`. Camera output is 640×480 at 90° horizontal FOV, mounted 0.5 m above the floor with 10° downward tilt. Depth remains disabled unless explicitly approved.
- Added the Claude tool loop, image-only camera input, latest-frame handling, JSONL/PNG logging, and `run_fetch.py` launcher.
- Added a Groq vision path (default model `qwen/qwen3.8-27b`) using one strict-schema action per decision; Claude remains selectable with `--provider claude` and uses native tool calls. Both send the simulated JPEG as image content and execute one action per decision.
- Added `scripts/preview_fetch_scene.py` to open the room and robot in the PyBullet GUI without an AI key. The preview was launched for inspection.
- Updated `README.md` with exact PowerShell commands for previewing the scene, Groq tests, switching to Claude, existing examples, and tests.

## Verification

- `python -m pytest -q`: 8 passed, including Groq image payload conversion and rate-limit wait parsing.
- `python -m py_compile` on the changed Python modules: passed.
- HTTP smoke checks covered state, camera image dimensions, move, turn, pickup, drop, depth gating, collision, recovery, and internal drop validation.
- `python run_fetch.py --dummy --seed 73 --port 8081 --max-steps 1`: launcher, camera capture, action, refreshed frame, and logging succeeded.
- Groq uses the documented vision-capable Qwen model and image-plus-tool request format; no Groq key was present, so no provider request was sent.
- Added bounded Groq HTTP 429 retries that honor `Retry-After` or the wait duration in the error message.
- A live one-action Groq run completed on port 8081: image sent, a schema-valid turn selected and executed, and a fresh image fetched.

## Limits

- Port 8080 was occupied by another listener during verification; the new launcher was verified on port 8081. Use `--port 8081` or stop the existing listener before using the default port.
- The provided Groq key enabled a one-action test, but no full Fetch run was attempted. The key was supplied in chat and should be rotated before further use. No key was written to the repository or logs. No Anthropic key was available, so Claude was not tested.
- Existing scripts `05_environment_builder.py` and `06_navigate_environment.py` were run during inspection but currently fail in this environment (PyBullet data path and Windows console encoding, respectively). `03_sensor_streams.py` starts its interactive loop and must be interrupted with `Ctrl+C`.
