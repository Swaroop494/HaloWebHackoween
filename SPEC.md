# SPEC: Technical Contract

This file is the single source of truth for interfaces, conventions and data formats. If code and this file disagree, fix one of them in the same commit. Never change an endpoint, a units convention, or a log field without updating this file.

## 1. Repository layout

```
membrane-fetch/
├── AGENTS.md              # rules for AI coding agents
├── PRD.md                 # product requirements
├── SPEC.md                # this file
├── README.md              # how to run (TODO(you))
├── .env.example           # env var names only
├── .gitignore             # ignore .env, logs/, clips/, __pycache__
├── requirements.txt
├── engine/                # PyBullet + HTTP server. NEVER imports agent/
│   ├── __init__.py
│   ├── scene.py           # scene generation (seeded)
│   ├── robot.py           # kinematic robot, collisions, state machine
│   ├── camera.py          # /look and /depth rendering
│   ├── server.py          # FastAPI app on :8080
│   ├── fetch_environment.py # private six-object task scene
│   ├── pickup_manager.py  # fixed-constraint pickup and validation
│   ├── fetch_state.py     # task state machine
│   ├── fetch_server.py    # single-owner-thread Fetch API
│   └── clip.py            # POV clip recorder (ffmpeg)
├── api/                   # public launch import for Fetch HTTP API
├── evaluation/            # JSONL and frame logging
├── agent/                 # Claude loop. Talks to engine ONLY over HTTP
│   ├── __init__.py
│   ├── main.py            # CLI: python -m agent.main --command "..."
│   ├── loop.py            # tool-use loop
│   ├── prompts.py         # system prompt (leak-checked)
│   ├── tools.py           # tool schemas (leak-checked)
│   ├── engine_client.py   # httpx wrapper
│   └── logger.py          # JSONL + PNG frame writer
├── tests/
│   ├── test_scene.py
│   ├── test_endpoints.py
│   ├── test_leak.py       # the most important test
│   └── test_logs.py
├── tools/
│   └── run_batch.py       # run N seeded scenarios and tally outcomes
├── run_fetch.py           # single-command Fetch demo launcher
├── viewer/                # optional Three.js dashboard (Phase 1 signed off first)
└── logs/                  # gitignored: run_XXX/ with log.jsonl + frames/
```

Only the directories above exist. Add files only inside them, or update this table first.

## 2. Coordinates and units

| Quantity | Convention |
|---|---|
| Distance | metres |
| Angles in the API | degrees |
| Room | square, centred at origin, walls at x = ±3.0 and y = ±3.0 (6 m × 6 m) |
| World axes | +x right on the map, +y up on the map, z up (PyBullet) |
| Heading | 0 = +x axis. Positive is clockwise when viewed from above. Range (-180, 180] |
| PyBullet yaw | counter-clockwise positive. Conversion: `yaw_rad = -heading_deg * pi / 180` |
| Turn direction | positive `degrees` turns right (clockwise), speed 90°/s |
| Move speed | 1 m/s forward or backward |
| Physics step | 1/240 s internal; physics advances only during `/move` and `/turn` |

## 3. Scene

Generated from an integer seed. Same seed must give the same scene.

| Element | Rule |
|---|---|
| Walls | 4 walls, 6 m × 6 m, no furniture (unless obstacle stretch goal is enabled) |
| Objects | Exactly 6 primitives drawn from {cube, sphere, cylinder, cone} |
| Object sizes | 0.2–0.6 m across; heights vary so "tallest" is meaningful |
| Colours | from a palette of at least 6. At least two objects share a colour |
| Shape repeats | At least two objects share a shape |
| Drop zone | flat coloured square, 1.0 m × 1.0 m, on the floor. Visual only. Its bounds are known to the engine only |
| Robot start | random position in the room, at least 1.5 m from walls and from the drop zone; random heading |
| Object placement | at least 0.8 m apart; at least 1.0 m from robot start; not inside the drop zone |

Engine-only data (never sent to Claude): object list with ids, shapes, colours, sizes, positions; drop-zone bounds; robot target id.

## 4. Robot

| Property | Value |
|---|---|
| Body | Existing URDF chassis (0.3×0.2×0.1 m) with two driven wheels and caster |
| Motion | Existing `scripts/diff_drive_robot.py` differential-drive controller and URDF wheel physics |
| Camera | Existing simulated onboard camera, 0.5 m above floor, 0.15 m forward of centre, tilted 10° down |
| Camera | 640×480; 90° horizontal FOV (73.74° vertical in PyBullet) |
| Pickup reach | object surface within 0.5 m of the robot's front point, and object within ±30° of heading |
| Carry | held object is attached at 0.3 m in front of the robot, 0.3 m above floor |
| Max holding | one object |

## 5. State machine

States: `IDLE`, `LOCATE`, `APPROACH`, `PICKUP`, `CARRY`, `DROP`, `DONE`, `BLOCKED`.

| From | Event | To |
|---|---|---|
| IDLE | server initializes the seeded scene | LOCATE |
| LOCATE | first move/turn | APPROACH |
| APPROACH | `/pickup` succeeds | CARRY |
| APPROACH | `/pickup` fails | LOCATE |
| CARRY | `/drop` called | DROP (internal validation) |
| DROP | held body is inside the zone | DONE (`pass`) |
| DROP | held body is outside the zone | CARRY (retry) |
| Any moving state | collision | BLOCKED |
| BLOCKED | reverse or turn recovery action | previous state |
| BLOCKED | retry count reaches 3 | DONE (`fail_collision`) |
| Any | action count reaches 60 | DONE (`fail_not_reached`) |

`DONE` is the only terminal state. The engine validates delivery internally; no zone coordinates or zone result are sent to Claude.

Outcomes: `pass`, `fail_wrong_object`, `fail_not_reached`, `fail_collision`.

## 6. Engine HTTP API

Base URL: `http://localhost:8080`. All bodies are JSON unless noted. Errors use HTTP 4xx with `{"error": "message"}`.

### GET /state

Response:
```json
{
  "state": "LOCATE",
  "x": 1.23,
  "y": -0.45,
  "heading_deg": 37.0,
  "holding": false,
  "retry_count": 0,
  "collided": false
}
```
No object information in this response.

### GET /look

Returns only a 640×480 `image/jpeg` from the simulated robot camera.

### GET /depth (optional)

Response:
```json
{ "distance_m": 2.31 }
```
`distance_m` is `null` if nothing is hit within 10 m. Single ray from camera centre, straight ahead.

### POST /move

Request:
```json
{ "distance": 0.5 }
```
Negative means reverse. Stops early on collision.

Response: `{ "moved_m": 0.31, "collided": true, "state": "BLOCKED" }`.

### POST /turn

Request:
```json
{ "degrees": 45 }
```
Positive turns right.

Response: `{ "turned_deg": 45, "heading_deg": 82.0, "state": "APPROACH" }`.

### POST /pickup

Request: empty body `{}`.

Response (success or failure only, never names what was picked up):
```json
{ "success": true, "holding": true, "state": "CARRY" }
```

### POST /drop

Request: empty body `{}`.

Response:
```json
{ "released": true, "holding": false, "state": "CARRY", "retry_count": 1 }
```
The drop-zone check is internal; the response contains no zone result.

### Clock rule

Simulation time advances only inside `/move` and `/turn`. All other endpoints are instantaneous.

## 7. Claude tool definitions

Exposed to Claude. The schemas below are the only tools. Descriptions must pass the leak test.

| Tool | Input | Maps to |
|---|---|---|
| `move` | `distance_m` (number, -2.0 to 2.0), `what_i_see` (string), `why` (string) | POST /move |
| `turn` | `degrees` (number, -180 to 180), `what_i_see` (string), `why` (string) | POST /turn |
| `pickup` | `what_i_see` (string), `why` (string) | POST /pickup |
| `drop` | `what_i_see` (string), `why` (string) | POST /drop |

Rules:
- `what_i_see` and `why` are required on every call. They go in the log and are judged in Phase 1 (25%).
- `tool_choice` is `{"type": "any"}` with parallel tool use disabled. Exactly one call per step.
- Claude has no tool for `/state`, `/depth` or `/look`. The loop attaches the image and a text status automatically.

## 8. Per-step message to Claude

Each step is one user message with two content blocks:

1. **Text block**, always the same format:
```
Command: {command}
State: {state}
Holding: {true|false}
Collision: {true|false}
Retries: {retry_count}
Last action result: {result or "none"}
```
2. **Image block**: JPEG from the most recent `/look`, base64, media type `image/jpeg`.

Rules:
- Send the original JPEG image block; do not turn the scene into a textual description.
- Fetch a new image after every move or turn. Pickup/drop decisions reuse the last observed image.
- Keep only the newest image in the request history; older pixels are removed.
- `/depth` is disabled unless `FETCH_MENTOR_APPROVED_DEPTH=true` is explicitly set.

## 9. System prompt rules

Must include (generic, no object data):
- The robot's job: fetch the object matching the command and deliver it to the marked zone.
- Units and heading convention (metres, degrees, 0 = +x, clockwise positive).
- That all information about the world comes from the camera image and the status text.
- Strategy: if the target is not visible, turn in place to scan. Centre the target in the frame before driving. Judge distance from the camera image. The drop zone is a flat marked area; look for it in the image.
- Recovery: after a collision, back off and turn before retrying.
- Reasoning: describe what you see before choosing an action, and fill `what_i_see` and `why` every time.

Must NOT include: colours, shapes, object counts, positions of objects or the zone, names of objects, or example object sets resembling the scene.

## 10. Logging

### Run directory
```
logs/run_001/
├── log.jsonl
├── frames/
│   ├── step_000.png
│   ├── step_001.png
│   └── ...
├── meta.json
└── clip.mp4          (optional)
```

### log.jsonl (one JSON object per line)
```json
{
  "run_id": "run_001",
  "step": 3,
  "timestamp": "2026-11-05T10:12:03+05:30",
  "command": "Bring the round red thing",
  "tool": "turn",
  "args": {"degrees": 45, "what_i_see": "...", "why": "..."},
  "result": {"turned_deg": 45, "heading_deg": 82.0, "state": "Locate"},
  "frame_path": "frames/step_003.png",
  "model": "claude-sonnet-5-5",
  "tokens_in": 1834,
  "tokens_out": 96,
  "cost_usd": 0.009,
  "engine_state": "Locate",
  "collided": false
}
```

### meta.json
```json
{
  "run_id": "run_001",
  "seed": 1234,
  "command": "Bring the round red thing",
  "started": "...",
  "ended": "...",
  "outcome": "pass",
  "steps": 17,
  "total_cost_usd": 0.16,
  "git_commit": "abc1234"
}
```

Frames are saved for every step, including the ones that trigger Blocked.

## 11. POV clip (Phase 2)

| Property | Value |
|---|---|
| Format | MP4, H.264, 1280×720, 30 FPS |
| Length | 10–20 s, enforced by the recorder (fail the build if over 20 s) |
| Start | when state leaves Idle |
| Stop | when state reaches Done |
| Overlay | command text, first 2 seconds, centred, bottom third |
| Forbidden on screen | team names, logos, code, file paths, the engine's debug text |
| Camera | separate POV camera, rendered at 1280×720, not the 640×480 agent camera |
| Audio | none |

Command overlay uses ffmpeg `drawtext`. Trim or retime with `setpts` if the run is too long, and log the adjustment in meta.json.

## 12. Budget and run limits

| Item | Value |
|---|---|
| Model | `claude-sonnet-5-5` |
| Budget target | $40 total per team |
| Mentor alert | $30 |
| Estimated cost per step | about $0.009 |
| Soft step limit | 25 |
| Hard step limit | 60 |
| Stop developing at | about 120 runs |
| Dev model for plumbing tests | Haiku or dummy agent |

A run counter lives in `logs/run_count.txt`. The agent refuses to start a new run once the counter reaches 120 unless `--force` is set.

## 13. Environment

Python 3.11.

`requirements.txt`:
```
pybullet>=3.2.6
numpy
pillow
opencv-python
fastapi
uvicorn
httpx
anthropic
pydantic
pytest
```

System: `ffmpeg` on PATH.

Environment variables (names in `.env.example`, values never committed):
```
ANTHROPIC_API_KEY=
ENGINE_URL=http://localhost:8080
CLAUDE_MODEL=claude-sonnet-5-5
MAX_STEPS=60
SOFT_STEPS=25
```

## 14. Commands

```
# end-to-end Fetch demo
python run_fetch.py --command "Fetch the requested item"

# no-cost plumbing mode
python run_fetch.py --dummy --seed 1234

# tests
pytest -q

# leak test only
pytest tests/test_leak.py -q

# batch of seeded runs (uses budget, asks first)
python tools/run_batch.py --seeds 1-10 --command "Bring the tallest object"
```

## 15. Acceptance tests

| Test | Pass condition |
|---|---|
| `test_fetch_task.py` | Six-object scene, onboard image size, fixed-constraint pickup, drop validation, collision recovery |
| Endpoint smoke check | API state, image, move, turn, pickup/drop responses, and gated `/depth` |
| `test_leak.py` | Every JSON response from `/state`, `/turn`, `/move`, `/pickup`, `/drop` and all headers contain no object ids, names, colours, shapes, or positions. Tool descriptions and system prompt contain no colour or shape words from the palette |
| `test_logs.py` | Each log line has all required fields; frame files exist; no two consecutive steps with images and no move/turn between them |
| Clip check | ffprobe reports 10–20 s, 1280×720 or larger |
