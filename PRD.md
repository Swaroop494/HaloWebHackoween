# PRD: Membrane Robot Fetch

Version: 2.0
Event: Membrane Robot Hackathon, 24 hours
Phase 1 sign-off target: hour 14
Hidden-command clip upload deadline: hour 20

---

## 1. Summary

We are building a simulated wheeled robot that fetches the object a person names, and delivers it to a marked drop zone in a small room. A plain-language command goes in. A finished fetch comes out. Nobody touches the keyboard or mouse during a run.

The robot's brain is Claude, a vision-language model. Claude decides every action from what the robot's camera sees. The simulation engine must never tell Claude where objects are, what they are called, or what colour or shape they are. This is the central constraint of the whole project and it is called the **no-leakage rule** (section 6).

The build has three moving parts:

- **Engine**: a PyBullet physics world, exposed as a local HTTP server. It owns the room, the objects, the robot's motion, collisions and the state machine.
- **Agent**: a Python program that runs Claude's tool-use loop. It only talks to the engine over HTTP.
- **Logs and clips**: every step is logged with the exact image Claude saw. Each run can be exported as a short point-of-view video for the crowd vote.

---

## 2. Background and why this matters

A robot that can follow a free-form instruction, look for the thing it was asked about, and recover when it bumps into something, is a basic form of **embodied AI**. Practical versions of this idea show up in:

- **Warehouse picking**: "pick the blue box with the damaged corner" instead of reprogramming for every new product.
- **Home and service robots**: "I spilled my drink, can you help?" The robot has to pick a plan that is physically possible.
- **Inspection robots**: navigating a facility and reading gauges, as Boston Dynamics and Google DeepMind demonstrated with Spot and Gemini Robotics-ER.
- **Simulation as training data**: an agent that explores a simulated world can generate trajectories for learning later.

For the hackathon, the interesting question is not whether a robot can move. It is whether a general vision-language model can identify the right object from pixels alone, plan a route, and recover from its own mistakes. The judging rewards exactly that.

Related work our team should read (listed in section 19): SayCan, Code as Policies, Inner Monologue, COME-robot, VoxPoser, Gemini Robotics-ER, Anthropic's Project Fetch.

---

## 3. Goals

### Phase 1: Robotics build (65% of total score)
1. A typed command starts a run.
2. The robot finds the named object using camera images only.
3. It drives to the object, picks it up, carries it to the drop zone and releases it.
4. The run ends with a clear outcome: `pass`, `fail_wrong_object`, `fail_not_reached`, or `fail_collision`.
5. At least three different commands reach `pass` in a live demo to a mentor.
6. The logs prove Claude identified objects from pixels.

### Phase 2: Human feedback (20% of total score)
1. Record one point-of-view clip for each of three hidden commands published by the judges at hour 18.
2. Upload all three clips through the Membrane feedback API by hour 20.
3. Real users watch each clip and answer two questions (section 15).

### Stretch goal (15% of total score)
Build **one** of the following and show it working in the demo:
- **Obstacle path (chosen for this project):** a box and a vase placed across the robot's straight path to the target. Claude must route around them using only camera images, depth and collision feedback. Details in section 9.8.
- Plan the full route using only `move` and `turn`, with no `go_to` shortcut.
- Multi-view reasoning, such as "bring the cube that is farthest from the green square".
- Improve behaviour using crowd results, with a before-and-after clip.

We are building the obstacle path. Other stretch goals are not started.

### Non-goals (we are deliberately not building these)
- A second robot.
- A drone.
- Realistic grasping, wheel physics, or motor simulation.
- Voice input or speech output.
- A Three.js viewer before Phase 1 sign-off. It is optional and comes last (section 17).
- Any use of engine object data by the agent, under any circumstances.

---

## 4. Users

| User | What they do | What they need from us |
|---|---|---|
| Judges | Give two live commands; read tool-call and image logs; watch clips | Reliable live runs; clear logs; readable clips |
| Crowd voters | Watch a 10–20 s clip and answer two questions | Short, clear clips with the command shown at the start; no team identity on screen |
| Our team | Build, test, debug, demo | Fast iteration; cheap runs; clear rules for AI coding tools |
| Mentors | Sign off Phase 1; approve extra sensors | Working demo; an agreed list of sensors |

---

## 5. Scope

### 5.1 In scope

| Area | Included |
|---|---|
| Room | Four walls, flat floor, 6 m × 6 m |
| Objects | 4–6 primitive shapes (cube, sphere, cylinder, cone) in varied colours and sizes |
| Drop zone | One flat, coloured square on the floor |
| Obstacles | One box and one vase, placed across the straight path between robot and target (obstacle stretch goal) |
| Robot | One wheeled robot, capsule body, forward-facing camera |
| Engine | PyBullet in DIRECT mode, FastAPI server on localhost:8080 |
| Agent | Python, Claude tool-use loop, one tool call per step |
| Logs | JSONL per run, one PNG per step |
| Clips | MP4 recording of the robot's point of view, with command overlay |
| Tests | Endpoint tests, scene determinism tests, leak test, log validator |
| Batch runs | Seeded runs for development, with a run counter |

### 5.2 Out of scope (Phase 1 and 2)
- Physical hardware.
- Multi-robot coordination.
- Any non-primitive objects except the box and vase (which are obstacles, not targets).
- Lighting effects, shadows beyond PyBullet defaults, post-processing.
- Accounts, login, or a public web app.

---

## 6. The no-leakage rule (critical)

### 6.1 What it means

Claude may only learn about the world from:
1. The camera image from `/look` (the robot's only view of objects).
2. Simple status: the robot's own state, whether it is holding something, whether it collided, and the optional distance straight ahead from `/depth`.
3. The result of its own actions.

### 6.2 What is forbidden

Claude must never receive any of the following, in any form:
- object names, IDs, colours, shapes, sizes, positions or counts
- distances to any object, the drop zone, or the obstacles
- segmentation masks, bounding boxes, coordinates of anything
- drop-zone coordinates or bounds
- filenames, headers, JSON keys, image metadata or log fields that contain any of the above
- colour or shape words in the system prompt or tool descriptions, even as examples

### 6.3 Why it matters

It is the difference between a robot that sees and a program that cheats. The judges check the logs to see that Claude reasoned from pixels. The brief states that any object-data leakage from the engine scores **0 on both Phase 1 components**: image-driven agent (25%) and Fetch works (40%). A leak discovered during judging also damages our credibility for the whole demo.

### 6.4 How we enforce it
- The engine and agent live in separate folders and communicate only over HTTP.
- `/state` and all other responses are defined by a schema that contains no object fields.
- `tests/test_leak.py` scans every engine response, the system prompt and the tool definitions for forbidden words. It must pass before any commit that touches the engine's API or the agent.
- Every AI coding tool follows `AGENTS.md` rules on this.

---

## 7. Functional requirements

Each requirement has an ID and an acceptance check. The check is what we actually run.

### 7.1 Scene

| ID | Requirement | Acceptance check |
|---|---|---|
| S1 | The scene is generated from an integer seed | Same seed gives identical object list, positions and colours |
| S2 | Each scene has 4–6 target objects from {cube, sphere, cylinder, cone} | Test counts and shapes |
| S3 | Objects have heights and sizes varied enough that "tallest" and "biggest" are meaningful | Height spread of at least 0.3 m in every scene |
| S4 | Colours come from the palette in section 9.5, with at least two objects sharing one colour in every scene | Test on 100 seeds |
| S5 | At least two objects share a shape in every scene | Test on 100 seeds |
| S6 | Objects are at least 0.8 m apart and at least 1.0 m from the robot's start | Test on 100 seeds |
| S7 | The drop zone is a flat 1.0 m × 1.0 m coloured square, not overlapping any object | Test on 100 seeds |
| S8 | The robot starts at a random position at least 1.5 m from walls and drop zone, with random heading | Test on 100 seeds |
| S9 | Obstacles (box and vase) are placed across the straight line from robot to target in at least 80% of seeds, and never on top of objects or the drop zone | Test on 100 seeds |

### 7.2 Robot motion (kinematic differential drive)

We do not simulate wheels or motors. The robot is a **kinematic differential-drive** body: its position and heading are set directly by the engine, one small step at a time, and physics is used only to detect contact.

| ID | Requirement | Acceptance check |
|---|---|---|
| R1 | `/move {distance}` moves the robot forward or backward at 1 m/s | Timing and displacement match within 2 cm |
| R2 | `/move` advances in 5 cm sub-steps, checking for contact after each | Unit test |
| R3 | On contact, `/move` stops, backs off 0.1 m, returns `collided=true`, and enters Blocked | Scripted test driving into a wall |
| R4 | `/turn {degrees}` rotates in place at 90°/s; positive means right (clockwise) | Heading after turn matches |
| R5 | Heading 0 = +x, clockwise positive, range (-180, 180] | Convention test with known angles |
| R6 | PyBullet yaw conversion happens in one function only: `yaw = -heading` | Code review check |
| R7 | Simulated time advances only during `/move` and `/turn` | Test confirms physics stepping count |
| R8 | Robot never leaves the room: walls are hard limits | Scripted test |
| R9 | Robot can hold at most one object | Test |

### 7.3 Camera and perception

| ID | Requirement | Acceptance check |
|---|---|---|
| C1 | `/look` returns a 640×480 JPEG from the robot's front camera | Image size check |
| C2 | Camera sits 0.5 m above the floor, 0.15 m forward of the robot centre, tilted 10° down | Geometry test |
| C3 | Horizontal field of view is 90°; PyBullet vertical FOV is 73.74° | Calculation test |
| C4 | `/depth` returns the distance straight ahead, up to 10 m, or `null` | Test with a wall at known distance |
| C5 | `/look` never adds object text, labels, or overlays to the image | Visual inspection of 20 frames |
| C6 | The image is taken from the same render the POV clip uses for the same moment, so logs and clips match | Frame comparison test |

### 7.4 Actions

| ID | Requirement | Acceptance check |
|---|---|---|
| A1 | `/pickup` attaches the nearest object within reach (0.5 m from robot front, within ±30° of heading), or returns failure | Scripted test |
| A2 | `/pickup` returns only `success` and `holding`. It never names the object | Leak test |
| A3 | A held object is carried 0.3 m in front of the robot, 0.3 m above the floor | Visual check |
| A4 | `/drop` releases the held object and returns `in_zone`, whether it landed in the drop zone | Scripted test |
| A5 | `/drop` reveals nothing about where the zone is, beyond `in_zone` | Leak test |

### 7.5 State machine

States: `Idle`, `Locate`, `GoToObject`, `PickUp`, `Carry`, `Drop`, `Done`, `Blocked`.

| ID | Requirement | Acceptance check |
|---|---|---|
| M1 | Transitions follow SPEC.md section 5 exactly | Table-driven test |
| M2 | `Blocked` is entered on collision or 2 s without movement | Scripted test |
| M3 | Three failed recoveries end the run as `fail_collision` | Test |
| M4 | 60 steps without success end the run as `fail_not_reached` | Test |
| M5 | Dropping the wrong object in the zone ends the run as `fail_wrong_object` | Test |
| M6 | `Done` is terminal | Test |

### 7.6 Agent and Claude loop

| ID | Requirement | Acceptance check |
|---|---|---|
| G1 | Claude receives the command, latest image, status text and last five steps as one-line summaries | Message format test |
| G2 | Claude returns exactly one tool call per step | Log check |
| G3 | Every move, turn, pickup and drop includes `what_i_see` and `why` | Log validator |
| G4 | Never two image blocks in a row without an intervening move or turn | Log validator |
| G5 | Each step is logged to JSONL with its frame path and token/cost fields | Log validator |
| G6 | Prompt caching is enabled on the system prompt and tool definitions | Usage log shows cache reads |
| G7 | The run stops at 25 soft steps (warning) and 60 hard steps (end) | Test with dummy agent |
| G8 | A run counter stops real runs at 120 unless forced | Test |
| G9 | Dummy agent (no API calls) can drive every endpoint | Test |

### 7.7 Logs and clips

| ID | Requirement | Acceptance check |
|---|---|---|
| L1 | Each run has its own folder: `logs/run_XXX/` with `log.jsonl`, `frames/`, `meta.json` | Folder test |
| L2 | `meta.json` records seed, command, outcome, step count, total cost, git commit | Schema test |
| L3 | Clip is MP4, H.264, 1280×720, 30 FPS | ffprobe |
| L4 | Clip is 10–20 seconds; the recorder fails the run if longer than 20 s | Test |
| L5 | Clip starts when state leaves Idle and ends when it reaches Done | Test |
| L6 | Command text is overlaid for the first 2 seconds | Visual check |
| L7 | Clip contains no team names, logos, code, file paths, or debug text | Manual review |
| L8 | Clip is rendered from a separate 1280×720 POV camera | Code review |

---

## 8. Non-functional requirements

| Category | Requirement |
|---|---|
| Performance | Engine runs at 30 FPS or better during physics; `/look` returns in under 100 ms on a laptop |
| Hardware | Runs on a 16 GB laptop with no GPU. TinyRenderer is CPU-based |
| Cost | Target $40 per team; mentor alert at $30; hard stop at 120 runs during development |
| Latency per step | Claude call 2–5 s; engine response under 100 ms |
| Reproducibility | Any run can be replayed from its seed and log |
| Security | No secrets in the repo; API key only in environment variables |
| Privacy | No personal data in logs or clips |
| Maintainability | Modules as in SPEC.md section 1; tunable numbers in one config file |
| Public repo | Assume judges read every committed string. No team or personal names in code |

---

## 9. Scene and asset specification

### 9.1 Room

- Square room, 6 m × 6 m, centred at the origin.
- Walls at x = ±3.0 m and y = ±3.0 m, 1.0 m tall, 0.1 m thick. Walls must be taller than the camera (0.5 m) so the robot can never see over them and the room always looks enclosed.
- Floor and walls use the environment colours in section 9.5.

### 9.2 Target objects

- 4–6 per scene, from {cube, sphere, cylinder, cone}.
- Size 0.2–0.6 m across. Heights varied so height comparisons are meaningful.
- Each object is a PyBullet body with a collision shape and a visual shape of the same primitive.
- Objects are dynamic bodies so they can fall or be knocked, but robot pickup attaches them kinematically.

### 9.3 Drop zone

- Flat 1.0 m × 1.0 m square on the floor, visual only, with a thin border.
- Its bounds are known only to the engine.
- `in_zone` is true when the object's centre is inside the square when released.

### 9.4 Robot

- Capsule body: radius 0.15 m, height 0.4 m.
- Centre of mass at the capsule centre.
- Camera mounted 0.15 m forward of centre, 0.5 m above floor, tilted 10° downward.
- Visual: charcoal body with a small light front marker, so the camera's facing direction is obvious in the POV clip.

### 9.5 Colour palette

Chosen for visual appeal and for contrast against the warm environment. Each colour is distinct enough that a vision model can tell them apart on a 640×480 image.

**Environment**
| Element | Colour | Hex |
|---|---|---|
| Floor | Warm sand | #E9DCC3 |
| Walls | Soft cream | #F7F3EA |
| Drop zone | Sage green with a darker border | #8FAE8B, border #5E7F5B |
| Robot body | Charcoal | #3A3F44 |
| Robot front marker | Warm white | #FFF8E7 |

**Target objects (6 colours; at least two repeat in each scene)**
| Name in the palette | Hex | Note |
|---|---|---|
| Coral | #E07A5F | Warm, reads clearly against sand |
| Teal | #3D8B8B | Cool, strong contrast with coral |
| Mustard | #E9B44C | Bright, distinct from sand floor |
| Dusty plum | #7D5A7F | Dark accent |
| Sky blue | #6FA8DC | Light cool tone |
| Indigo | #3F4E8C | Deep cool blue-violet, distinct from teal and sky blue |

Rules:
- Floor is always sand, never a target colour, so targets never blend in.
- Drop zone is sage, which no target uses, so the zone is always identifiable as its own area.
- There is no true red among targets. Coral is the warmest target colour. Command generation must never use the word "red" unless the scene contains exactly one coral object. Colour words must match palette names exactly ("coral", "teal", "mustard", "plum", "sky blue", "indigo").
- The palette's colour words are used in commands. They must not appear in the system prompt or tool descriptions (section 6.2).

### 9.6 Obstacles (stretch goal, chosen)

Obstacles are never targets and never shaped or coloured like targets.

| Obstacle | Size | Colour | Collision shape |
|---|---|---|---|
| Box | 0.5 m × 0.5 m × 0.5 m | Warm white with a faint wood tone, #D8C8A8 | Box |
| Vase | 0.35 m diameter, 0.6 m tall | Soft blue-grey glaze, #9FB4C7 | Cylinder approximation for physics, with a narrower neck visual |

Placement rules:
- Placed on the straight line between robot start and target object, at a point 40–60% of the way along, with a 0.3 m lateral offset in a random direction.
- The box and vase must never overlap each other, any object, the drop zone, or the robot start.
- The robot can push obstacles, but it cannot pick them up. `/pickup` ignores obstacles.
- Collision with an obstacle is treated like a wall collision: Blocked, back off, turn, retry.

### 9.7 Lighting and rendering

- One directional light from above-front, soft shadows on.
- Background colour matches the cream wall colour.
- POV clip uses the same lighting as the agent camera, so what judges see is what Claude saw.

### 9.8 Obstacle path scenarios

Test scenes will include:
1. Target directly behind the box.
2. Target behind the vase, with the vase directly in line.
3. Box and vase on both sides of the path, leaving a gap.
4. No obstacle on the path (control case).

---

## 10. Commands and tests

### 10.1 Command format
- One plain-language sentence per run.
- Examples the judges may use: "Bring the round red thing", "Bring the tallest object", "Get the sky blue cube".
- Commands must refer to something unambiguous in the scene. Our test generator checks that the target is unique for the command. If not unique, the command is rejected.

### 10.2 Development command set
- At least 20 varied commands covering: shape ("the sphere"), colour ("the teal one"), height ("the shortest object"), relative position ("the object closest to the box"), and combinations ("the round red thing").
- Keep a list in `tests/commands.yaml` (created by the coding agent after asking the user; see section 13).

### 10.3 Test plan

| Test | Pass condition |
|---|---|
| Scene determinism | Same seed, same scene |
| Scene rules | All of S2–S9 pass on 100 seeds |
| Endpoint schema | All endpoints match SPEC.md |
| Motion | Move and turn accuracy within tolerance |
| Collision | Wall and obstacle collisions set the right flags |
| Leak | Engine responses, system prompt and tool descriptions contain no forbidden words |
| Logs | Every step writes JSONL and PNG; validator passes |
| Clip | Length 10–20 s, resolution 1280×720, overlay present |
| End-to-end | Three different commands reach `pass` in live runs |

---

## 11. Engine behaviour in detail

### 11.1 Run lifecycle
1. The agent calls `POST /run/start` with seed, run ID and command. The engine builds the scene, places the robot and obstacles, and returns `ready`.
2. The agent loops: `GET /look`, Claude decides, the agent calls one action endpoint, and the engine advances the simulation for that action only.
3. The run ends on `Done`. The agent calls `POST /run/end`, which returns the outcome.
4. The clip recorder stops and the run folder is finalised.

### 11.2 Clock
- The engine's physics clock is frozen between actions.
- `/move` and `/turn` advance the clock in small steps.
- Claude's thinking time does not advance the scene.

### 11.3 Error handling
- Invalid requests (e.g. `/move` with `distance` out of range) return HTTP 422 with a message.
- If the engine crashes, the agent logs the failure and stops the run. It does not guess.
- If a Claude call fails, the agent retries once, then records a failed step and stops.

---

## 12. Claude behaviour

### 12.1 Tools
- `move`, `turn`, `pickup`, `drop`. Each requires `what_i_see` and `why`.
- Exactly one call per step. Parallel tool use disabled.

### 12.2 Per-step message
- One text block with command, step count, last action and result, collision and holding status, distance ahead, and the last five steps in one line each.
- One image block with the latest `/look` frame.

### 12.3 System prompt principles
- Describes the job and units.
- Teaches strategy: scan by turning when the target is not visible, centre the target before driving, use depth to judge distance, recover after collisions, look for the flat drop-zone area.
- Forbids object-specific hints (no colours, shapes, counts or positions).
- Asks Claude to describe what it sees before choosing an action.

### 12.4 Models and budget
- Claude Sonnet 5.5 for real runs, with vision enabled.
- Haiku or the dummy agent for plumbing and endpoint work.
- Budget target $40; alert at $30; stop development at about 120 runs.

---

## 13. Clarifying questions the coding agent must ask the user

The coding agent must not guess these. Before writing the code that depends on each answer, it asks the user and waits.

**Before starting the engine (hour 0–1)**
1. Which operating system and Python version does your laptop use? (We recommend Python 3.11.)
2. Do you already have `ffmpeg` installed, or should the agent use `imageio-ffmpeg` as a fallback?
3. Are you using Cursor, Antigravity, Kiro, Kilo Code, or a mix? Which one should own which folder?
4. Do you want the engine to run in a terminal window you can watch, or fully headless?

**Before the scene code (hour 0–3)**
5. Confirm the colour palette in section 9.5, or tell me which colours to change.
6. Confirm the room size (6 m × 6 m) and object size range (0.2–0.6 m).
7. Which object count do you want as the default: 4, 5, or 6 (the agent can randomise within 4–6 otherwise)?

**Before the agent code (hour 5–8)**
8. Do you have your Sonnet 5.5 API key ready in an environment variable? (The agent will never ask you to paste it into a file.)
9. Should the first real run use Sonnet, or do you want a Haiku dry run first to test plumbing and save budget?

**Before the obstacle work (hour 14–18)**
10. Confirm the box and vase designs in section 9.6, or describe changes.
11. Should the vase be a true vase shape or a simple cylinder with a neck? (A cylinder is faster and more reliable for physics.)

**Before the clip recorder (hour 11–14)**
12. Confirm the clip overlay style: centred, bottom third, white text with a dark translucent band.
13. Your team name, which stays off screen. Tell me the exact string so the agent can check that it never appears in a clip.

**Before the hidden commands (hour 18)**
14. Who runs the three hidden-command tests, and who uploads the clips?

If the user answers "use your judgement" to any question, the agent applies the default in this PRD, states the default in its reply, and records the decision in `SPEC.md`.

---

## 14. Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Accidental object-data leak | Medium | Very high (0 score) | Separate folders; HTTP-only boundary; leak test on every commit; prompt review |
| Claude misidentifies objects | High | High | `what_i_see` field; scan strategy; log reasoning; test commands |
| Budget overrun | Medium | High | Dummy and Haiku runs; prompt caching; one image per step; run counter |
| Clip over 20 seconds | Medium | Score 0 for Phase 2 clip | Small room; step cap; retime with `setpts`; recorder fails if over 20 s |
| `pybullet` install fails | Medium | High early | Python 3.11; conda-forge fallback; test install in first hour |
| Ambiguous colour commands | Medium | Medium | Command generator checks uniqueness; avoid "red" unless unique |
| Robot stuck behind obstacle | Medium | Medium | Blocked state with backoff and turn; 60-step cap |
| Scope creep into second robot or drone | Medium | High | This PRD; changes need an explicit decision |
| Team runs out of sleep and makes errors | High | Medium | Sign-off checkpoint at hour 14; mentor check-ins |
| Clip shows team identity | Low | Score 0 | Overlay and recorder check for forbidden strings |

---

## 15. Phase 2: Human feedback

### 15.1 Clip requirements
- MP4, 720p or higher, no audio.
- 10–20 seconds.
- Starts when the robot leaves Idle, stops when it reaches Done.
- Command shown as text for the first 2 seconds.
- No team names, logos or code on screen.

### 15.2 Questions asked to the crowd
- **Pulse 1 (binary):** "Did the robot bring the correct object to the drop zone?" Yes or no.
- **Pulse 2 (label):** "What went wrong?" Hit something; wrong object; did not reach the drop zone; too slow; nothing wrong.

### 15.3 Scoring
- Each clip receives at least 30 responses.
- Our score is the average Pulse 1 yes-rate and the average "nothing wrong" share across the three clips.

### 15.4 Process
1. Hour 18: judges publish three hidden commands. Run each once, with no retries.
2. Save each run's log and clip.
3. Hour 20: upload all three clips. Anything late scores 0.
4. Within 45 minutes: result sheet per clip arrives.

---

## 16. Judging and our priorities

| Component | Weight | Our plan |
|---|---|---|
| Phase 1: Fetch works | 40% | Reliable live fetch on two judge commands; direct routes |
| Phase 1: Image-driven agent | 25% | Clear logs with `what_i_see` and `why`; recovery from collisions visible |
| Phase 2: User verdict | 20% | Clean clips; command overlay; no identity leaks |
| Stretch goal | 15% | Obstacle path, shown working live and in the clip |
| Tie-breaker | — | Phase 1 demo score |

Priority order if time runs short:
1. Phase 1 reliability on three commands.
2. No-leakage compliance.
3. Clean logs.
4. Clips meeting length and content rules.
5. Obstacle stretch goal.
6. Viewer (probably skipped).

---

## 17. Timeline

| Hours | Milestone |
|---|---|
| 0–1 | Answer section 13 questions 1–4; environment setup; `pybullet` installed |
| 1–3 | Scene generator with palette; kinematic robot; collision; `/look` renders correctly |
| 3–5 | All endpoints; state machine; drop-zone check; curl tests; dummy agent drives a full run |
| 5–8 | Claude loop; logging; first successful fetch |
| 8–11 | 15–20 randomised scenes; varied commands; fix failure modes |
| 11–14 | Clip recorder; three clean passes with nobody touching the keyboard; mentor demo and sign-off |
| 14–16 | Obstacle path (box and vase); scene rule updates; test on the four scenarios |
| 16–18 | Obstacle path passes in at least 80% of seeded runs; prepare for hidden commands |
| 18 | Hidden commands published; run each once |
| 18–20 | Record and export clips; upload |
| 20 | Hard deadline for clip upload |

Viewer work (Three.js) happens only if time remains after hour 20 and is optional.

---

## 18. Team roles (suggested)

| Role | Owns | Main tools |
|---|---|---|
| Engine lead | `engine/` | PyBullet, FastAPI |
| Agent lead | `agent/`, `tests/test_logs.py` | Anthropic SDK |
| Scene and clips | `engine/scene.py`, `engine/clip.py` | PyBullet, ffmpeg |
| Tester and logs | `tests/`, `tools/run_batch.py` | pytest |
| Presenter | Demo, clip review, judge interaction | — |

With a solo team, work in the timeline order and use the coding tools on their own folders as `AGENTS.md` describes.

---

## 19. References

- SayCan: https://say-can.github.io
- Code as Policies: https://lastweekin.ai/p/code-as-policies
- Inner Monologue: https://ar5iv.labs.arxiv.org/html/2207.05608
- COME-robot (GPT-4V closed loop): https://come-robot.github.io
- VoxPoser: https://github.com/huangwl18/VoxPoser
- Microsoft PromptCraft-Robotics: https://github.com/microsoft/PromptCraft-Robotics
- Gemini Robotics-ER 1.6: https://deepmind.google/blog/gemini-robotics-er-1-6
- Anthropic Project Fetch: https://www.anthropic.com/research/project-fetch-robot-dog
- Claude Agent SDK in embodied robotics (arXiv 2601.20334): https://arxiv.org/pdf/2601.20334
- ALRM, agentic LLM for manipulation (arXiv 2601.19510): https://arxiv.org/pdf/2601.19510
- XLeRobot LLM agent tutorial: https://xlerobot.readthedocs.io/en/latest/software/getting_started/LLM_agent.html
- Strands Robots (MuJoCo, natural-language control): https://github.com/strands-labs/robots
- LeRobot (Hugging Face): https://github.com/huggingface/lerobot
- Hackathon retrospectives: https://robotics.umich.edu/news/2025/hackathon-qa-brouze and https://seeed.cc/post/2025-embodied-ai-hackathon-recap
