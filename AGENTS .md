# AGENTS.md: Rules for AI coding agents

Every AI coding tool working in this repo (Cursor, Antigravity, Kiro, Kilo Code, or any other) must follow these rules. Read this file, `PRD.md` and `SPEC.md` before making any change.

## 1. Read first

1. `PRD.md`: what we are building and why.
2. `SPEC.md`: the contract. Interfaces, units, data formats and the state machine are defined there.
3. This file.

If a request conflicts with `SPEC.md`, stop and ask. Do not silently change the contract.

## 2. The most important rule: no object data reaches Claude

Claude may only learn about objects from `/look` images, plus `collided`, `holding`, and `distance_m` from `/depth`.

Never send to Claude, in any form:
- object names, IDs, colours, shapes, sizes, positions, counts
- distances to objects or to the drop zone
- segmentation masks, bounding boxes, or object coordinates
- drop-zone coordinates or bounds
- any filename, header, JSON key or log field that contains the above

This applies to:
- `/state` responses
- system prompt text
- tool names and descriptions
- image metadata and EXIF
- anything in the message history

Violations score 0 on both Phase 1 judging components. The leak test (`tests/test_leak.py`) must pass before any commit that touches `agent/` or `engine/server.py`.

When writing prompts or tool descriptions, do not use colour words or shape words from the scene palette, even as examples.

## 3. Boundaries

- `engine/` must never import from `agent/`.
- `agent/` talks to the engine only through HTTP (`engine_client.py`). It must not import `engine` modules.
- The engine's scene internals (`scene.py` object list) must not be exposed on any endpoint except the engine-only control endpoints (`/run/start`, `/run/end`), and those must not return object data.
- One tool owns one folder at a time. If you are working in `engine/`, do not edit `agent/`, and the reverse.

## 4. Scope discipline

- Build one robot. No drone, no second robot. Do not add them.
- Stretch goals: only the one chosen in `PRD.md` section 10. Do not start a second one.
- The Three.js viewer is off-limits until Phase 1 sign-off is recorded in `PRD.md`.
- Do not add dependencies not listed in `SPEC.md` section 13 without asking.
- Do not add files outside the layout in `SPEC.md` section 1 without updating that section first.

## 5. Units and conventions

- Metres and degrees everywhere in the API. Radians only inside PyBullet calls, converted in one place.
- Heading: 0 = +x, clockwise positive. PyBullet yaw is counter-clockwise, so convert with `yaw = -heading`. Do this conversion in one function (`engine/robot.py`) and nowhere else.
- Positive `degrees` in `/turn` means right (clockwise).
- Negative `distance` in `/move` means reverse.

## 6. Budget and cost safety

- Never run the real Claude agent in tests. Tests use `--dummy` or mocked responses.
- Do not run `tools/run_batch.py` without asking the human first.
- Check `logs/run_count.txt` before any real run. Stop at 120 unless the human says otherwise.
- Use the Haiku model or the dummy agent for plumbing and endpoint work. Use Sonnet only for end-to-end runs.
- Never remove or bypass the step limits (25 soft, 60 hard).

## 7. Secrets and repo hygiene

- Never write an API key into any file, commit message, log, or test. Read it from `ANTHROPIC_API_KEY`.
- Never commit `.env`, `logs/`, `clips/`, or `__pycache__/`. Check `.gitignore` exists before the first commit.
- The repo is public. Assume every committed string is read by judges. No team names, personal names, or emails in code or clips. Keep the team name out of any clip.
- Don't commit large binaries (videos, big frame dumps). Keep logs out of git; keep only one sample run in `docs/samples/` if needed.

## 8. Logging requirements

Every real agent step must write one JSONL line and one PNG, per `SPEC.md` section 10. Do not skip logging to speed up a run. Every log line must include `what_i_see` and `why` in `args` when the tool is a move, turn, pickup or drop.

If a field is missing, fix the code. Don't edit the log validator to accept it.

## 9. Code standards

- Python 3.11, type hints on public functions, `pydantic` models for every request and response body.
- Keep functions short. One responsibility per module, as listed in `SPEC.md`.
- Prefer deterministic code. Any randomness goes through a seeded `random.Random(seed)` instance.
- No global mutable state except the engine's single `World` object, which is created in `server.py`.
- Put tunable numbers (speeds, reach, FOV, room size) in `engine/config.py` as named constants. Don't scatter magic numbers.
- Comments explain why, not what.

## 10. Testing before every commit

Run before committing:
```
pytest -q
```

All tests must pass. In particular:
- `tests/test_leak.py` must pass.
- `tests/test_endpoints.py` must pass if engine code changed.
- `tests/test_logs.py` must pass if agent code changed.

Do not disable or skip a failing test to get a commit through. Fix the code or ask the human.

## 11. Git conventions

- Commit small and often. One logical change per commit.
- Commit message format: `<area>: <what changed>`, for example `engine: add kinematic move with collision backoff`.
- Areas: `engine`, `agent`, `scene`, `robot`, `camera`, `clip`, `logs`, `tests`, `docs`, `chore`.
- Do not force-push. Do not rewrite history on the main branch.
- Before committing, confirm `SPEC.md` still matches the code. If an endpoint changed, the same commit updates `SPEC.md`.

## 12. When to stop and ask the human

Stop and ask before:
- changing any endpoint, field name, or unit in `SPEC.md`
- adding a new dependency or a new top-level folder
- running real Claude calls beyond 5 runs in one session
- running `tools/run_batch.py`
- changing the step limits, budget numbers, or the model name
- anything that would put object information into Claude's context, even "temporarily" or "for debugging"
- deleting logs or frames from a run someone may still need for judging

## 13. Debugging a run

1. Check `logs/run_XXX/log.jsonl` for the failing step.
2. Open the matching PNG in `frames/`.
3. Confirm what Claude saw matches what the engine rendered.
4. Check the outcome against `SPEC.md` section 5.
5. Reproduce with the same seed using `--dummy` before spending budget.

Don't add debug output that prints object positions into any file or response that Claude can read. Debug prints go to the engine's stderr, not into the agent's messages.

## 14. Definition of done for a task

A task is done when:
1. The change matches `SPEC.md`, or `SPEC.md` was updated in the same commit.
2. `pytest -q` passes.
3. If the agent or prompt changed, the leak test passes.
4. The change is committed with a correct message.
5. You report back: what changed, which tests ran, anything you couldn't verify.

Do not say a run works unless you ran it and read its log.

## 15. Working style

- Ask one short clarifying question rather than guessing when a decision affects judging or the contract.
- When unsure whether something leaks object data, treat it as leaking and ask.
- Keep reports short: what changed, what you verified, what's left.
