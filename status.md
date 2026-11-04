# Membrane Fetch Status

## Working

- The task reuses `scripts/diff_drive_robot.py` and `urdf/diff_drive_robot.urdf` with the existing wheel joints. The onboard rendered camera is 640×480 with 90° horizontal FOV, 0.5 m height, and 10° downward tilt.
- The engine owns the seeded room, object metadata, target resolution, collision checks, fixed-constraint pickup, delivery validation, and task outcome. The HTTP schemas expose robot status and action results; they do not expose object IDs, attributes, positions, counts, segmentation, or drop-zone coordinates.
- Pickup checks the 0.5 m front reach and ±30° heading cone, attaches with a PyBullet fixed constraint, filters collisions only between the held object and robot links, and restores collisions on release. The marked drop zone is visual-only. A drop is settled to floor height and validated internally without advancing physics outside move/turn actions.
- `run_fetch.py --gui` opens the same HTTP-controlled simulation in the PyBullet GUI. The one-key local physics demonstration is:

  ```powershell
  python scripts/visual_pickup_demo.py --seed 1234
  ```

  This scripted demonstration completed in the GUI: the existing robot approached, picked up with a fixed constraint, carried the object, released it in the marked zone, and the engine reported validation pass. It is a physics demonstration, not an AI vision run.
- The agent sends the simulated JPEG as image content, enforces one action tool call per decision, fetches a fresh frame after each move/turn, retains the latest image and recent action summaries, and reports success only when the engine ends with `pass` after an in-zone release.
- The engine internally resolves supported target criteria (including a unique warm-color sphere, tallest object, and blue box). An in-zone delivery only passes when the held body matches the resolved target; an unresolved or ambiguous instruction cannot pass accidentally.
- Each run writes JSONL action records, PNG frames, and `meta.json`. Unpriced usage is recorded as unknown rather than zero. Real model runs are capped at 120 unless `--force` is supplied; the soft 25-action limit warns, and the hard limit remains 60.

## Verification

- `python -m py_compile` on the modified modules passed.
- `python -m pytest -q`: 15 passed.
- After tightening target validation so an unresolved instruction cannot produce a false pass, `python -m pytest -q`: 15 passed.
- Seed 1234 full physics rehearsal passed: approach, fixed-constraint pickup, carry route, release, and internal in-zone validation.
- The visible GUI demo completed and was inspected after placement.
- Three natural-language target criteria resolve internally on the acceptance seed. The test does not claim the vision model completed those commands.
- No `ANTHROPIC_API_KEY` or `GROQ_API_KEY` was configured during this session, so no live Claude/Groq request or three-command model acceptance run was performed.

## Remaining PRD Work

- Run three live vision-model commands after a valid API key is configured; confirm pixel-based target selection in the resulting logs.
- The PRD obstacle stretch scenario and POV video/clip pipeline are not implemented or verified.
- Prompt-cache usage and priced per-run model cost are not verified; cost is logged as `null` until a pricing source is configured.

