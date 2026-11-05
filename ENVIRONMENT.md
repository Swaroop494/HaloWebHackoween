# PyBullet Environment Documentation

This document describes the PyBullet simulation environments for the autonomous robotics research facility.

## Overview

The system provides **two environments**:

1. **Original Environment** (`engine/server.py`) - Basic physics simulation
2. **Premium Environment** (`engine/premium_server.py`) - Industrial robotics lab aesthetic

Both environments share the same API and support the Membrane Fetch task.

## Environments

### Original Environment

A basic physics simulation suitable for development and testing.

**Start the server:**
```bash
python -m engine.server
```

**API Endpoint:** http://localhost:8080

**Characteristics:**
- Plain plane floor
- Single-color walls
- Basic object rendering
- Default PyBullet lighting

### Premium Environment

A premium industrial robotics laboratory with:

- Graphite floor with subtle grid
- Off-white modular wall panels
- Cyan LED lighting accents
- Yellow/black safety markings
- Illuminated drop zone
- Specular highlights on objects

**Start the server:**
```bash
python -m engine.premium_server
```

**API Endpoint:** http://localhost:8081

**Start with GUI:**
```bash
PREMIUM_GUI=true python -m engine.premium_server
```

## Quick Start

### Using the Run Script

The `run_premium.py` script starts both servers:

```bash
python run_premium.py
```

This starts:
- Original server on port 8080
- Premium server on port 8081

Both servers can be controlled with `--gui` flag.

### Manual Start

**Original:**
```bash
python -m engine.server  # or --gui
```

**Premium:**
```bash
PREMIUM_GUI=true python -m engine.premium_server
```

## API Endpoints

Both environments expose identical endpoints:

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/health` | GET | Check server status |
| `/state` | GET | Get robot state |
| `/look` | GET | Get camera image |
| `/depth` | GET | Get distance to obstacle |
| `/move` | POST | Move robot forward/backward |
| `/turn` | POST | Rotate robot |
| `/pickup` | POST | Pick up object |
| `/drop` | POST | Drop held object |
| `/run/start` | POST | Start a new run |
| `/run/end` | POST | End current run |

### Example API Calls

**Health Check:**
```bash
curl http://localhost:8080/health
```

**Get State:**
```bash
curl http://localhost:8080/state
```

**Move Robot:**
```bash
curl -X POST http://localhost:8080/move -d '{"distance": 1.0}'
```

**Turn Robot:**
```bash
curl -X POST http://localhost:8080/turn -d '{"degrees": 90}'
```

**Get Camera Image:**
```bash
curl http://localhost:8080/look --output camera.jpg
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `FETCH_SEED` | `1234` | Random seed for scene generation |
| `FETCH_GUI` | `false` | Show PyBullet GUI for original |
| `PREMIUM_SEED` | `1234` | Random seed for premium scene |
| `PREMIUM_GUI` | `false` | Show PyBullet GUI for premium |
| `FETCH_COMMAND` | `""` | Task command for target selection |
| `FETCH_MENTOR_APPROVED_DEPTH` | `false` | Enable depth sensor |

### Configuration Constants

Edit `engine/config.py` to customize:

```python
ROOM_SIZE = 6.0              # Room dimensions
WALL_HEIGHT = 1.0            # Wall height
FETCH_OBJECT_COUNT = 6       # Number of task objects
FETCH_OBJECT_MIN_SIZE = 0.22 # Minimum object size
FETCH_OBJECT_MAX_SIZE = 0.34 # Maximum object size
FETCH_MOVE_SPEED = 0.5       # Movement speed
TURN_SPEED_DEG = 90.0        # Turn speed
```

## Scene Generation

### Drop Zone

- Randomly placed within safe area
- Illuminated in premium environment
- Yellow border and cyan indicator

### Task Objects

- 4-6 objects per scene
- Shapes: box, sphere, cylinder, cone
- Colors from palette (coral, teal, mustard, plum, sky blue, indigo)
- Minimum spacing constraints

### Robot Start Position

- Far from drop zone (minimum distance)
- Far from objects (minimum distance)
- Far from walls (minimum distance)
- Random heading

## Asset Sources

### KenneyNL Assets

Downloadable from https://kenney.nl/assets:

| Pack | Category | Usage |
|------|----------|-------|
| Factory Kit | Industrial | Machinery, equipment |
| Industrial Pack | Industrial | Structures, barriers |
| Interior Pack | Interior | Lighting, fixtures |
| Architectural Pack | Architectural | Wall details |

### Asset Formats

- **OBJ**: Best for PyBullet compatibility
- **STL**: Alternative for collision meshes
- **FBX**: For advanced engines (requires conversion)
- **GLB**: Modern format (requires conversion)

### Asset Integration

Use the Kenney downloader:
```bash
python assets/kenney_downloader.py download 1
```

## Performance

### Optimization Tips

1. **Use Primitive Shapes**: Collision geometry should be simple
2. **Share Shapes**: Reuse collision/visual shapes
3. **Limit Objects**: Keep mesh count under 100
4. **Disable Collisions**: Use collision groups
5. **Optimize Rendering**: Reduce camera resolution

### Troubleshooting

**Slow Simulation:**
- Reduce camera resolution
- Use primitive shapes for collision
- Disable unnecessary visual elements

**Rendering Issues:**
- Check mesh file paths
- Verify texture files
- Ensure proper scale (meters)

**API Errors:**
- Check server is running
- Verify port number
- Ensure simulation is initialized

## Development

### Adding New Assets

1. Download from KenneyNL
2. Convert to OBJ/STL if needed
3. Place in `assets/` directory
4. Update `premium_environment.py` to use

### Testing

Run tests:
```bash
pytest -q
```

### Logs

Logs are saved to `logs/run_*/`:
- `log.jsonl`: Step-by-step agent actions
- `frames/step_XXX.png`: Camera images per step

## License

This project uses KenneyNL assets under CC0 1.0 Universal.
