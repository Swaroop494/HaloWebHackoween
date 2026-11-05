# Premium Robotics Facility Environment

This document describes the premium PyBullet environment upgrade for the autonomous robotics research facility.

## Overview

The premium environment transforms the basic physics demo into a clean, premium industrial robotics laboratory with:

- **Graphite floor** with subtle grid pattern
- **Off-white walls** with modular panel design and charcoal structural ribs
- **Cyan/teal lighting accents** via LED strips
- **Yellow/black safety markings** throughout the facility
- **Illuminated drop zone** with border and indicator light
- **Premium object rendering** with specular highlights

## Art Direction

### Palette

- **Floor**: Graphite (#38383D) with subtle grid lines
- **Walls**: Off-white (#EDEDED) with charcoal structural detailing (#2E2E2E)
- **Drop Zone**: Cyan/teal (#279E94) with yellow border (#D7A909)
- **Lighting**: Cyan LED accents (#00D9D9 at 60% opacity)
- **Safety**: Yellow/black caution strips

### Environment Details

- **Subtle floor tiles/grid**: 1m spacing with fine grid lines
- **Modular wall panels**: Vertical structural ribs at 1/3 intervals
- **Ceiling LED strips**: Along walls and cross strips for even lighting
- **Safety markings**: Yellow/black caution strips every 2m along walls
- **Illuminated drop zone**: Border with cyan indicator light
- **Polished materials**: Objects have specular highlights for premium feel

## Assets

### KenneyNL Asset Sources

The environment uses lightweight assets from KenneyNL:

| Asset Pack | Category | Formats | Usage |
|------------|----------|---------|-------|
| Factory Kit | Industrial | OBJ, FBX, GLB | Machinery, equipment |
| Industrial Pack | Industrial | OBJ, FBX, GLB | Structures, barriers |
| Interior Pack | Interior | OBJ, FBX, GLB | Lighting, fixtures |
| Architectural Pack | Architectural | OBJ, FBX, GLB | Wall details, panels |

### Asset Usage Guidelines

1. **PyBullet Compatibility**: Use OBJ or STL formats
2. **Lightweight Meshes**: Keep triangle count low
3. **Separate Collision**: Use simplified collision geometry
4. **Texture Support**: KenneyNL assets include textures; verify MTL references
5. **Scale**: All dimensions in meters; PyBullet expects SI units

### Asset Integration

```python
# Load KenneyNL OBJ asset
collision_id = p.createCollisionShape(p.GEOM_MESH, file_name="asset.obj")
visual_id = p.createVisualShape(p.GEOM_MESH, file_name="asset.obj", rgbaColor=[1, 1, 1, 1])

body_id = p.createMultiBody(mass, collision_id, visual_id, position)
```

## Performance Considerations

### Optimization Strategies

1. **Mesh Reuse**: Share collision and visual shapes where possible
2. **Simple Geometry**: Use primitive shapes for collision, mesh for visual
3. **Low Polygon Count**: Keep visual meshes under 1000 triangles
4. **Batch Rendering**: Group similar objects for efficient rendering
5. **Collision Filtering**: Use collision groups to reduce pairwise checks

### Memory Management

- Reuse collision shapes for identical objects
- Keep mesh count under 100 for real-time performance
- Use `p.changeVisualShape` for dynamic appearance changes
- Clean up unused bodies with `p.removeBody`

## Usage

### Basic Usage

```python
from engine.premium_environment import PremiumEnvironment

# Create premium environment
env = PremiumEnvironment(seed=1234).build()

# Access environment data
print(f"Drop zone: {env.drop_center}")
print(f"Objects: {len(env.object_ids)}")

# Check if position is in drop zone
if env.is_in_drop_zone(x, y):
    print("In drop zone")
```

### With Server

The premium environment can be used with the HTTP server:

```bash
# Run premium server on port 8081
python -m engine.premium_server

# Or with GUI
PREMIUM_GUI=true python -m engine.premium_server
```

### Comparison with Original

| Feature | Original | Premium |
|---------|----------|---------|
| Floor | Plain plane | Graphite grid |
| Walls | Single color | Modular panels |
| Lighting | Default | LED strips |
| Drop Zone | Simple | Illuminated |
| Objects | Basic | Specular highlights |
| Safety | None | Markings |

## API

### PremiumEnvironment

```python
class PremiumEnvironment:
    def __init__(self, seed: int)
    def build(self) -> "PremiumEnvironment"
    def select_target(self, command: str) -> int | None
    def is_in_drop_zone(self, x: float, y: float) -> bool
    def get_lighting_positions(self) -> list[tuple[float, float, float]]
```

### Config Changes

New configuration constants:

```python
PREMIUM_FLOOR_COLOR      # Graphite floor
FLOOR_GRID_COLOR         # Subtle grid lines
PREMIUM_WALL_COLOR       # Off-white walls
WALL_STRUCTURAL_COLOR    # Charcoal ribs
DROP_ZONE_COLOR          # Cyan drop zone
DROP_ZONE_BORDER         # Yellow safety border
DROP_ZONE_INDICATOR_COLOR  # Cyan indicator
LED_CYAN                 # Cyan LED accent
SAFETY_YELLOW            # Yellow safety
SAFETY_BLACK             # Black safety
METALLIC_ACCENT          # Metallic surfaces
```

## Migration Guide

### From Original to Premium

1. **Update import**:
   ```python
   from engine.premium_environment import PremiumEnvironment
   ```

2. **Replace environment initialization**:
   ```python
   # Original
   env = OriginalEnvironment(seed).build()
   
   # Premium
   env = PremiumEnvironment(seed).build()
   ```

3. **Update colors** (optional):
   ```python
   config.PREMIUM_FLOOR_COLOR  # Instead of FLOOR_COLOR
   ```

### Backwards Compatibility

The original `fetch_environment.py` remains unchanged for fallback use. Both environments share the same API and configuration.

## Future Enhancements

Potential upgrades:

1. **KenneyNL Asset Integration**: Download and integrate real 3D models
2. **Dynamic Lighting**: Add point lights and shadows
3. **Sound Effects**: Industrial soundscapes
4. **Post-Processing**: Screen space effects for camera
5. **Weather**: Subtle atmospheric effects

## License

This premium environment uses KenneyNL assets under the [CC0 1.0 Universal](https://creativecommons.org/publicdomain/zero/1.0/) license.
