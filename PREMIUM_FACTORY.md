# Premium Sci-Fi Robotics R&D Facility

## Overview

A fully functional premium autonomous robotics research and testing facility with industrial aesthetics, proper lighting, and factory obstacles.

## Features Implemented

### Floor
- **Dark graphite concrete base** (visible box, not invisible plane)
- **1m × 1m panel grid** with prominent seams
- **Yellow safety walkway markings** near all walls
- **Physically flat** - no obstacles beneath task objects

### Walls
- **Warm off-white modular panels**
- **Brushed metal top trims**
- **Charcoal base sections**
- **Vertical panel seam lines** every 2m
- 4 walls maintaining exact 6m × 6m room dimensions

### Ceiling
- **White modular panels**
- **Recessed cyan LED strips** along all perimeters
- **Structural cross beam** (charcoal)
- **Central light panel** for even illumination

### Drop Zone
- **Bright cyan perimeter border** (highly visible)
- **Dark graphite inner surface**
- **Crosshair precision markings**
- **Corner LED markers** (cyan dots)
- Validation logic preserved

### Factory Obstacles

#### Industrial Crates
- Placed in corners (avoids robot paths)
- Dark gray with yellow safety stripes
- Physics-enabled (can be pushed)

#### Charging Stations
- Located near east/west walls
- Dark pads with cyan indicators
- Visual markers for robot navigation

#### Safety Barriers
- Yellow posts near north/south walls
- Visual-only (won't block movement)

### Task Objects
- **Exactly 6 objects** preserved:
  - 2 boxes
  - 2 spheres
  - 1 cylinder
  - 1 cone
- Original shapes, colors, physics maintained
- Distinguishable colors: coral, teal, mustard, plum, sky blue, indigo

## Visual Design

**Color Palette:**
- Floor: RGB(51, 51, 56) - Deep graphite
- Walls: RGB(240, 237, 232) - Warm off-white
- Drop Zone Border: RGB(26, 230, 230) - Bright cyan
- Safety Markings: RGB(242, 191, 0) - Industrial amber
- LED Accents: RGB(0, 204, 204) - Cyan glow
- Metal Trims: RGB(128, 133, 138) - Brushed aluminium

**Style:** Premium industrial minimalism inspired by aerospace R&D facilities and high-end automotive engineering labs.

## Running the Demo

### Interactive GUI Demo
```bash
python demo_factory.py
```

Features:
- Rotating camera views
- Physics simulation
- Object list display
- Keyboard interrupt to exit

### Server Mode (for agent)
```bash
# Start the fetch server
python -m engine.fetch_server

# Or with GUI
FETCH_GUI=true python -m engine.fetch_server
```

### Test Suite
```bash
pytest tests/ -v
```

All 15 tests pass:
- ✓ Object pickup/drop mechanics
- ✓ Collision detection
- ✓ Drop zone validation
- ✓ Camera rendering
- ✓ Agent API schemas

## Files Modified

1. **`engine/fetch_environment.py`**
   - Complete redesign with visible floor
   - Multi-layer wall details
   - Detailed ceiling with LEDs
   - Factory obstacles system
   - Preserved: 6 objects, physics, API

2. **`engine/config.py`**
   - Premium color constants
   - Original colors preserved as fallback

3. **New:** `demo_factory.py`
   - Interactive visualization
   - Camera rotation
   - Object information display

## Preserved Systems

### Robot
- URDF unchanged
- Wheels, joints, wheelbase preserved
- Camera: 640×480, 90° FOV, 0.5m height, 10° tilt
- Movement controller intact

### Physics
- 6 task objects with exact collision geometry
- Drop zone validation logic
- Pickup/drop mechanics
- Wall collision detection

### API
- All HTTP endpoints working
- `/look`, `/move`, `/turn`, `/pickup`, `/drop`
- Agent integration preserved

## Agent Workflow

The premium environment is fully compatible with the autonomous fetch agent:

1. **Visual:** Agent receives 640×480 camera images with visible floor, walls, objects, drop zone
2. **Navigation:** Panel seams and safety lines provide visual reference
3. **Drop Zone:** Bright cyan border clearly visible from camera
4. **Obstacles:** Crates and barriers add realism without blocking core paths
5. **Objects:** All 6 task objects distinguishable by color and shape

## Troubleshooting

**Scene appears blank:**
- Ensure using `p.GUI` mode (not `p.DIRECT` with ER_TINY_RENDERER)
- Floor is now a visible box, not invisible plane
- Check camera distance (should be 7-8m for full view)

**Objects floating:**
- Physics needs 240 steps to settle
- Run `p.stepSimulation()` in loop

**Obstacles block robot:**
- Obstacles placed >1.5m from drop zone
- Corner crates avoid central navigation
- Charging stations are visual-only near walls

## Performance

- **Lightweight:** Procedural geometry, no textures
- **Fast:** <5 seconds to build environment
- **Stable:** 240Hz physics timestep
- **Compatible:** Works with existing tests and agent

## Future Enhancements

Potential additions (not currently implemented):
- Dynamic lighting (point lights at charging stations)
- Animated LED strips (pulsing effect)
- Additional equipment (robotic arms, conveyors - visual only)
- Sound effects (ambient factory hum)
- Fog/atmosphere effects

## License

Same as parent project. Premium environment uses only procedural PyBullet geometry.
