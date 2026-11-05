"""Premium PyBullet scene with industrial robotics laboratory aesthetic."""
from __future__ import annotations

import math
import random

import pybullet as p
import pybullet_data
from engine import config


class PremiumEnvironment:
    """Industrial robotics laboratory environment with KenneyNL assets."""

    ROOM_HALF = config.ROOM_SIZE / 2
    WALL_HEIGHT = config.WALL_HEIGHT
    WALL_THICKNESS = config.WALL_THICK
    # The premium demonstration delivers all six objects into separated slots.
    DROP_HALF = config.PREMIUM_DROP_ZONE_SIZE / 2
    OBJECT_COUNT = config.FETCH_OBJECT_COUNT

    def __init__(self, seed: int):
        self.rng = random.Random(seed)
        self.object_ids: list[int] = []
        self.object_meta: dict[int, dict] = {}
        self.wall_ids: list[int] = []
        self.floor_grid_id: int | None = None
        self.drop_center: tuple[float, float] = (0.0, 0.0)
        self.drop_id: int | None = None
        self.robot_start: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.target_body_id: int | None = None
        self.lighting_ids: list[int] = []
        self.safety_markings: list[int] = []
        self.wall_panel_ids: list[int] = []
        self.floor_id: int | None = None  # Initialize before use

    def build(self) -> "PremiumEnvironment":
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        self._create_premium_floor()
        self._make_industrial_walls()
        self._make_drop_zone_with_lighting()
        self._make_safety_markings()
        self._create_ceiling_lighting()
        self._make_objects()
        self.robot_start = self._choose_robot_start()
        return self

    def _create_premium_floor(self) -> None:
        """Create premium industrial floor with subtle grid pattern."""
        # Use a thin box for a reliable visible and collidable floor in GUI and DIRECT.
        half = self.ROOM_HALF
        floor_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half, half, 0.02])
        floor_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, half, 0.02],
                                        rgbaColor=config.PREMIUM_FLOOR_COLOR)
        self.floor_id = p.createMultiBody(0, floor_col, floor_vis, [0, 0, -0.02])

        # Sparse orthogonal lines give a deliberate industrial grid without moiré.
        horizontal = p.createVisualShape(p.GEOM_BOX, halfExtents=[half, 0.006, 0.0008],
                                         rgbaColor=config.FLOOR_GRID_COLOR)
        vertical = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.006, half, 0.0008],
                                       rgbaColor=config.FLOOR_GRID_COLOR)
        for offset in range(-int(half), int(half) + 1):
            p.createMultiBody(0, -1, horizontal, [0, offset, 0.001])
            p.createMultiBody(0, -1, vertical, [offset, 0, 0.001])

    def _make_industrial_walls(self) -> None:
        """Create modular industrial wall panels with structural detailing."""
        h, t = self.ROOM_HALF, self.WALL_THICKNESS

        # Main wall panels
        for pos, ext in [
            ([0, h, self.WALL_HEIGHT / 2], [h, t / 2, self.WALL_HEIGHT / 2]),
            ([0, -h, self.WALL_HEIGHT / 2], [h, t / 2, self.WALL_HEIGHT / 2]),
            ([h, 0, self.WALL_HEIGHT / 2], [t / 2, h, self.WALL_HEIGHT / 2]),
            ([-h, 0, self.WALL_HEIGHT / 2], [t / 2, h, self.WALL_HEIGHT / 2]),
        ]:
            wall_id = p.createMultiBody(
                0,
                p.createCollisionShape(p.GEOM_BOX, halfExtents=ext),
                p.createVisualShape(p.GEOM_BOX, halfExtents=ext, rgbaColor=config.PREMIUM_WALL_COLOR),
                pos
            )
            self.wall_ids.append(wall_id)
            self.wall_panel_ids.append(wall_id)

            # Slim upright ribs sit flush on the room-facing wall surface.
            for offset in (-h / 3, 0.0, h / 3):
                if pos[1] == 0:  # East/west walls run along Y.
                    rib_vis = p.createVisualShape(
                        p.GEOM_BOX, halfExtents=[0.018, 0.035, ext[2] - 0.015],
                        rgbaColor=config.WALL_STRUCTURAL_COLOR)
                    x_face = math.copysign(h - 0.018, pos[0])
                    p.createMultiBody(0, -1, rib_vis, [x_face, offset, pos[2]])
                else:  # North/south walls run along X.
                    rib_vis = p.createVisualShape(
                        p.GEOM_BOX, halfExtents=[0.035, 0.018, ext[2] - 0.015],
                        rgbaColor=config.WALL_STRUCTURAL_COLOR)
                    y_face = math.copysign(h - 0.018, pos[1])
                    p.createMultiBody(0, -1, rib_vis, [offset, y_face, pos[2]])

    def _make_drop_zone_with_lighting(self) -> None:
        """Create illuminated drop zone with warning markings."""
        zone_limit = self.ROOM_HALF - self.DROP_HALF - 0.65
        self.drop_center = (
            self.rng.uniform(-zone_limit, zone_limit),
            self.rng.uniform(-zone_limit, zone_limit)
        )

        # Drop zone base (visual only)
        base_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.DROP_HALF, self.DROP_HALF, 0.002],
            rgbaColor=config.PREMIUM_DROP_ZONE_COLOR
        )
        self.drop_id = p.createMultiBody(0, -1, base_vis, [*self.drop_center, 0.002])

        # Illuminated border
        border_half = self.DROP_HALF + 0.05
        border_x = p.createVisualShape(p.GEOM_BOX, halfExtents=[border_half, 0.025, 0.004],
                                       rgbaColor=config.SAFETY_YELLOW)
        border_y = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.025, border_half, 0.004],
                                       rgbaColor=config.SAFETY_YELLOW)
        for sign in (-1, 1):
            p.createMultiBody(0, -1, border_x,
                              [self.drop_center[0], self.drop_center[1] + sign * border_half, 0.004])
            p.createMultiBody(0, -1, border_y,
                              [self.drop_center[0] + sign * border_half, self.drop_center[1], 0.004])

        # Center indicator light
        center_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.3, 0.3, 0.002],
            rgbaColor=config.PREMIUM_DROP_ZONE_INDICATOR_COLOR
        )
        p.createMultiBody(0, -1, center_vis, [*self.drop_center, 0.004])

    def _make_safety_markings(self) -> None:
        """Add yellow/black safety markings."""
        # Caution strips on floor near walls
        strip_length = 1.0
        strip_half = strip_length / 2
        strip_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[strip_half, 0.02, 0.001],
            rgbaColor=config.SAFETY_YELLOW
        )
        black_strip = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[strip_half, 0.02, 0.001],
            rgbaColor=config.SAFETY_BLACK
        )
        side_strip_vis = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.02, strip_half, 0.001],
            rgbaColor=config.SAFETY_YELLOW)
        side_black_strip = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.02, strip_half, 0.001],
            rgbaColor=config.SAFETY_BLACK)

        # Place at 2m intervals along walls
        interval = 2.0
        for side in range(4):
            for pos in range(-int(self.ROOM_HALF) + 2, int(self.ROOM_HALF) - 1, int(interval)):
                if side == 0:  # North wall
                    p.createMultiBody(0, -1, strip_vis, [pos, self.ROOM_HALF - 0.2, 0.001])
                    p.createMultiBody(0, -1, black_strip, [pos, self.ROOM_HALF - 0.22, 0.001])
                elif side == 1:  # South wall
                    p.createMultiBody(0, -1, strip_vis, [pos, -self.ROOM_HALF + 0.2, 0.001])
                    p.createMultiBody(0, -1, black_strip, [pos, -self.ROOM_HALF + 0.22, 0.001])
                elif side == 2:  # East wall
                    p.createMultiBody(0, -1, side_strip_vis, [self.ROOM_HALF - 0.2, pos, 0.001])
                    p.createMultiBody(0, -1, side_black_strip, [self.ROOM_HALF - 0.22, pos, 0.001])
                else:  # West wall
                    p.createMultiBody(0, -1, side_strip_vis, [-self.ROOM_HALF + 0.2, pos, 0.001])
                    p.createMultiBody(0, -1, side_black_strip, [-self.ROOM_HALF + 0.22, pos, 0.001])

    def _create_ceiling_lighting(self) -> None:
        """Create LED strip lighting for premium illumination."""
        # Ceiling mounted LED strips along walls
        strip_y = self.WALL_HEIGHT - 0.1
        strip_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[self.ROOM_HALF - 0.1, 0.02, 0.02],
            rgbaColor=config.LED_CYAN
        )

        for pos_y in [self.ROOM_HALF - 0.1, -self.ROOM_HALF + 0.1]:
            strip_id = p.createMultiBody(0, -1, strip_vis, [0, pos_y, strip_y])
            self.lighting_ids.append(strip_id)

        # Additional cross strips for even lighting
        strip_x = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[0.02, self.ROOM_HALF - 0.1, 0.02],
            rgbaColor=config.LED_CYAN
        )
        for pos_x in [self.ROOM_HALF / 2, -self.ROOM_HALF / 2]:
            strip_id = p.createMultiBody(0, -1, strip_x, [pos_x, 0, strip_y])
            self.lighting_ids.append(strip_id)

    def _make_objects(self) -> None:
        """Create premium task objects with improved visual quality."""
        types = config.FETCH_SHAPES
        palette_by_name = {name: rgba for rgba, name in config.PALETTE}
        coral_sphere = self.rng.choice((1, 5))
        blue_cube = self.rng.choice((0, 4))
        remaining_indices = [i for i in range(len(types)) if i not in (coral_sphere, blue_cube)]
        remaining_palette = [name for name in palette_by_name
                             if name not in ("coral", "sky blue")]
        remaining_colours = self.rng.sample(remaining_palette, 3)
        remaining_colours.append(self.rng.choice(remaining_colours))
        self.rng.shuffle(remaining_colours)
        colour_names = [None] * len(types)
        colour_names[coral_sphere] = "coral"
        colour_names[blue_cube] = "sky blue"
        for index, name in zip(remaining_indices, remaining_colours):
            colour_names[index] = name
        heights = (0.22, 0.28, 0.34, 0.45, 0.55, 0.31)
        placed: list[tuple[float, float]] = []
        for index, shape in enumerate(types):
            for _ in range(2000):
                object_limit = self.ROOM_HALF - 0.65
                x, y = (self.rng.uniform(-object_limit, object_limit), self.rng.uniform(-object_limit, object_limit))
                # Keep the marked staging area clear so the six final slots
                # remain visually distinct and physically reachable.
                if (abs(x - self.drop_center[0]) <= self.DROP_HALF + 0.35 and
                        abs(y - self.drop_center[1]) <= self.DROP_HALF + 0.35):
                    continue
                if any(math.dist((x, y), other) < 0.8 for other in placed):
                    continue
                placed.append((x, y))
                break
            size = self.rng.uniform(config.FETCH_OBJECT_MIN_SIZE, config.FETCH_OBJECT_MAX_SIZE)
            height = size if shape == "sphere" else heights[index]
            colour_name = colour_names[index]
            colour = palette_by_name[colour_name]

            # Use mesh approximation for better visual quality
            if shape == "box":
                # Smoother edges for box
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size / 2, size / 2, height / 2])
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[size / 2, size / 2, height / 2],
                                         rgbaColor=colour, specularColor=[0.3, 0.3, 0.3])
            elif shape == "sphere":
                col = p.createCollisionShape(p.GEOM_SPHERE, radius=size / 2)
                vis = p.createVisualShape(p.GEOM_SPHERE, radius=size / 2, rgbaColor=colour,
                                         specularColor=[0.5, 0.5, 0.5])
            elif shape == "cylinder":
                col = p.createCollisionShape(p.GEOM_CYLINDER, radius=size / 2, height=height)
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=size / 2, length=height, rgbaColor=colour,
                                         specularColor=[0.4, 0.4, 0.4])
            else:  # cone
                vertices = [[0, 0, -height / 2], [-size / 2, -size / 2, -height / 2],
                           [size / 2, -size / 2, -height / 2], [size / 2, size / 2, -height / 2],
                           [-size / 2, size / 2, -height / 2], [0, 0, height / 2]]
                indices = [5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 1,
                          0, 4, 3, 0, 3, 2, 0, 2, 1, 0, 1, 4]
                # Dynamic concave triangle meshes can resist fixed-constraint
                # carrying even when they are convex (as this cone is). Let
                # PyBullet build the collision hull from the same vertices;
                # retain the indexed mesh only for the rendered appearance.
                col = p.createCollisionShape(p.GEOM_MESH, vertices=vertices)
                vis = p.createVisualShape(p.GEOM_MESH, vertices=vertices, indices=indices, rgbaColor=colour,
                                         specularColor=[0.4, 0.4, 0.4])

            body = p.createMultiBody(0.18, col, vis, [x, y, height / 2])
            p.changeDynamics(body, -1, lateralFriction=0.8, restitution=0.0)
            self.object_ids.append(body)
            self.object_meta[body] = {
                "shape": shape,
                "position": (x, y),
                "colour": colour,
                "colour_name": colour_name,
                "height": height,
                "size": size
            }

    def select_target(self, command: str) -> int | None:
        """Resolve simple, unique task attributes internally; return no scene data."""
        text = (command or "").lower()
        candidates = list(self.object_ids)
        colour_aliases = {
            "red": "coral", "coral": "coral", "teal": "teal", "mustard": "mustard",
            "plum": "plum", "purple": "plum", "sky blue": "sky blue",
            "light blue": "sky blue", "blue": "blue", "indigo": "indigo",
        }
        requested_colour = None
        for phrase, mapped in sorted(colour_aliases.items(), key=lambda entry: -len(entry[0])):
            if phrase in text:
                requested_colour = mapped
                break
        if requested_colour == "blue":
            candidates = [body for body in candidates
                          if self.object_meta[body]["colour_name"] == "sky blue"]
        elif requested_colour is not None:
            candidates = [body for body in candidates
                          if self.object_meta[body]["colour_name"] == requested_colour]
        shape_aliases = {"cube": "box", "box": "box", "sphere": "sphere", "round": "sphere",
                        "ball": "sphere", "cylinder": "cylinder", "cone": "cone"}
        requested_shape = next((shape for phrase, shape in shape_aliases.items() if phrase in text), None)
        if requested_shape:
            candidates = [body for body in candidates if self.object_meta[body]["shape"] == requested_shape]
        if "tallest" in text or "highest" in text:
            candidates = sorted(candidates, key=lambda body: self.object_meta[body]["height"], reverse=True)[:1]
        elif "largest" in text or "biggest" in text:
            candidates = sorted(candidates, key=lambda body: self.object_meta[body]["size"], reverse=True)[:1]
        return candidates[0] if len(candidates) == 1 else None

    def _choose_robot_start(self) -> tuple[float, float, float]:
        for _ in range(4000):
            start_limit = self.ROOM_HALF - 0.45
            x, y = self.rng.uniform(-start_limit, start_limit), self.rng.uniform(-start_limit, start_limit)
            if math.dist((x, y), self.drop_center) < config.MIN_ROBOT_DROP_DIST:
                continue
            if (abs(x - self.drop_center[0]) <= self.DROP_HALF + 0.4 and
                    abs(y - self.drop_center[1]) <= self.DROP_HALF + 0.4):
                continue
            if any(math.dist((x, y), self.object_meta[i]["position"]) < config.MIN_OBJ_ROBOT_DIST for i in self.object_ids):
                continue
            return x, y, self.rng.uniform(-math.pi, math.pi)
        return 0.0, 0.0, 0.0

    def is_in_drop_zone(self, x: float, y: float) -> bool:
        return (abs(x - self.drop_center[0]) <= self.DROP_HALF and
                abs(y - self.drop_center[1]) <= self.DROP_HALF)

    def get_lighting_positions(self) -> list[tuple[float, float, float]]:
        """Return positions for additional dynamic lighting."""
        positions = []
        for light_id in self.lighting_ids:
            pos, _ = p.getBasePositionAndOrientation(light_id)
            positions.append(pos)
        return positions
