"""Private PyBullet scene construction for the membrane fetch task - Premium R&D Facility."""
from __future__ import annotations

import math
import random

import pybullet as p
import pybullet_data
from engine import config


class FetchEnvironment:
    """Own scene body IDs and goal validation data inside the engine."""

    ROOM_HALF = config.ROOM_SIZE / 2
    WALL_HEIGHT = config.WALL_HEIGHT
    WALL_THICKNESS = config.WALL_THICK
    DROP_HALF = config.DROP_ZONE_SIZE / 2
    OBJECT_COUNT = config.FETCH_OBJECT_COUNT

    def __init__(self, seed: int):
        self.rng = random.Random(seed)
        self.object_ids: list[int] = []
        self.object_meta: dict[int, dict] = {}
        self.wall_ids: list[int] = []
        self.drop_center: tuple[float, float] = (0.0, 0.0)
        self.drop_id: int | None = None
        self.floor_id: int | None = None
        self.robot_start: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self.target_body_id: int | None = None

    def build(self) -> "FetchEnvironment":
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1 / 240)
        self._create_detailed_floor()
        self._make_detailed_walls()
        self._make_detailed_ceiling()
        self._make_premium_drop_zone()
        self._make_factory_obstacles()
        self._make_objects()
        self.robot_start = self._choose_robot_start()
        return self

    def _create_detailed_floor(self) -> None:
        """Create premium industrial concrete floor with panel seams and safety markings."""
        # Create a large visible floor box instead of invisible plane
        floor_half = self.ROOM_HALF
        floor_thickness = 0.05
        floor_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[floor_half, floor_half, floor_thickness])
        floor_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[floor_half, floor_half, floor_thickness],
            rgbaColor=config.PREMIUM_FLOOR_COLOR
        )
        self.floor_id = p.createMultiBody(0, floor_col, floor_vis, [0, 0, -floor_thickness])
        
        # Panel seams - fine grid (1m spacing)
        seam_color = [0.15, 0.15, 0.17, 1.0]
        seam_thickness = 0.008
        seam_height = 0.008
        
        # Create 1m x 1m panel grid - prominent seams
        for x in range(-3, 4):
            line_x = x * 1.0
            if abs(line_x) < self.ROOM_HALF - 0.05:
                seam_vis = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[seam_thickness, self.ROOM_HALF, seam_height],
                    rgbaColor=seam_color
                )
                p.createMultiBody(0, -1, seam_vis, [line_x, 0, seam_height])
        
        for y in range(-3, 4):
            line_y = y * 1.0
            if abs(line_y) < self.ROOM_HALF - 0.05:
                seam_vis = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[self.ROOM_HALF, seam_thickness, seam_height],
                    rgbaColor=seam_color
                )
                p.createMultiBody(0, -1, seam_vis, [0, line_y, seam_height])
        
        # Safety walkway markings near walls (bright yellow lines)
        safety_yellow = config.SAFETY_YELLOW
        walkway_width = 0.10
        walkway_dist = 0.5
        walkway_height = 0.005
        
        # Walkway lines along walls
        for wall in ['north', 'south']:
            y_pos = (self.ROOM_HALF - walkway_dist) if wall == 'north' else (-self.ROOM_HALF + walkway_dist)
            walkway = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[self.ROOM_HALF - 0.6, walkway_width / 2, walkway_height],
                rgbaColor=safety_yellow
            )
            p.createMultiBody(0, -1, walkway, [0, y_pos, walkway_height])
        
        for wall in ['east', 'west']:
            x_pos = (self.ROOM_HALF - walkway_dist) if wall == 'east' else (-self.ROOM_HALF + walkway_dist)
            walkway = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[walkway_width / 2, self.ROOM_HALF - 0.6, walkway_height],
                rgbaColor=safety_yellow
            )
            p.createMultiBody(0, -1, walkway, [x_pos, 0, walkway_height])

    def _make_detailed_walls(self) -> None:
        """Create premium walls with layered panels, recessed sections, and metal trims."""
        h, t = self.ROOM_HALF, self.WALL_THICKNESS
        wall_color = config.PREMIUM_WALL_COLOR
        accent_color = config.WALL_STRUCTURAL_COLOR
        metal_color = config.METALLIC_ACCENT
        
        # Wall configurations: position, size, orientation
        walls = [
            ([0, h, 0.5], [h, t/2, 0.5], 'horizontal'),
            ([0, -h, 0.5], [h, t/2, 0.5], 'horizontal'),
            ([h, 0, 0.5], [t/2, h, 0.5], 'vertical'),
            ([-h, 0, 0.5], [t/2, h, 0.5], 'vertical'),
        ]
        
        for i, (pos, ext, orient) in enumerate(walls):
            # Main wall panel
            wall_id = self._box(ext, pos, wall_color)
            self.wall_ids.append(wall_id)
            
            # Top border/trim - brushed metal
            trim_height = 0.03
            if orient == 'horizontal':
                # Top metal trim
                trim = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[ext[0], trim_height/2, trim_height/2],
                    rgbaColor=metal_color
                )
                p.createMultiBody(0, -1, trim, [pos[0], pos[1], pos[2] + ext[2] - trim_height])
                
                # Bottom charcoal base
                base = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[ext[0], 0.02, 0.08],
                    rgbaColor=accent_color
                )
                p.createMultiBody(0, -1, base, [pos[0], pos[1], 0.04])
                
                # Vertical panel seam lines
                for seam_x in [-2, 0, 2]:
                    seam_pos = pos[0] + seam_x
                    if abs(seam_pos) < h - 0.1:
                        seam = p.createVisualShape(
                            p.GEOM_BOX,
                            halfExtents=[0.008, ext[1], 0.4],
                            rgbaColor=accent_color + [0.5]
                        )
                        p.createMultiBody(0, -1, seam, [seam_pos, pos[1], 0.45])
            else:
                # Vertical wall metal trim
                trim = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[trim_height/2, ext[1], trim_height/2],
                    rgbaColor=metal_color
                )
                p.createMultiBody(0, -1, trim, [pos[0], pos[1], pos[2] + ext[2] - trim_height])
                
                # Bottom charcoal base
                base = p.createVisualShape(
                    p.GEOM_BOX,
                    halfExtents=[0.02, ext[1], 0.08],
                    rgbaColor=accent_color
                )
                p.createMultiBody(0, -1, base, [pos[0], pos[1], 0.04])
                
                # Horizontal panel seam lines
                for seam_y in [-2, 0, 2]:
                    seam_pos = pos[1] + seam_y
                    if abs(seam_pos) < h - 0.1:
                        seam = p.createVisualShape(
                            p.GEOM_BOX,
                            halfExtents=[0.4, 0.008, 0.4],
                            rgbaColor=accent_color + [0.5]
                        )
                        p.createMultiBody(0, -1, seam, [pos[0], seam_pos, 0.45])

    def _make_detailed_ceiling(self) -> None:
        """Create detailed ceiling with panels, recessed LED strips and structural beams."""
        h = self.ROOM_HALF
        ceiling_height = self.WALL_HEIGHT
        led_color = config.LED_CYAN
        metal_color = config.METALLIC_ACCENT
        white_ceiling = [0.95, 0.95, 0.95, 1.0]
        
        # Ceiling panel (very thin, visual only)
        ceiling_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[h - 0.02, h - 0.02, 0.01],
            rgbaColor=white_ceiling
        )
        p.createMultiBody(0, -1, ceiling_vis, [0, 0, ceiling_height - 0.01])
        
        # Perimeter LED strips (recessed look)
        led_width = 0.04
        led_height = 0.02
        
        # North LED strip
        led_n = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[h - 0.3, led_width/2, led_height/2],
            rgbaColor=led_color
        )
        p.createMultiBody(0, -1, led_n, [0, h - 0.1, ceiling_height - 0.03])
        
        # South LED strip
        led_s = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[h - 0.3, led_width/2, led_height/2],
            rgbaColor=led_color
        )
        p.createMultiBody(0, -1, led_s, [0, -h + 0.1, ceiling_height - 0.03])
        
        # East LED strip
        led_e = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[led_width/2, h - 0.3, led_height/2],
            rgbaColor=led_color
        )
        p.createMultiBody(0, -1, led_e, [h - 0.1, 0, ceiling_height - 0.03])
        
        # West LED strip
        led_w = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[led_width/2, h - 0.3, led_height/2],
            rgbaColor=led_color
        )
        p.createMultiBody(0, -1, led_w, [-h + 0.1, 0, ceiling_height - 0.03])
        
        # Structural beams (charcoal)
        beam_width = 0.06
        beam_depth = 0.08
        beam_color = [0.20, 0.20, 0.20, 1.0]
        
        # Cross beams
        beam_h = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[h - 0.1, beam_width/2, beam_depth/2],
            rgbaColor=beam_color
        )
        p.createMultiBody(0, -1, beam_h, [0, 0, ceiling_height - 0.04])
        
        # Central light panel (recessed)
        panel_size = 2.5
        panel_color = [0.98, 0.98, 1.0, 0.9]
        panel = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[panel_size/2, panel_size/2, 0.005],
            rgbaColor=panel_color
        )
        p.createMultiBody(0, -1, panel, [0, 0, ceiling_height - 0.015])

    def _make_premium_drop_zone(self) -> None:
        """Create premium delivery station with dark graphite, cyan perimeter and crosshair."""
        zone_limit = self.ROOM_HALF - 1.3
        self.drop_center = (self.rng.uniform(-zone_limit, zone_limit), self.rng.uniform(-zone_limit, zone_limit))
        
        dz_half = self.DROP_HALF
        dz_thickness = 0.015
        
        # Outer cyan border (perimeter) - bright and visible
        border_color = [0.1, 0.9, 0.9, 1.0]
        border_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[dz_half, dz_half, dz_thickness],
            rgbaColor=border_color
        )
        self.drop_id = p.createMultiBody(0, -1, border_vis, [self.drop_center[0], self.drop_center[1], dz_thickness])
        
        # Inner dark graphite surface
        inner_color = [0.12, 0.12, 0.14, 1.0]
        inner_vis = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[dz_half - 0.05, dz_half - 0.05, dz_thickness - 0.005],
            rgbaColor=inner_color
        )
        p.createMultiBody(0, -1, inner_vis, [self.drop_center[0], self.drop_center[1], dz_thickness + 0.005])
        
        # Crosshair markings for precision
        line_color = [0.3, 0.3, 0.32, 1.0]
        line_width = 0.015
        line_height = 0.008
        
        # Horizontal line
        h_line = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[dz_half * 0.4, line_width, line_height],
            rgbaColor=line_color
        )
        p.createMultiBody(0, -1, h_line, [self.drop_center[0], self.drop_center[1], dz_thickness + 0.015])
        
        # Vertical line
        v_line = p.createVisualShape(
            p.GEOM_BOX,
            halfExtents=[line_width, dz_half * 0.4, line_height],
            rgbaColor=line_color
        )
        p.createMultiBody(0, -1, v_line, [self.drop_center[0], self.drop_center[1], dz_thickness + 0.015])
        
        # Corner markers (cyan LED-style dots)
        corner_size = 0.03
        corner_color = [0.1, 1.0, 1.0, 1.0]
        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            corner = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[corner_size, corner_size, 0.01],
                rgbaColor=corner_color
            )
            cx = self.drop_center[0] + dx * (dz_half - 0.1)
            cy = self.drop_center[1] + dy * (dz_half - 0.1)
            p.createMultiBody(0, -1, corner, [cx, cy, dz_thickness + 0.02])

    def _make_factory_obstacles(self) -> None:
        """Add sci-fi factory obstacles: crates, barriers, charging stations."""
        # Obstacle placement - avoid drop zone, robot start, and task objects
        obstacle_color = [0.25, 0.27, 0.30, 1.0]
        metal_color = config.METALLIC_ACCENT
        yellow_stripe = config.SAFETY_YELLOW
        
        # Industrial crates in corners
        crate_positions = [
            (2.3, 2.3),
            (-2.3, 2.3),
            (2.3, -2.3),
            (-2.3, -2.3)
        ]
        
        for cx, cy in crate_positions:
            # Check distance from drop zone and robot path
            if math.dist((cx, cy), self.drop_center) < 1.5:
                continue
            
            # Base crate
            crate_size = 0.3
            crate_height = 0.4
            crate_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[crate_size, crate_size, crate_height/2])
            crate_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[crate_size, crate_size, crate_height/2],
                rgbaColor=obstacle_color
            )
            p.createMultiBody(5, crate_col, crate_vis, [cx, cy, crate_height/2])
            
            # Yellow safety stripe on top
            stripe = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[crate_size, 0.03, 0.02],
                rgbaColor=yellow_stripe
            )
            p.createMultiBody(0, -1, stripe, [cx, cy, crate_height + 0.02])
        
        # Charging station markers near walls
        charge_positions = [
            (2.5, 0, 90),
            (-2.5, 0, 270),
        ]
        
        for cx, cy, angle in charge_positions:
            if math.dist((cx, cy), self.drop_center) < 1.2:
                continue
            
            # Charging pad
            pad_size = 0.4
            pad_height = 0.05
            pad_vis = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[pad_size/2, pad_size/2, pad_height/2],
                rgbaColor=[0.15, 0.15, 0.17, 1.0]
            )
            p.createMultiBody(0, -1, pad_vis, [cx, cy, pad_height/2])
            
            # Cyan indicator
            indicator = p.createVisualShape(
                p.GEOM_BOX,
                halfExtents=[0.15, 0.15, 0.01],
                rgbaColor=[0.0, 0.9, 0.9, 1.0]
            )
            p.createMultiBody(0, -1, indicator, [cx, cy, pad_height + 0.01])
        
        # Safety barriers (small, won't block robot)
        barrier_positions = [
            (0, 2.2, 0),
            (0, -2.2, 0)
        ]
        
        for bx, by, angle in barrier_positions:
            if math.dist((bx, by), self.drop_center) < 1.0:
                continue
            
            # Barrier post
            post_height = 0.6
            post_rad = 0.05
            post_vis = p.createVisualShape(
                p.GEOM_CYLINDER,
                radius=post_rad,
                length=post_height,
                rgbaColor=yellow_stripe
            )
            p.createMultiBody(0, -1, post_vis, [bx, by, post_height/2])

    def _box(self, half_extents, position, colour, mass=0):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=colour)
        return p.createMultiBody(mass, col, vis, position)

    def _make_objects(self) -> None:
        """Create exactly 6 task objects with primitive shapes and distinct colors."""
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
                if math.dist((x, y), self.drop_center) < 0.95:
                    continue
                if any(math.dist((x, y), other) < 0.8 for other in placed):
                    continue
                placed.append((x, y))
                break
            size = self.rng.uniform(config.FETCH_OBJECT_MIN_SIZE, config.FETCH_OBJECT_MAX_SIZE)
            height = size if shape == "sphere" else heights[index]
            colour_name = colour_names[index]
            colour = palette_by_name[colour_name]
            if shape == "box":
                col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size / 2, size / 2, height / 2])
                vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[size / 2, size / 2, height / 2], rgbaColor=colour)
            elif shape == "sphere":
                col = p.createCollisionShape(p.GEOM_SPHERE, radius=size / 2)
                vis = p.createVisualShape(p.GEOM_SPHERE, radius=size / 2, rgbaColor=colour)
            elif shape == "cylinder":
                col = p.createCollisionShape(p.GEOM_CYLINDER, radius=size / 2, height=height)
                vis = p.createVisualShape(p.GEOM_CYLINDER, radius=size / 2, length=height, rgbaColor=colour)
            else:
                vertices = [[0, 0, -height / 2], [-size / 2, -size / 2, -height / 2],
                            [size / 2, -size / 2, -height / 2], [size / 2, size / 2, -height / 2],
                            [-size / 2, size / 2, -height / 2], [0, 0, height / 2]]
                indices = [5, 1, 2, 5, 2, 3, 5, 3, 4, 5, 4, 1,
                           0, 4, 3, 0, 3, 2, 0, 2, 1, 0, 1, 4]
                col = p.createCollisionShape(p.GEOM_MESH, vertices=vertices, indices=indices)
                vis = p.createVisualShape(p.GEOM_MESH, vertices=vertices, indices=indices, rgbaColor=colour)
            body = p.createMultiBody(0.18, col, vis, [x, y, height / 2])
            p.changeDynamics(body, -1, lateralFriction=0.8, restitution=0.0)
            self.object_ids.append(body)
            self.object_meta[body] = {"shape": shape, "position": (x, y), "colour": colour,
                                      "colour_name": colour_name, "height": height, "size": size}

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
            start_limit = self.ROOM_HALF - config.MIN_ROBOT_WALL_DIST
            x, y = self.rng.uniform(-start_limit, start_limit), self.rng.uniform(-start_limit, start_limit)
            if math.dist((x, y), self.drop_center) < config.MIN_ROBOT_DROP_DIST:
                continue
            if any(math.dist((x, y), self.object_meta[i]["position"]) < config.MIN_OBJ_ROBOT_DIST for i in self.object_ids):
                continue
            return x, y, self.rng.uniform(-math.pi, math.pi)
        return 0.0, 0.0, 0.0

    def is_in_drop_zone(self, x: float, y: float) -> bool:
        return (abs(x - self.drop_center[0]) <= self.DROP_HALF and
                abs(y - self.drop_center[1]) <= self.DROP_HALF)

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
            start_limit = self.ROOM_HALF - config.MIN_ROBOT_WALL_DIST
            x, y = self.rng.uniform(-start_limit, start_limit), self.rng.uniform(-start_limit, start_limit)
            if math.dist((x, y), self.drop_center) < config.MIN_ROBOT_DROP_DIST:
                continue
            if any(math.dist((x, y), self.object_meta[i]["position"]) < config.MIN_OBJ_ROBOT_DIST for i in self.object_ids):
                continue
            return x, y, self.rng.uniform(-math.pi, math.pi)
        return 0.0, 0.0, 0.0

    def is_in_drop_zone(self, x: float, y: float) -> bool:
        return (abs(x - self.drop_center[0]) <= self.DROP_HALF and
                abs(y - self.drop_center[1]) <= self.DROP_HALF)
