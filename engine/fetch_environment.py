"""Private PyBullet scene construction for the membrane fetch task."""
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
        self.floor_id = p.loadURDF("plane.urdf")
        self._make_walls()
        self._make_drop_zone()
        self._make_objects()
        self.robot_start = self._choose_robot_start()
        return self

    def _box(self, half_extents, position, colour, mass=0):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=colour)
        return p.createMultiBody(mass, col, vis, position)

    def _make_walls(self) -> None:
        h, t = self.ROOM_HALF, self.WALL_THICKNESS
        for pos, ext in [
            ([0, h, self.WALL_HEIGHT / 2], [h, t / 2, self.WALL_HEIGHT / 2]),
            ([0, -h, self.WALL_HEIGHT / 2], [h, t / 2, self.WALL_HEIGHT / 2]),
            ([h, 0, self.WALL_HEIGHT / 2], [t / 2, h, self.WALL_HEIGHT / 2]),
            ([-h, 0, self.WALL_HEIGHT / 2], [t / 2, h, self.WALL_HEIGHT / 2]),
        ]:
            self.wall_ids.append(self._box(ext, pos, [0.72, 0.74, 0.76, 1]))

    def _make_drop_zone(self) -> None:
        zone_limit = self.ROOM_HALF - 1.3
        self.drop_center = (self.rng.uniform(-zone_limit, zone_limit), self.rng.uniform(-zone_limit, zone_limit))
        # The marked area is visual-only: it must not act as a raised obstacle
        # for the robot or the delivered object.
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.DROP_HALF, self.DROP_HALF, 0.002],
                                     rgbaColor=config.DROP_ZONE_COLOR)
        self.drop_id = p.createMultiBody(0, -1, visual, [*self.drop_center, 0.002])

    def _make_objects(self) -> None:
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
