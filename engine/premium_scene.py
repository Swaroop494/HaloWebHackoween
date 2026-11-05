"""Premium PyBullet scene for industrial robotics laboratory."""
from __future__ import annotations

import pybullet as p
import pybullet_data
from engine import config
from engine.premium_environment import PremiumEnvironment


class PremiumScene:
    """Premium scene with industrial laboratory aesthetic."""

    def __init__(self, seed: int, use_premium: bool = True):
        self.seed = seed
        self.use_premium = use_premium
        self.environment: PremiumEnvironment | None = None
        self.original_floor_id: int | None = None
        self.original_wall_ids: list[int] = []
        self.original_drop_id: int | None = None

    def build(self) -> "PremiumScene":
        """Build the premium scene."""
        if self.use_premium:
            self.environment = PremiumEnvironment(self.seed).build()
        else:
            # Use original plane
            self.original_floor_id = p.loadURDF("plane.urdf")
        return self

    def get_environment(self) -> PremiumEnvironment | None:
        """Return the premium environment if built."""
        return self.environment

    def is_in_drop_zone(self, x: float, y: float) -> bool:
        """Check if position is in the drop zone."""
        if self.environment:
            return self.environment.is_in_drop_zone(x, y)
        return False

    def select_target(self, command: str) -> int | None:
        """Select target object based on command."""
        if self.environment:
            return self.environment.select_target(command)
        return None

    def get_lighting_positions(self) -> list[tuple[float, float, float]]:
        """Get positions of lighting elements."""
        if self.environment:
            return self.environment.get_lighting_positions()
        return []

    def cleanup(self) -> None:
        """Clean up the scene."""
        if self.environment:
            # Clean up environment objects
            for body_id in self.environment.object_ids:
                try:
                    p.removeBody(body_id)
                except Exception:
                    pass
            self.environment = None
        if self.original_floor_id is not None:
            try:
                p.removeBody(self.original_floor_id)
            except Exception:
                pass
            self.original_floor_id = None
