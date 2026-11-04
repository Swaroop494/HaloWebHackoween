"""Physics-backed pickup, carry and drop validation owned by the engine."""
from __future__ import annotations

import math
import pybullet as p
from engine import config


class PickupManager:
    def __init__(self, environment, robot, reach: float = config.PICKUP_REACH,
                 angle_deg: float = config.PICKUP_ANGLE_DEG):
        self.environment = environment
        self.robot = robot
        self.reach = float(reach)
        self.angle = math.radians(float(angle_deg))
        self.constraint_id: int | None = None
        self.held_body: int | None = None
        self._saved_dynamics: dict | None = None
        self._filtered_links: list[int] = []

    def _candidate_distance(self, body: int, front_x: float, front_y: float) -> tuple[float, float, float]:
        """Return surface range, heading-relative bearing, and center range."""
        pos, _ = p.getBasePositionAndOrientation(body)
        dx, dy = pos[0] - front_x, pos[1] - front_y
        center_range = math.hypot(dx, dy)
        bearing = math.atan2(math.sin(math.atan2(dy, dx) - self.robot.get_pose()[2]),
                             math.cos(math.atan2(dy, dx) - self.robot.get_pose()[2]))
        # PyBullet's body AABB gives shape-independent horizontal extents for
        # box, sphere, cylinder, and mesh objects.
        lower, upper = p.getAABB(body, -1)
        radius = max(upper[0] - lower[0], upper[1] - lower[1]) / 2
        return max(0.0, center_range - radius), bearing, center_range

    def pickup(self) -> bool:
        """Attach the nearest eligible scene body with a real fixed constraint."""
        if self.held_body is not None:
            return False
        pose = self.robot.get_pose()
        x, y, heading = map(float, pose)
        front_x = x + config.CAMERA_FORWARD * math.cos(heading)
        front_y = y + config.CAMERA_FORWARD * math.sin(heading)
        candidates = []
        for body in self.environment.object_ids:
            surface_range, bearing, center_range = self._candidate_distance(body, front_x, front_y)
            if surface_range <= self.reach and abs(bearing) <= self.angle:
                candidates.append((surface_range, center_range, body))
        if not candidates:
            return False

        _, _, body = min(candidates)
        base_pos, base_orn = p.getBasePositionAndOrientation(self.robot.robot_id)
        inv_pos, inv_orn = p.invertTransform(base_pos, base_orn)
        body_pos, body_orn = p.getBasePositionAndOrientation(body)
        object_aabb = p.getAABB(body, -1)
        object_height = object_aabb[1][2] - object_aabb[0][2]
        grasp_height = config.CARRY_OFFSET_Z
        desired_world_pos, desired_world_orn = p.multiplyTransforms(
            base_pos, base_orn,
            [config.CARRY_OFFSET_X, 0.0, grasp_height - config.ROBOT_BASE_Z],
            [0, 0, 0, 1],
        )
        parent_frame_pos, parent_frame_orn = p.multiplyTransforms(
            inv_pos, inv_orn, desired_world_pos, desired_world_orn)
        self._saved_dynamics = {
            "mass": p.getDynamicsInfo(body, -1)[0],
            "lateralFriction": p.getDynamicsInfo(body, -1)[1],
            "rollingFriction": p.getDynamicsInfo(body, -1)[6],
            "spinningFriction": p.getDynamicsInfo(body, -1)[7],
            "restitution": p.getDynamicsInfo(body, -1)[5],
        }
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot.robot_id, parentLinkIndex=-1,
            childBodyUniqueId=body, childLinkIndex=-1,
            jointType=p.JOINT_FIXED, jointAxis=[0, 0, 0],
            parentFramePosition=parent_frame_pos, parentFrameOrientation=parent_frame_orn,
            childFramePosition=[0, 0, 0], childFrameOrientation=[0, 0, 0, 1],
        )
        # Keep loads suitable for this small mobile robot and suppress jitter
        # while held, restoring the object's original values on release.
        p.changeConstraint(self.constraint_id, maxForce=500, erp=0.9)
        p.resetBasePositionAndOrientation(body, desired_world_pos, desired_world_orn)
        p.resetBaseVelocity(body, [0, 0, 0], [0, 0, 0])
        # Match common fixed-constraint gripper practice: filter contacts only
        # between the held object and robot links. The object still collides
        # with the room, floor, and other objects during transport.
        self._filtered_links = list(range(-1, p.getNumJoints(self.robot.robot_id)))
        for link in self._filtered_links:
            p.setCollisionFilterPair(self.robot.robot_id, body, link, -1, enableCollision=0)
        self.held_body = body
        return True

    def drop(self) -> tuple[bool, bool]:
        """Release the body, step it to settle, and validate its final location."""
        if self.held_body is None:
            return False, False
        body = self.held_body
        constraint = self.constraint_id
        self.held_body = None
        self.constraint_id = None
        if constraint is not None:
            p.removeConstraint(constraint)
        for link in self._filtered_links:
            p.setCollisionFilterPair(self.robot.robot_id, body, link, -1, enableCollision=1)
        self._filtered_links.clear()
        dynamics = self._saved_dynamics or {}
        p.changeDynamics(body, -1, mass=dynamics.get("mass", 0.18),
                         lateralFriction=dynamics.get("lateralFriction", 0.8),
                         rollingFriction=dynamics.get("rollingFriction", 0.0),
                         spinningFriction=dynamics.get("spinningFriction", 0.0),
                         restitution=dynamics.get("restitution", 0.0))
        self._saved_dynamics = None
        # Bounded settle advances only within the explicit drop action.
        for _ in range(120):
            p.stepSimulation()
        position, _ = p.getBasePositionAndOrientation(body)
        inside = self.environment.is_in_drop_zone(position[0], position[1])
        return True, bool(inside)
