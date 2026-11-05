"""Run a visible, physics-backed six-object fetch demonstration.

This local demonstration uses the existing PremiumEnvironment, differential
drive controller and PickupManager. Scene geometry stays inside the simulator;
no object description is sent to an LLM or exposed over HTTP.
"""
from __future__ import annotations

import argparse
import heapq
import io
import math
import os
import sys
import time
import uuid

import pybullet as p
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from engine import config
from engine.premium_environment import PremiumEnvironment
from engine.pickup_manager import PickupManager
from evaluation.fetch_logger import FetchLogger
from scripts.diff_drive_robot import DiffDriveRobot


GRID = 0.15
ROBOT_CLEARANCE = 0.16
CARRY_OFFSET = config.CARRY_OFFSET_X
DROP_SLOTS = ((-0.75, -0.9), (0.0, -0.9), (0.75, -0.9),
              (-0.75, 0.9), (0.0, 0.9), (0.75, 0.9))


def _wrap(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def _point_segment_distance(point, start, end) -> float:
    vx, vy = end[0] - start[0], end[1] - start[1]
    length2 = vx * vx + vy * vy
    if length2 < 1e-12:
        return math.dist(point, start)
    t = max(0.0, min(1.0, ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / length2))
    return math.hypot(point[0] - start[0] - t * vx, point[1] - start[1] - t * vy)


def _body_radius(body: int) -> float:
    lower, upper = p.getAABB(body, -1)
    return math.hypot((upper[0] - lower[0]) / 2, (upper[1] - lower[1]) / 2)


def _obstacles(environment: PremiumEnvironment, excluded: set[int]):
    items = []
    for body in environment.object_ids:
        if body in excluded:
            continue
        (x, y, _), _ = p.getBasePositionAndOrientation(body)
        items.append((float(x), float(y), _body_radius(body)))
    return items


def _route(start, goal, obstacles, held_radius: float = 0.0):
    """A* route that checks both robot and carried-object swept paths."""
    half = PremiumEnvironment.ROOM_HALF - 0.42
    count = int(math.floor(2 * half / GRID))
    coords = [-half + index * GRID for index in range(count + 1)]

    def nearest(value):
        return min(range(len(coords)), key=lambda index: abs(coords[index] - value))

    source = (nearest(start[0]), nearest(start[1]))
    destination = (nearest(goal[0]), nearest(goal[1]))
    steps = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))

    def xy(node):
        if node == source:
            return tuple(start)
        if node == destination:
            return tuple(goal)
        return coords[node[0]], coords[node[1]]

    def clear_edge(a, b):
        axy, bxy = xy(a), xy(b)
        length = math.dist(axy, bxy)
        if length < 1e-9:
            return True
        dx, dy = (bxy[0] - axy[0]) / length, (bxy[1] - axy[1]) / length
        for ox, oy, radius in obstacles:
            if _point_segment_distance((ox, oy), axy, bxy) < radius + ROBOT_CLEARANCE:
                return False
            if held_radius:
                held_a = (axy[0] + CARRY_OFFSET * dx, axy[1] + CARRY_OFFSET * dy)
                held_b = (bxy[0] + CARRY_OFFSET * dx, bxy[1] + CARRY_OFFSET * dy)
                if _point_segment_distance((ox, oy), held_a, held_b) < radius + held_radius + 0.035:
                    return False
        return True

    def heuristic(node):
        return math.dist(xy(node), xy(destination))

    frontier = [(heuristic(source), 0.0, source)]
    cost = {source: 0.0}
    parent = {source: None}
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current == destination:
            break
        if current_cost != cost[current]:
            continue
        for dx, dy in steps:
            neighbor = current[0] + dx, current[1] + dy
            if not (0 <= neighbor[0] < len(coords) and 0 <= neighbor[1] < len(coords)):
                continue
            if not clear_edge(current, neighbor):
                continue
            next_cost = current_cost + GRID * (math.sqrt(2) if dx and dy else 1.0)
            if next_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = next_cost
                parent[neighbor] = current
                heapq.heappush(frontier, (next_cost + heuristic(neighbor), next_cost, neighbor))
    if destination not in parent:
        return None

    nodes = []
    current = destination
    while current is not None:
        nodes.append(xy(current))
        current = parent[current]
    nodes.reverse()
    if not nodes:
        return None

    # Remove collinear grid nodes, keeping the route readable in the GUI.
    compact = [nodes[0]]
    previous_direction = None
    for index in range(1, len(nodes)):
        a, b = nodes[index - 1], nodes[index]
        direction = (round((b[0] - a[0]) / GRID, 5), round((b[1] - a[1]) / GRID, 5))
        if previous_direction is not None and direction != previous_direction:
            compact.append(a)
        previous_direction = direction
    compact.append(nodes[-1])
    exact_start, exact_goal = tuple(start), tuple(goal)
    if math.dist(compact[0], exact_start) > 0.005:
        compact.insert(0, exact_start)
    if math.dist(compact[-1], exact_goal) > 0.005:
        compact.append(exact_goal)
    return compact


def _camera_png(robot) -> bytes:
    rgb, _ = robot.get_camera_image()
    image = Image.fromarray(rgb, mode="RGB")
    stream = io.BytesIO()
    image.save(stream, format="PNG")
    return stream.getvalue()


class BatchRunner:
    def __init__(self, environment, robot, manager, logger, render: bool):
        self.environment = environment
        self.robot = robot
        self.manager = manager
        self.logger = logger
        self.render = render
        self.retries = 0
        self.collided = False
        self.state = "IDLE"

    def record(self, tool, args, result, next_state):
        self.state = next_state
        state = {"state": self.state, "holding": self.manager.held_body is not None,
                 "retry_count": self.retries, "collided": bool(result.get("collided", False)),
                 "robot_pose": [float(value) for value in self.robot.get_pose()]}
        self.logger.record(tool, args, result, _camera_png(self.robot), state, model="local-demo")
        if self.render:
            p.getCameraImage(1, 1)
            time.sleep(0.025)

    def turn(self, degrees, next_state):
        result = self.robot.turn_angle(float(degrees), speed_deg_s=65.0)
        self.collided = bool(result["collided"])
        self.record("turn", {"degrees": float(degrees)}, result, next_state)
        requested = float(degrees)
        actual = float(result["turned_deg"])
        return not self.collided and (abs(requested) <= 3.5 or abs(actual - requested) <= 3.0)

    def drive(self, distance, next_state):
        result = self.robot.drive_distance(float(distance), speed=0.42)
        self.collided = bool(result["collided"])
        self.record("move", {"distance_m": float(distance)}, result, next_state)
        moved = abs(float(result["moved_m"]))
        ok = not self.collided and moved >= min(abs(float(distance)) * 0.75, 0.025)
        return ok, moved

    def recover(self, next_state):
        if self.retries >= config.BLOCKED_RETRIES:
            return False
        self.retries += 1
        self.collided = False
        # Reverse and change heading; each call is a bounded physics action.
        back = self.robot.drive_distance(-0.14, speed=0.28, collision_check=False)
        self.record("move", {"distance_m": -0.14, "recovery": True}, back, next_state)
        turn = self.robot.turn_angle(32.0 if self.retries % 2 else -32.0,
                                     speed_deg_s=55.0, collision_check=False)
        self.record("turn", {"degrees": float(turn["turned_deg"]), "recovery": True}, turn, next_state)
        return True

    def follow(self, points, next_state):
        for target in points[1:]:
            pose = self.robot.get_pose()
            dx, dy = float(target[0] - pose[0]), float(target[1] - pose[1])
            distance = math.hypot(dx, dy)
            if distance < 0.025:
                continue
            heading = math.degrees(math.atan2(dy, dx))
            current = math.degrees(float(pose[2]))
            delta = _wrap(heading - current)
            if abs(delta) > 0.8:
                for _ in range(max(1, math.ceil(abs(delta) / 28.0))):
                    part = math.copysign(min(abs(delta), 28.0), delta)
                    if not self.turn(part, next_state):
                        return False
                    delta -= part
            remaining = distance
            while remaining > 0.018:
                amount = min(remaining, 0.22)
                moved_ok, moved = self.drive(amount, next_state)
                if not moved_ok:
                    return False
                remaining -= moved
            # Micro routes get a fresh pose and heading at each waypoint.
        return True

    def navigate(self, goal, obstacles, held_body=None, final_heading=None, next_state="APPROACH"):
        held_radius = _body_radius(held_body) if held_body is not None else 0.0
        for attempt in range(config.BLOCKED_RETRIES + 1):
            pose = self.robot.get_pose()
            padding = attempt * 0.05
            route_obstacles = [(x, y, radius + padding) for x, y, radius in obstacles]
            route = _route(pose[:2], goal, route_obstacles, held_radius)
            if route is None:
                return False
            if self.follow(route, next_state):
                if final_heading is not None:
                    delta = _wrap(math.degrees(final_heading - float(self.robot.get_pose()[2])))
                    ok = True
                    while abs(delta) > 0.8:
                        part = math.copysign(min(abs(delta), 28.0), delta)
                        if not self.turn(part, next_state):
                            ok = False
                            break
                        delta -= part
                    if not ok:
                        pass
                    else:
                        return True
                else:
                    return True
            if attempt >= config.BLOCKED_RETRIES or not self.recover(next_state):
                return False
        return False


def _approach_plan(runner: BatchRunner, body: int, environment):
    (ox, oy, _), _ = p.getBasePositionAndOrientation(body)
    obstacles = _obstacles(environment, {body})
    choices = []
    # Eight camera-facing approaches let the planner go around nearby items.
    for index in range(8):
        angle = index * math.pi / 4
        goal = (float(ox) - 0.55 * math.cos(angle), float(oy) - 0.55 * math.sin(angle))
        if max(abs(goal[0]), abs(goal[1])) > PremiumEnvironment.ROOM_HALF - 0.48:
            continue
        front = (goal[0] + config.CAMERA_FORWARD * math.cos(angle),
                 goal[1] + config.CAMERA_FORWARD * math.sin(angle))
        eligible = []
        for candidate in environment.object_ids:
            (cx, cy, _), _ = p.getBasePositionAndOrientation(candidate)
            dx, dy = cx - front[0], cy - front[1]
            center_range = math.hypot(dx, dy)
            bearing = _wrap(math.degrees(math.atan2(dy, dx)) - math.degrees(angle))
            lower, upper = p.getAABB(candidate, -1)
            surface_range = max(0.0, center_range -
                                max(upper[0] - lower[0], upper[1] - lower[1]) / 2)
            if surface_range <= config.PICKUP_REACH and abs(bearing) <= config.PICKUP_ANGLE_DEG:
                eligible.append((surface_range, center_range, candidate))
        if not eligible or min(eligible)[2] != body:
            continue
        route = _route(runner.robot.get_pose()[:2], goal, obstacles)
        if route:
            length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            choices.append((length, goal, angle, route))
    return min(choices, default=None, key=lambda item: item[0])


def _carry_plan(runner: BatchRunner, environment, body: int, slot):
    (sx, sy) = slot
    obstacles = _obstacles(environment, {body})
    candidates = []
    # Approach each row from its outer edge so the base remains clear of the
    # delivered objects and can leave the marked area after every release.
    inward_heading = math.pi / 2 if sy < 0 else -math.pi / 2
    for delta in (-0.16, 0.0, 0.16):
        heading = inward_heading + delta
        goal = (environment.drop_center[0] + sx - CARRY_OFFSET * math.cos(heading),
                environment.drop_center[1] + sy - CARRY_OFFSET * math.sin(heading))
        if max(abs(goal[0]), abs(goal[1])) > PremiumEnvironment.ROOM_HALF - 0.48:
            continue
        route = _route(runner.robot.get_pose()[:2], goal, obstacles, _body_radius(body))
        if route:
            length = sum(math.dist(a, b) for a, b in zip(route, route[1:]))
            candidates.append((length, goal, heading, route))
    return min(candidates, default=None, key=lambda item: item[0])


def run(seed: int, gui: bool, hold_open: bool):
    connection = p.connect(p.GUI if gui else p.DIRECT)
    if connection < 0:
        raise RuntimeError("PyBullet could not connect")
    logger = None
    outcome = "failed"
    total_in = total_out = 0
    try:
        p.setRealTimeSimulation(0)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        environment = PremiumEnvironment(seed).build()
        sx, sy, heading = environment.robot_start
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        robot = DiffDriveRobot(os.path.join(root, "urdf", "diff_drive_robot.urdf"),
                               [sx, sy, config.ROBOT_BASE_Z], heading)
        robot.ignored_collision_bodies = {environment.floor_id, environment.drop_id}
        manager = PickupManager(environment, robot)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
            p.resetDebugVisualizerCamera(cameraDistance=7.5, cameraYaw=42, cameraPitch=-35,
                                         cameraTargetPosition=[*environment.drop_center, 0.2])

        run_id = f"premium_batch_{int(time.time())}_{uuid.uuid4().hex[:6]}"
        logger = FetchLogger(run_id, "Pick and place all six objects one at a time")
        runner = BatchRunner(environment, robot, manager, logger, gui)
        print("Premium Fetch scene ready. Six separate deliveries will run.", flush=True)
        print("The task is engine-assisted because no LLM API key is configured.", flush=True)
        remaining = list(environment.object_ids)
        delivered = set()

        for delivery_index, slot in enumerate(DROP_SLOTS, start=1):
            if not remaining:
                break
            runner.retries = 0
            approach_options = []
            for candidate in remaining:
                candidate_plan = _approach_plan(runner, candidate, environment)
                if candidate_plan is not None:
                    approach_options.append((candidate_plan[0], candidate, candidate_plan))
            if not approach_options:
                print(f"Delivery {delivery_index}/6: no safe approach route.", flush=True)
                break
            _, target, plan = min(approach_options, key=lambda choice: choice[0])
            _, approach, target_heading, route = plan
            runner.state = "APPROACH"
            if not runner.navigate(approach, _obstacles(environment, {target}),
                                   final_heading=target_heading, next_state="APPROACH"):
                print(f"Delivery {delivery_index}/6: approach stopped after bounded recovery.", flush=True)
                break

            runner.state = "PICKUP"
            picked = manager.pickup()
            runner.record("pickup", {}, {"success": bool(picked), "collided": False}, "PICKUP")
            if not picked or manager.constraint_id is None:
                print(f"Delivery {delivery_index}/6: no fixed-constraint pickup.", flush=True)
                break
            actual_body = manager.held_body
            robot.ignored_collision_bodies.add(actual_body)
            if actual_body in delivered:
                print(f"Delivery {delivery_index}/6: an already placed item was selected; stopping safely.", flush=True)
                break

            runner.state = "CARRY"
            carry_plan = _carry_plan(runner, environment, actual_body, slot)
            if carry_plan is None:
                print(f"Delivery {delivery_index}/6: no safe carry route.", flush=True)
                break
            _, carry_goal, carry_heading, carry_route = carry_plan
            if not runner.navigate(carry_goal, _obstacles(environment, {actual_body}),
                                   held_body=actual_body, final_heading=carry_heading,
                                   next_state="CARRY"):
                print(f"Delivery {delivery_index}/6: carry route stopped after bounded recovery.", flush=True)
                break

            runner.state = "DROP"
            released, in_zone = manager.drop()
            robot.ignored_collision_bodies.discard(actual_body)
            runner.record("drop", {}, {"released": bool(released), "in_zone": bool(in_zone),
                                        "collided": False}, "DROP")
            if not released or not in_zone:
                print(f"Delivery {delivery_index}/6: drop validation failed.", flush=True)
                break
            delivered.add(actual_body)
            remaining = [body for body in remaining if body != actual_body]
            print(f"Delivery {delivery_index}/6: pickup, carry, release, and zone validation passed.", flush=True)

        if len(delivered) == len(DROP_SLOTS):
            outcome = "all six delivered and individually validated"
            runner.state = "DONE"
            print("All six objects are placed in separate marked-zone slots.", flush=True)
        else:
            print(f"Validated deliveries: {len(delivered)}/6.", flush=True)
        logger.finalize(seed, outcome, total_in, total_out)
        print(f"Action log and camera frames: {logger.directory}", flush=True)
        print("Simulation is paused for inspection.", flush=True)
        while gui and hold_open and p.isConnected(connection):
            p.getCameraImage(1, 1)
            time.sleep(0.1)
        return outcome
    finally:
        if logger is not None and logger.path.exists() and not (logger.directory / "meta.json").exists():
            logger.finalize(seed, outcome, total_in, total_out)
        if p.isConnected(connection):
            p.disconnect(connection)


def main():
    parser = argparse.ArgumentParser(description="Pick and place all six Premium Fetch objects one at a time.")
    parser.add_argument("--seed", type=int, default=1234)
    parser.add_argument("--direct", action="store_true", help="Run headless in PyBullet DIRECT mode.")
    parser.add_argument("--no-hold", action="store_true", help="Close GUI automatically when the task ends.")
    args = parser.parse_args()
    run(args.seed, gui=not args.direct, hold_open=not args.no_hold)


if __name__ == "__main__":
    main()
