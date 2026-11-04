"""Visible physics demo: drive the existing robot to an object and grasp it."""
from __future__ import annotations

import argparse
import heapq
import math
import os
import sys
import time

import pybullet as p

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from engine import config
from engine.fetch_environment import FetchEnvironment
from engine.pickup_manager import PickupManager
from scripts.diff_drive_robot import DiffDriveRobot


def _path_clear(start, goal, target, environment) -> bool:
    """Check a straight approach corridor using engine-private scene geometry."""
    sx, sy = start
    gx, gy = goal
    length2 = (gx - sx) ** 2 + (gy - sy) ** 2
    if min(abs(sx), abs(sy), abs(gx), abs(gy)) < 0.35:
        return False
    for body in environment.object_ids:
        if body == target:
            continue
        (x, y, _), _ = p.getBasePositionAndOrientation(body)
        fraction = 0.0 if length2 == 0 else max(0.0, min(1.0, ((x - sx) * (gx - sx) + (y - sy) * (gy - sy)) / length2))
        closest = (sx + fraction * (gx - sx), sy + fraction * (gy - sy))
        aabb = p.getAABB(body, -1)
        radius = math.hypot(aabb[1][0] - aabb[0][0], aabb[1][1] - aabb[0][1]) / 2
        if math.dist((x, y), closest) < radius + 0.27:
            return False
    return True


def _carry_path(start, goal, carried_body, environment):
    """Plan a collision-clear grid route while keeping the held body in front."""
    cell = 0.2
    limit = config.ROOM_SIZE / 2 - 1.0
    count = round(2 * limit / cell)
    coords = [-limit + i * cell for i in range(count + 1)]

    def nearest(value):
        return min(range(len(coords)), key=lambda i: abs(coords[i] - value))

    source = (nearest(start[0]), nearest(start[1]))
    destination = (nearest(goal[0]), nearest(goal[1]))
    obstacles = []
    for body in environment.object_ids:
        if body == carried_body:
            continue
        (x, y, _), _ = p.getBasePositionAndOrientation(body)
        aabb = p.getAABB(body, -1)
        radius = math.hypot(aabb[1][0] - aabb[0][0], aabb[1][1] - aabb[0][1]) / 2
        obstacles.append((x, y, radius + 0.57))

    def clear(node):
        x, y = coords[node[0]], coords[node[1]]
        if node == source:
            return True
        return all((x - ox) ** 2 + (y - oy) ** 2 >= radius ** 2 for ox, oy, radius in obstacles)

    def heuristic(node):
        return math.hypot(coords[node[0]] - coords[destination[0]],
                          coords[node[1]] - coords[destination[1]])

    frontier = [(heuristic(source), 0.0, source)]
    came_from = {source: None}
    cost = {source: 0.0}
    steps = ((1, 0), (-1, 0), (0, 1), (0, -1),
             (1, 1), (1, -1), (-1, 1), (-1, -1))
    while frontier:
        _, current_cost, current = heapq.heappop(frontier)
        if current == destination:
            break
        if current_cost != cost[current]:
            continue
        for dx, dy in steps:
            neighbor = (current[0] + dx, current[1] + dy)
            if not (0 <= neighbor[0] < len(coords) and 0 <= neighbor[1] < len(coords)) or not clear(neighbor):
                continue
            next_cost = current_cost + cell * (math.sqrt(2) if dx and dy else 1.0)
            if next_cost < cost.get(neighbor, float("inf")):
                cost[neighbor] = next_cost
                came_from[neighbor] = current
                heapq.heappush(frontier, (next_cost + heuristic(neighbor), next_cost, neighbor))
    if destination not in came_from:
        return None
    nodes = []
    current = destination
    while current is not None:
        nodes.append((coords[current[0]], coords[current[1]]))
        current = came_from[current]
    return list(reversed(nodes))


def _animated_route(robot, points) -> bool:
    for waypoint_index, target in enumerate(points[1:], start=1):
        print(f"Carry waypoint {waypoint_index}/{len(points) - 1}.", flush=True)
        pose = robot.get_pose()
        dx, dy = target[0] - pose[0], target[1] - pose[1]
        distance = math.hypot(dx, dy)
        if distance < 0.025:
            continue
        desired = math.atan2(dy, dx)
        delta = math.degrees((desired - float(pose[2]) + math.pi) % (2 * math.pi) - math.pi)
        turned = _animated_turn(robot, delta)
        print(f"Carry turn complete: {turned}.", flush=True)
        if not turned:
            return False
        moved = _animated_drive(robot, distance)
        print(f"Carry move complete: {moved}.", flush=True)
        if not moved:
            return False
    return True


def _animated_turn(robot: DiffDriveRobot, degrees: float) -> bool:
    remaining = float(degrees)
    while abs(remaining) > 0.5:
        part = math.copysign(min(abs(remaining), 8.0), remaining)
        result = robot.turn_angle(part, speed_deg_s=55.0)
        if result["collided"]:
            return False
        remaining -= part
        p.getCameraImage(1, 1)
        time.sleep(0.05)
    return True


def _animated_drive(robot: DiffDriveRobot, distance: float) -> bool:
    remaining = float(distance)
    while remaining > 0.01:
        chunk = min(remaining, 0.07)
        stalled_attempts = 0
        while chunk > 0.01:
            result = robot.drive_distance(chunk, speed=0.32)
            if result["collided"]:
                return False
            progress = max(0.0, result["moved_m"])
            if progress < 0.004:
                stalled_attempts += 1
                if stalled_attempts >= 3:
                    return False
            else:
                chunk -= progress
                remaining -= progress
                stalled_attempts = 0
            p.getCameraImage(1, 1)
            time.sleep(0.06)
    return True


def main():
    parser = argparse.ArgumentParser(description="Show the existing robot drive to and physically grasp one object.")
    parser.add_argument("--seed", type=int, default=1234)
    args = parser.parse_args()

    client_id = p.connect(p.GUI)
    if client_id < 0:
        raise RuntimeError("Could not open the PyBullet GUI")
    try:
        environment = FetchEnvironment(args.seed).build()
        print("Fetch room initialized.", flush=True)
        sx, sy, heading = environment.robot_start
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        robot = DiffDriveRobot(os.path.join(root, "urdf", "diff_drive_robot.urdf"),
                               [sx, sy, config.ROBOT_BASE_Z], heading)
        p.resetDebugVisualizerCamera(cameraDistance=7, cameraYaw=45, cameraPitch=-45,
                                     cameraTargetPosition=[0, 0, 0])
        # This standalone demo uses the same engine-side pickup system. It
        # excludes only ground/goal contacts from robot collision sensing.
        robot.ignored_collision_bodies = {environment.floor_id, environment.drop_id}
        manager = PickupManager(environment, robot)

        candidates = []
        for body in environment.object_ids:
            (ox, oy, _), _ = p.getBasePositionAndOrientation(body)
            distance = math.hypot(ox - sx, oy - sy)
            if distance < 0.85:
                continue
            ux, uy = (ox - sx) / distance, (oy - sy) / distance
            goal = (ox - 0.48 * ux, oy - 0.48 * uy)
            if max(abs(goal[0]), abs(goal[1])) <= 1.6 and _path_clear((sx, sy), goal, body, environment):
                candidates.append((math.dist((sx, sy), goal), body, goal, math.atan2(uy, ux)))
        print(f"Clear approach options: {len(candidates)}.", flush=True)

        if not candidates:
            print("No clear straight pickup approach for this seed. Try --seed 42 or another seed.")
        else:
            _, target, goal, desired_heading = min(candidates)
            print("Robot is moving toward an internally selected reachable object.")
            initial_heading = float(robot.get_pose()[2])
            delta = math.degrees((desired_heading - initial_heading + math.pi) % (2 * math.pi) - math.pi)
            turned = _animated_turn(robot, delta)
            traveled = math.dist((sx, sy), goal)
            moved = turned and _animated_drive(robot, traveled)
            if moved:
                # The approach vector already points at the target; pickup uses
                # the manager's internal range/bearing checks and real fixed joint.
                picked_up = manager.pickup()
                print(f"Drive completed: yes. Fixed-constraint pickup: {'yes' if picked_up and manager.constraint_id is not None else 'no'}.")
            else:
                picked_up = False
                print("Drive was stopped because the route collided or motion was insufficient; no pickup claimed.")
            if picked_up:
                robot.ignored_collision_bodies.add(manager.held_body)
                robot_pose = robot.get_pose()
                dx, dy = environment.drop_center[0] - robot_pose[0], environment.drop_center[1] - robot_pose[1]
                drop_distance = math.hypot(dx, dy)
                ux, uy = dx / drop_distance, dy / drop_distance
                drop_goal = (environment.drop_center[0] - 0.42 * ux,
                             environment.drop_center[1] - 0.42 * uy)
                print("Planning the carry route.", flush=True)
                route = _carry_path(robot_pose[:2], drop_goal, manager.held_body, environment)
                print(f"Carry route ready: {route is not None}.", flush=True)
                transported = route is not None and _animated_route(robot, route)
                print(f"Carry motion complete: {transported}.", flush=True)
                if transported:
                    # Align the held body with the centre of the marked area.
                    final_pose = robot.get_pose()
                    bearing = math.atan2(dy, dx)
                    final_delta = math.degrees((bearing - float(final_pose[2]) + math.pi) % (2 * math.pi) - math.pi)
                    transported = _animated_turn(robot, final_delta)
                if transported:
                    released, in_zone = manager.drop()
                    print(f"Transport completed: yes. Released: {released}. Drop-zone validation: {'pass' if in_zone else 'fail'}.")
                else:
                    print("Pickup succeeded, but transport route did not complete; object remains held.")
            if not picked_up:
                print("This run did not demonstrate a pickup. Close the window to exit.")
            else:
                print("The scene is paused for inspection. Close the window to exit.")

        print("Simulation is idle; GUI remains open.", flush=True)
        while p.isConnected(client_id):
            p.getCameraImage(1, 1)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except Exception:
        import traceback
        traceback.print_exc()
        raise
    finally:
        if p.isConnected(client_id):
            p.disconnect(client_id)


if __name__ == "__main__":
    main()
