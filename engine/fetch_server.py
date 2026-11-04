"""HTTP interface for the isolated, single-owner-thread PyBullet simulation."""
from __future__ import annotations

import asyncio
import io
import math
import os
from contextlib import asynccontextmanager
from concurrent.futures import ThreadPoolExecutor

import pybullet as p
from fastapi import FastAPI, HTTPException, Response
from pydantic import BaseModel, Field
from PIL import Image

from engine.fetch_environment import FetchEnvironment
from engine.fetch_state import FetchPhase, FetchState
from engine.pickup_manager import PickupManager
from engine import config
from scripts.diff_drive_robot import DiffDriveRobot

_sim_thread = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pybullet-owner")
_started = False
_environment = None
_robot = None
_pickup = None
_state = None


@asynccontextmanager
async def _lifespan(_app):
    await _call(_initialize, int(os.getenv("FETCH_SEED", "1234")))
    try:
        yield
    finally:
        if _started:
            await _call(p.disconnect)
        _sim_thread.shutdown(wait=False)


app = FastAPI(title="Membrane Fetch Simulation", lifespan=_lifespan)


class MoveRequest(BaseModel):
    distance: float = Field(ge=-2.0, le=2.0)


class TurnRequest(BaseModel):
    degrees: float = Field(ge=-180.0, le=180.0)


class HealthResponse(BaseModel):
    task: str
    ready: bool


class StateResponse(BaseModel):
    state: str
    x: float
    y: float
    heading_deg: float
    holding: bool
    retry_count: int
    collided: bool


class MoveResponse(BaseModel):
    moved_m: float
    collided: bool
    state: str


class TurnResponse(BaseModel):
    turned_deg: float
    heading_deg: float
    state: str


class PickupResponse(BaseModel):
    success: bool
    holding: bool
    state: str


class DropResponse(BaseModel):
    released: bool
    success: bool
    holding: bool


class DepthResponse(BaseModel):
    distance_m: float | None


async def _call(fn, *args):
    return await asyncio.get_running_loop().run_in_executor(_sim_thread, fn, *args)


def _initialize(seed: int = 1234):
    global _started, _environment, _robot, _pickup, _state
    if _started:
        return
    p.connect(p.DIRECT)
    _environment = FetchEnvironment(seed).build()
    x, y, yaw = _environment.robot_start
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    _robot = DiffDriveRobot(os.path.join(base_dir, "urdf", "diff_drive_robot.urdf"), [x, y, config.ROBOT_BASE_Z], yaw)
    for j in (_robot.left_wheel_joint, _robot.right_wheel_joint):
        p.changeDynamics(_robot.robot_id, j, lateralFriction=1.5, spinningFriction=0.01, rollingFriction=0.01)
    _pickup = PickupManager(_environment, _robot)
    _robot.ignored_collision_bodies = {_environment.floor_id, _environment.drop_id}
    _robot.ignored_collision_bodies.update(_environment.object_ids)
    _state = FetchState()
    _state.start()
    _started = True


def _check_started():
    if not _started:
        raise HTTPException(status_code=503, detail="simulation not ready")


def _heading_deg() -> float:
    return (-math.degrees(float(_robot.get_pose()[2])) + 180.0) % 360.0 - 180.0


def _collision() -> bool:
    for contact in p.getContactPoints(bodyA=_robot.robot_id):
        other = contact[2]
        if other not in (_environment.floor_id, _environment.drop_id, _pickup.held_body):
            return True
    return False


def _move(distance: float):
    _check_started()
    if _state.phase == FetchPhase.DONE:
        return {"moved_m": 0.0, "collided": _state.collided, "state": _state.phase.value}
    recovering = _state.phase == FetchPhase.BLOCKED
    sign = 1.0 if distance >= 0 else -1.0
    target = abs(distance)
    moved = 0.0
    _robot.set_velocity(sign * config.FETCH_MOVE_SPEED, 0.0)
    ticks_per_second = round(1 / config.PHYSICS_STEP)
    max_ticks = max(1, math.ceil(target / config.FETCH_MOVE_SPEED * ticks_per_second * 3))
    for _ in range(max_ticks):
        before = _robot.get_pose().copy()
        p.stepSimulation()
        after = _robot.get_pose()
        moved += math.hypot(after[0] - before[0], after[1] - before[1])
        if not recovering and _collision():
            _robot.set_velocity(0, 0)
            _state.collision()
            _state.step += 1
            _check_step_limit()
            return {"moved_m": round(moved, 4), "collided": True, "state": _state.phase.value}
        if moved >= target:
            break
    moved = min(moved, target)
    _robot.set_velocity(0, 0)
    _state.collided = False
    _state.action()
    _state.step += 1
    if recovering:
        _state.recover()
    _check_step_limit()
    return {"moved_m": round(moved, 4), "collided": False, "state": _state.phase.value}


def _turn(degrees: float):
    _check_started()
    if _state.phase == FetchPhase.DONE:
        return {"turned_deg": 0.0, "heading_deg": _heading_deg(), "state": _state.phase.value}
    recovering = _state.phase == FetchPhase.BLOCKED
    start_heading = _heading_deg()
    direction = math.copysign(1.0, degrees) if degrees else 0.0
    _robot.set_velocity(0.0, -direction * math.radians(config.TURN_SPEED_DEG))
    ticks = 0
    ticks_per_second = round(1 / config.PHYSICS_STEP)
    max_ticks = max(1, math.ceil(abs(degrees) / config.TURN_SPEED_DEG * ticks_per_second * 3))
    while ticks < max_ticks:
        p.stepSimulation()
        ticks += 1
        if not recovering and _collision():
            _robot.set_velocity(0, 0)
            _state.collision()
            _state.step += 1
            _check_step_limit()
            return {"turned_deg": direction * abs((_heading_deg() - start_heading + 180.0) % 360.0 - 180.0),
                    "heading_deg": _heading_deg(), "state": _state.phase.value}
        turned = (_heading_deg() - start_heading + 180.0) % 360.0 - 180.0
        if direction * turned >= abs(degrees) - 0.4:
            break
    _robot.set_velocity(0, 0)
    _state.collided = False
    _state.action()
    _state.step += 1
    if recovering:
        _state.recover()
    _check_step_limit()
    return {"turned_deg": degrees, "heading_deg": _heading_deg(), "state": _state.phase.value}


def _check_step_limit():
    if _state.step >= 60 and _state.phase != FetchPhase.DONE:
        _state.phase = FetchPhase.DONE
        _state.outcome = "fail_not_reached"


def _status():
    _check_started()
    pose = _robot.get_pose()
    return {"state": _state.phase.value, "x": float(pose[0]), "y": float(pose[1]),
            "heading_deg": _heading_deg(), "holding": _pickup.held_body is not None,
            "retry_count": _state.retry_count, "collided": _state.collided}


def _look():
    _check_started()
    rgb, _ = _robot.get_camera_image()
    image = Image.fromarray(rgb.astype("uint8"), "RGB")
    output = io.BytesIO()
    image.save(output, format="JPEG", quality=90)
    return output.getvalue()


def _depth():
    _check_started()
    pose = _robot.get_pose()
    pos, _ = p.getBasePositionAndOrientation(_robot.robot_id)
    yaw = pose[2]
    start = [pos[0] + config.CAMERA_FORWARD * math.cos(yaw),
             pos[1] + config.CAMERA_FORWARD * math.sin(yaw), config.CAMERA_HEIGHT]
    end = [start[0] + 10 * math.cos(yaw), start[1] + 10 * math.sin(yaw),
           config.CAMERA_HEIGHT - 10 * math.sin(math.radians(config.CAMERA_TILT_DEG))]
    result = p.rayTest(start, end)[0]
    distance = None if result[0] < 0 else float(result[2] * 10)
    return {"distance_m": distance}


@app.get("/health", response_model=HealthResponse)
async def health():
    return {"task": "fetch", "ready": _started}


@app.get("/state", response_model=StateResponse)
async def get_state():
    return await _call(_status)


@app.get("/look")
async def get_look():
    payload = await _call(_look)
    return Response(payload, media_type="image/jpeg")


@app.get("/depth", response_model=DepthResponse)
async def get_depth():
    if os.getenv("FETCH_MENTOR_APPROVED_DEPTH", "").lower() != "true":
        raise HTTPException(status_code=403, detail="depth sensor is disabled")
    return await _call(_depth)


@app.post("/move", response_model=MoveResponse)
async def post_move(req: MoveRequest):
    return await _call(_move, req.distance)


@app.post("/turn", response_model=TurnResponse)
async def post_turn(req: TurnRequest):
    return await _call(_turn, req.degrees)


@app.post("/pickup", response_model=PickupResponse)
async def post_pickup():
    def action():
        _check_started()
        if _state.phase in (FetchPhase.BLOCKED, FetchPhase.DONE):
            return {"success": False, "holding": _pickup.held_body is not None,
                    "state": _state.phase.value}
        _state.phase = FetchPhase.PICKUP
        success = _pickup.pickup()
        _state.picked_up(success, _pickup.held_body is not None)
        _state.step += 1
        _check_step_limit()
        return {"success": success, "holding": _pickup.held_body is not None, "state": _state.phase.value}
    return await _call(action)


@app.post("/drop", response_model=DropResponse)
async def post_drop():
    def action():
        _check_started()
        if _state.phase in (FetchPhase.BLOCKED, FetchPhase.DONE):
            return {"released": False, "success": False, "holding": _pickup.held_body is not None}
        previous_phase = _state.phase
        released, in_zone = _pickup.drop()
        if released:
            _state.phase = FetchPhase.DROP
        else:
            _state.phase = previous_phase
        _state.dropped(released, in_zone, _pickup.held_body is not None)
        _state.step += 1
        _check_step_limit()
        return {"released": released, "success": bool(released and in_zone),
                "holding": _pickup.held_body is not None}
    return await _call(action)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("engine.fetch_server:app", host="127.0.0.1", port=8080)
