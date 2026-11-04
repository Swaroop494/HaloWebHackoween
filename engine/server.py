import pybullet as p
import pybullet_data
import time
import os
import json
import math
from fastapi import FastAPI, Response, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
import uvicorn
from engine import config
from engine.scene import Scene
from engine.robot import Robot
from engine.camera import Camera
from PIL import Image
import io

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=['*'],
    allow_credentials=True,
    allow_methods=['*'],
    allow_headers=['*'],
)

world = None
scene_obj = None
robot = None
camera = None
run_info = {}
step_counter = 0
_connected = False

class MoveReq(BaseModel):
    distance: float

class TurnReq(BaseModel):
    degrees: float

class RunStartReq(BaseModel):
    seed: int
    run_id: str
    command: str

def _ensure_connected():
    global _connected, camera
    if not _connected:
        try:
            p.disconnect()
        except Exception:
            pass
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        camera = Camera()
        _connected = True

@app.on_event('startup')
def startup_event():
    _ensure_connected()

@app.post('/run/start')
def run_start(req: RunStartReq):
    global world, scene_obj, robot, run_info, step_counter, camera
    _ensure_connected()
    p.resetSimulation()
    p.setGravity(0,0,-9.8)
    plane = p.loadURDF('plane.urdf')
    s = config.ROOM_SIZE
    wall_h = config.WALL_HEIGHT
    wall_t = config.WALL_THICK
    col_box = p.createCollisionShape(p.GEOM_BOX, halfExtents=[s/2, wall_t/2, wall_h/2])
    vis_box = p.createVisualShape(p.GEOM_BOX, halfExtents=[s/2, wall_t/2, wall_h/2], rgbaColor=config.WALL_COLOR)
    p.createMultiBody(0, col_box, vis_box, [0, s/2, wall_h/2])
    p.createMultiBody(0, col_box, vis_box, [0, -s/2, wall_h/2])
    col_box2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_t/2, s/2, wall_h/2])
    vis_box2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_t/2, s/2, wall_h/2], rgbaColor=config.WALL_COLOR)
    p.createMultiBody(0, col_box2, vis_box2, [s/2, 0, wall_h/2])
    p.createMultiBody(0, col_box2, vis_box2, [-s/2, 0, wall_h/2])
    dz_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[config.DROP_ZONE_SIZE/2, config.DROP_ZONE_SIZE/2, 0.001])
    dz_vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[config.DROP_ZONE_SIZE/2, config.DROP_ZONE_SIZE/2, 0.001], rgbaColor=config.DROP_ZONE_COLOR)
    scene_obj = Scene(req.seed)
    scene_obj.generate()
    p.createMultiBody(0, dz_col, dz_vis, [scene_obj.drop_zone_pos[0], scene_obj.drop_zone_pos[1], 0.001])
    for o in scene_obj.objects:
        pos = [o['pos'][0], o['pos'][1], o['height']/2.0]
        if o['primitive'] == 'cube':
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[o['size']/2.0, o['size']/2.0, o['height']/2.0])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[o['size']/2.0, o['size']/2.0, o['height']/2.0], rgbaColor=o['color'])
        elif o['primitive'] == 'sphere':
            col = p.createCollisionShape(p.GEOM_SPHERE, radius=o['size']/2.0)
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=o['size']/2.0, rgbaColor=o['color'])
        elif o['primitive'] == 'cylinder':
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=o['size']/2.0, height=o['height'])
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=o['size']/2.0, length=o['height'], rgbaColor=o['color'])
        elif o['primitive'] == 'cone':
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=o['size']/2.0, height=o['height'])
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=o['size']/2.0, length=o['height'], rgbaColor=o['color'])
        else:
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[o['size']/2.0, o['size']/2.0, o['height']/2.0])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[o['size']/2.0, o['size']/2.0, o['height']/2.0], rgbaColor=o['color'])
        p.createMultiBody(1, col, vis, pos)
    for obs in scene_obj.obstacles:
        if obs['type'] == 'box':
            col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.25,0.25,0.25])
            vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.25,0.25,0.25], rgbaColor=obs['color'])
            p.createMultiBody(1, col, vis, obs['pos'])
        elif obs['type'] == 'vase':
            col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.175, height=0.6)
            vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.175, length=0.6, rgbaColor=obs['color'])
            p.createMultiBody(1, col, vis, obs['pos'])
    rs = scene_obj.robot_start
    robot = Robot(rs, scene_obj.robot_start_heading)
    col_r = p.createCollisionShape(p.GEOM_CAPSULE, radius=config.ROBOT_RADIUS, height=config.ROBOT_HEIGHT)
    vis_r = p.createVisualShape(p.GEOM_CAPSULE, radius=config.ROBOT_RADIUS, length=config.ROBOT_HEIGHT, rgbaColor=config.ROBOT_COLOR)
    robot.robot_id = p.createMultiBody(1, col_r, vis_r, [rs[0], rs[1], config.ROBOT_HEIGHT/2.0])
    orn = p.getQuaternionFromEuler([0,0,robot.heading_to_yaw(robot.heading_deg)])
    p.resetBasePositionAndOrientation(robot.robot_id, [rs[0], rs[1], config.ROBOT_HEIGHT/2.0], orn)
    run_info = {'seed': req.seed, 'run_id': req.run_id, 'command': req.command}
    step_counter = 0
    robot.state = 'Locate'
    robot.collided = False
    robot.retry_count = 0
    robot.holding_id = None
    return {'run_id': req.run_id, 'status': 'ready'}

@app.get('/state')
def get_state():
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    return {
        'state': robot.state,
        'x': float(robot.x),
        'y': float(robot.y),
        'heading_deg': float(robot.heading_deg),
        'holding': robot.holding_id is not None,
        'retry_count': int(robot.retry_count),
        'step': int(step_counter),
        'collided': bool(robot.collided)
    }

@app.get('/look')
def get_look():
    global camera
    _ensure_connected()
    if robot is None or camera is None:
        raise HTTPException(status_code=400, detail='no run')
    arr = camera.render(robot)
    img = Image.fromarray(arr, 'RGBA').convert('RGB')
    buf = io.BytesIO()
    img.save(buf, format='JPEG')
    buf.seek(0)
    resp = Response(content=buf.getvalue(), media_type='image/jpeg')
    resp.headers['X-Step'] = str(step_counter)
    resp.headers['X-Frame-Id'] = f'step_{step_counter:03d}'
    return resp

@app.get('/depth')
def get_depth():
    global camera
    _ensure_connected()
    if robot is None or camera is None:
        raise HTTPException(status_code=400, detail='no run')
    d = camera.depth_ahead(robot)
    return {'distance_m': d}

@app.post('/move')
def post_move(req: MoveReq):
    global step_counter
    _ensure_connected()
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    dist = req.distance
    if dist > 2.0: dist = 2.0
    if dist < -2.0: dist = -2.0
    moved = 0.0
    collided = False
    backed_off = 0.0
    steps = int(abs(dist) / 0.05) if abs(dist) > 1e-9 else 1
    direction = 1.0 if dist >= 0 else -1.0
    for i in range(steps):
        nx = robot.x + direction * 0.05 * math.cos(math.radians(robot.heading_deg))
        ny = robot.y + direction * 0.05 * math.sin(math.radians(robot.heading_deg))
        robot.x = nx; robot.y = ny
        robot.set_pose(robot.x, robot.y, robot.heading_deg)
        for _ in range(5):
            p.stepSimulation()
        contacts = p.getContactPoints(robot.robot_id)
        if contacts and len(contacts) > 0:
            collided = True
            break
        moved += 0.05
    if collided:
        bx = robot.x - direction * 0.1 * math.cos(math.radians(robot.heading_deg))
        by = robot.y - direction * 0.1 * math.sin(math.radians(robot.heading_deg))
        robot.x = bx; robot.y = by
        robot.set_pose(robot.x, robot.y, robot.heading_deg)
        for _ in range(10):
            p.stepSimulation()
        backed_off = 0.1
        robot.collided = True
        robot.state = 'Blocked'
        robot.retry_count += 1
    else:
        robot.collided = False
    step_counter += 1
    return {'moved_m': float(moved), 'collided': bool(collided), 'state': robot.state, 'backed_off_m': float(backed_off)}

@app.post('/turn')
def post_turn(req: TurnReq):
    global step_counter
    _ensure_connected()
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    deg = req.degrees
    if deg > 180.0: deg = 180.0
    if deg < -180.0: deg = -180.0
    robot.heading_deg += deg
    if robot.heading_deg > 180.0: robot.heading_deg -= 360.0
    if robot.heading_deg <= -180.0: robot.heading_deg += 360.0
    robot.set_pose(robot.x, robot.y, robot.heading_deg)
    for _ in range(20):
        p.stepSimulation()
    robot.collided = False
    step_counter += 1
    return {'turned_deg': float(deg), 'heading_deg': float(robot.heading_deg), 'state': robot.state}

@app.post('/pickup')
def post_pickup():
    global step_counter
    _ensure_connected()
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    if robot.holding_id is not None:
        step_counter += 1
        return {'success': False, 'holding': True, 'state': robot.state}
    robot.holding_id = 1
    robot.state = 'Carry'
    step_counter += 1
    return {'success': True, 'holding': True, 'state': robot.state}

@app.post('/drop')
def post_drop():
    global step_counter
    _ensure_connected()
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    released = True
    in_zone = False
    if robot.holding_id is not None:
        dz = scene_obj.drop_zone_pos if scene_obj else (0,0)
        if abs(robot.x - dz[0]) <= config.DROP_ZONE_SIZE/2.0 and abs(robot.y - dz[1]) <= config.DROP_ZONE_SIZE/2.0:
            in_zone = True
            robot.state = 'Done'
        else:
            robot.retry_count += 1
            robot.state = 'Carry'
        robot.holding_id = None
    step_counter += 1
    return {'released': released, 'in_zone': bool(in_zone), 'state': robot.state, 'retry_count': int(robot.retry_count)}

@app.post('/run/end')
def run_end():
    if robot is None:
        raise HTTPException(status_code=400, detail='no run')
    outcome = 'pass' if robot.state == 'Done' else 'fail_not_reached'
    res = {'outcome': outcome, 'steps': int(step_counter), 'duration_s': 0.0}
    return res

if __name__ == '__main__':
    uvicorn.run(app, host='127.0.0.1', port=8080)
