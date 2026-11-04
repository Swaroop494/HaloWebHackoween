import pybullet as p
import numpy as np
import math
from engine import config

class Robot:
    def __init__(self, start_pos=(0,0), start_heading=0.0):
        self.start_pos = start_pos
        self.start_heading = start_heading  # degrees, 0=+x, clockwise positive
        self.x = start_pos[0]
        self.y = start_pos[1]
        self.heading_deg = start_heading
        self.robot_id = None
        self.holding_id = None
        self.collided = False
        self.state = 'Locate'
        self.retry_count = 0
        self.step = 0

    def heading_to_yaw(self, heading_deg):
        # PyBullet yaw is counter-clockwise positive; convert: yaw_rad = -heading_deg * pi/180
        return -math.radians(heading_deg)

    def set_pose(self, x=None, y=None, heading_deg=None):
        if x is not None: self.x = x
        if y is not None: self.y = y
        if heading_deg is not None: self.heading_deg = heading_deg % 360.0
            # normalize to (-180, 180]
        if self.heading_deg > 180.0: self.heading_deg -= 360.0
        if self.heading_deg <= -180.0: self.heading_deg += 360.0
        if self.robot_id is not None:
            pos = [self.x, self.y, config.ROBOT_HEIGHT/2.0]
            orn = p.getQuaternionFromEuler([0,0,self.heading_to_yaw(self.heading_deg)])
            p.resetBasePositionAndOrientation(self.robot_id, pos, orn)

    def get_camera_pose(self):
        # 0.15 m forward of centre, 0.5 m above floor, tilted 10° down
        fx = self.x + config.CAMERA_FORWARD * math.cos(math.radians(self.heading_deg))
        fy = self.y + config.CAMERA_FORWARD * math.sin(math.radians(self.heading_deg))
        fz = config.CAMERA_HEIGHT
        # tilt down 10 degrees
        return (fx, fy, fz)
