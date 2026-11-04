import pybullet as p
import numpy as np
import math
from engine import config

class Camera:
    def __init__(self):
        pass

    def render(self, robot):
        width = config.CAMERA_WIDTH
        height = config.CAMERA_HEIGHT_PIX
        fov = config.CAMERA_FOV_H
        aspect = width / height
        near = 0.01
        far = 10.0
        view_matrix = self._view_matrix(robot)
        proj_matrix = p.computeProjectionMatrixFOV(fov=fov, aspect=aspect, nearVal=near, farVal=far)
        try:
            img_arr = p.getCameraImage(width, height, view_matrix, proj_matrix, renderer=p.ER_TINY_RENDERER)
        except Exception:
            img_arr = p.getCameraImage(width, height, view_matrix, proj_matrix)
        rgb = img_arr[2]
        arr = np.array(rgb, dtype=np.uint8)
        arr = arr.reshape((height, width, 4))
        return arr

    def _view_matrix(self, robot):
        cx = robot.x + config.CAMERA_FORWARD * math.cos(math.radians(robot.heading_deg))
        cy = robot.y + config.CAMERA_FORWARD * math.sin(math.radians(robot.heading_deg))
        cz = config.CAMERA_HEIGHT
        tilt = math.radians(config.CAMERA_TILT_DEG)
        dx = math.cos(math.radians(robot.heading_deg)) * math.cos(tilt)
        dy = math.sin(math.radians(robot.heading_deg)) * math.cos(tilt)
        dz = -math.sin(tilt)
        target = [cx + dx, cy + dy, cz + dz]
        up = [0, 0, 1]
        return p.computeViewMatrix([cx, cy, cz], target, up)

    def depth_ahead(self, robot):
        cx = robot.x + config.CAMERA_FORWARD * math.cos(math.radians(robot.heading_deg))
        cy = robot.y + config.CAMERA_FORWARD * math.sin(math.radians(robot.heading_deg))
        cz = config.CAMERA_HEIGHT
        dx = math.cos(math.radians(robot.heading_deg))
        dy = math.sin(math.radians(robot.heading_deg))
        dz = -math.sin(math.radians(config.CAMERA_TILT_DEG))
        ray_from = [cx, cy, cz]
        ray_to = [cx + dx * 10.0, cy + dy * 10.0, cz + dz * 10.0]
        res = p.rayTest(ray_from, ray_to)
        if res and len(res) > 0 and res[0][0] >= 0:
            hit = res[0]
            dist = hit[2]
            if dist < 0: return None
            if dist > 10.0: return None
            return float(dist)
        return None
