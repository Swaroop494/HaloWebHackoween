import pybullet as p
import pybullet_data
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
plane = p.loadURDF('plane.urdf')
col = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.15, height=0.4)
vis = p.createVisualShape(p.GEOM_CAPSULE, radius=0.15, length=0.4, rgbaColor=[0.2,0.2,0.2,1])
rid = p.createMultiBody(1,col,vis,[0,0,0.2])
import math
from engine import config
from engine.camera import Camera
cam = Camera()
import pybullet as p
# set pose
cam.render(type('R',(),{'x':0,'y':0,'heading_deg':0})())
