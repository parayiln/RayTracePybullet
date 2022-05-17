import pybullet as p
import time
import math
import pybullet_data
import ray_trace as ray

def create_cylinder(center, len, rad):
    visualShapeId =p.createVisualShape(shapeType=p.GEOM_CYLINDER,radius=rad,length=len, rgbaColor=[0,0,1,1])
    collisionShapeId=p.createCollisionShape(shapeType=p.GEOM_CYLINDER,radius=rad,height=len)
    # rangex = 1
    # rangey = 1
    # for i in range(rangex):
    #   for j in range(rangey):
    id =p.createMultiBody(baseMass=1,
                      baseInertialFramePosition=[0, 0, 0],
                      baseCollisionShapeIndex=collisionShapeId,
                      baseVisualShapeIndex=visualShapeId,
                      basePosition=[0,0,0],
                      useMaximalCoordinates=True)


    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    return id
