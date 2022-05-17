import pybullet as p
import time
import math
import pybullet_data
import ray_trace as ray
import create_cylinder as cylinder

useGui = True
def getCenterAndOrient(points):
    mid_pt=[0,0,0]
    # points =[[0,0,0],[1,1,1]]
    line=p.addUserDebugLine(points[0], points[1],lineColorRGB =[1,0,0],lineWidth =1000)
    pose=p.getBasePositionAndOrientation(line)
    for i in range(3):
        mid_pt[i]=(points[0][i]+points[1][i])/2
    return mid_pt, pose[1]


def point_wrt_world(points):
    scale=.1
    tree_orient =p.getQuaternionFromEuler([0,0,1.54])
    new_pt=[]
    for i in range(len(points)):
        pt=p.multiplyTransforms([0,0,0],tree_orient,[point[i][0]*scale,point[i][1]*scale,point[i][2]*scale],[0,0,0,1])
        print("worlds",pt)
        new_pt.append(pt[0])
    return new_pt

if (useGui):
  p.connect(p.GUI)
else:
  p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

robot = p.loadURDF("tree.urdf")
cy = p.loadURDF("cylinder.urdf")
center_pt = [1.365,1.065,.99]
length=.01
radius= .03
# cyliner_id =cylinder.create_cylinder(center_pt, length,radius)
point=[[.49,.07,5.61],[.53719,.092,5.52]]
point_w=point_wrt_world(point)
print(point, point_w)
center_pt, orient=getCenterAndOrient(point_w)
quater=p.getBasePositionAndOrientation(cy)
val=p.getQuaternionFromEuler([60*math.pi/180,90*math.pi/180,90*math.pi/180])
ray.ray_trace(useGui, center_pt, length, orient, cy)
