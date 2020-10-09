import pybullet as p
import pybullet_data
import time
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
bob1pos = [2,0,0]
bob2pos = [2.1,0,1]
bob3pos = [2.2,0,1]
bob4pos = [2.3,0,1]
pin1pos= [2,0,2]
pin2pos= [2.1,0,2]
pin3pos= [2.2,0,2]
pin4pos= [2.3,0,2]
cubestartorientation= p.getQuaternionFromEuler([0,0,0])
bob1 = p.loadURDF("sphere.urdf",bob1pos,cubestartorientation)
bob2 = p.loadURDF("sphere.urdf",bob2pos,cubestartorientation)
bob3 = p.loadURDF("sphere.urdf",bob3pos,cubestartorientation)
bob4 = p.loadURDF("sphere.urdf",bob4pos,cubestartorientation)
pin1 = p.loadURDF("cube_small.urdf",[pin1pos[0],pin1pos[1],pin1pos[2]],cubestartorientation,useFixedBase=1)
pin2 = p.loadURDF("cube_small.urdf",[pin2pos[0],pin2pos[1],pin2pos[2]],cubestartorientation,useFixedBase=1)
pin3 = p.loadURDF("cube_small.urdf",[pin3pos[0],pin3pos[1],pin3pos[2]],cubestartorientation,useFixedBase=1)
pin4 = p.loadURDF("cube_small.urdf",[pin4pos[0],pin4pos[1],pin4pos[2]],cubestartorientation,useFixedBase=1)
p.createConstraint(pin1,-1,childBodyUniqueId=bob1,childLinkIndex=-1,jointType=p.JOINT_POINT2POINT,jointAxis= [2,0,1] ,parentFramePosition= [0,0,0],childFramePosition= [0,0,1])
p.createConstraint(pin2,-1,childBodyUniqueId=bob2,childLinkIndex=-1,jointType=p.JOINT_POINT2POINT,jointAxis= [2.1,0,1] ,parentFramePosition= [0,0,0],childFramePosition= [0,0,1])
p.createConstraint(pin3,-1,childBodyUniqueId=bob3,childLinkIndex=-1,jointType=p.JOINT_POINT2POINT,jointAxis= [2.2,0,1] ,parentFramePosition= [0,0,0],childFramePosition= [0,0,1])
p.createConstraint(pin4,-1,childBodyUniqueId=bob4,childLinkIndex=-1,jointType=p.JOINT_POINT2POINT,jointAxis= [2.2,0,1] ,parentFramePosition= [0,0,0],childFramePosition= [0,0,1])
for i in range(200):
    p.applyExternalForce(bob1,-1,[-50,0,0],[2,0,0],p.WORLD_FRAME)
while(1):
    p.setRealTimeSimulation(0)
    p.stepSimulation()
    time.sleep(1./240.)
p.disconnect()
