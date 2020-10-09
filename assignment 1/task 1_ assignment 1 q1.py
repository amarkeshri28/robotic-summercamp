import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("dabba.urdf",cubeStartPos, cubeStartOrientation)



    

while True:
    t=0
    while t<=9.8:

      if t<9.8:
              p.setGravity(t/pow(2,1/2),t/pow(2,1/2),0)
              p.stepSimulation()

              time.sleep(1./240.)
              t+=0.001;
      else:
              break;
 
      
      
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
