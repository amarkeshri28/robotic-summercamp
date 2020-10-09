import pybullet as p
import pybullet_data
import numpy as np
import time
import cv2
from operator import add
from operator import sub

p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]
cube=p.loadURDF("cube_small.urdf")
p.resetBasePositionAndOrientation(cube, [2, 2, 0.05], [0, 0, 0, 0.707])
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
value = 3 #rad/s
maxForce = 100 #Newton
#p.applyExternalForce(car,3,[100,0,0],)
#x=[0,0,0]
y=[0,0,0.5]
z=[0,0,0.25]
qKey = ord('a')
qkey1= ord('c')
rkey = ord('r')
width = 512
height = 512
fov = 60
aspect = width/height
near = 0.02
far = 5

while (1):
    
    carpos1= p.getBasePositionAndOrientation(car)
    linkpos=p.getLinkState(car,8)
    #print(carpos1)
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = value
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
           
            p.stepSimulation()
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -value
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            
            p.stepSimulation()

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel=value
            for joint in range(2, 6):
                if joint%2==0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if(k== p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel=0
            for joint in range(2, 6):
                if joint%2==0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel=value
            for joint in range(2, 6):
                if joint%2!=0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if(k== p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel=0
            for joint in range(2, 6):
                if joint%2!=0:
                    p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()

        if (k==qKey and(v&p.KEY_WAS_RELEASED)):
            value = value+1

        if (k==rkey and(v&p.KEY_WAS_RELEASED)):
            p.resetBaseVelocity(car,linearVelocity=[0,0,0])
            for i in range(100):
                carpos1= p.getBasePositionAndOrientation(car)
                eular_orientation=p.getEulerFromQuaternion(carpos1[1])
                p.resetBasePositionAndOrientation(car,carpos1[0],list(p.getQuaternionFromEuler(list(map(add,eular_orientation,z)))))
                time.sleep(0.001)
                p.stepSimulation()

        if (k==qkey1 and(v&p.KEY_WAS_RELEASED)):
            view_matrix = p.computeViewMatrix(list(map(add,carpos1[0],y)),list(map(add,linkpos[0],z)),[0,0,1])
            projection_matrix= p.computeProjectionMatrixFOV(fov, aspect,near, far)
            image = p.getCameraImage(width,height,view_matrix,projection_matrix,shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rbg_opengl= np.reshape(image[2],(height,width,4))*1./255.
            cv2.imshow('rgb',rbg_opengl)
            cv2.waitKey(0)
            cv2.destroyAllWindows()



p.getContactPoints(car)

p.disconnect()

