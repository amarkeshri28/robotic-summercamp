import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
from operator import add
#import matplotlib.pyplot as plt
p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)

huskypos = [0, 0, 0.1]
target_block=p.loadURDF("block.urdf",2,0,0)

husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
p.createConstraint(husky, -1, -1, -1, p.JOINT_POINT2POINT, [0, 1, 0], [0, 0,0 ], [0, 0,0])


maxForce = 200 #Newton.m
#camera should be facing in the direction of the car


def turn(speed):
    baseSpeed = 100
    targetVel_R = baseSpeed + speed
    targetVel_L = baseSpeed - speed
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    p.stepSimulation()
'''
tune the kp and kd from experiments, 
a hint set kd = 10*kp
'''
Kp=1.3
Kd=13
last_error=0
PID_CONTROL=False


    
while (1):
    keys = p.getKeyboardEvents()
    if (PID_CONTROL):
    # 1. Get the image feed, as in subpart-1
        y=[0,0,0.55]
        z=[0,0,0.25]
        width = 512
        height = 512
        fov = 60
        aspect = width/height
        near = 0.02
        far = 5
        carpos1= p.getBasePositionAndOrientation(husky)
        linkpos=p.getLinkState(husky,8)
        view_matrix = p.computeViewMatrix(list(map(add,carpos1[0],y)),list(map(add,linkpos[0],z)),[0,0,1])
        projection_matrix= p.computeProjectionMatrixFOV(fov, aspect,near, far)
        image = p.getCameraImage(width,height,view_matrix,projection_matrix,shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb=image[2]
        hsv1=np.array([50,100,100])
        hsv2=np.array([80,255,255])
        img=cv2.cvtColor(rgb,cv2.COLOR_RGB2HSV)
        mask=cv2.inRange(img,hsv1,hsv2)
        img2=cv2.bitwise_and(rgb,rgb,mask=mask)
        counter,_=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        for c in counter:
            area=cv2.contourArea(c)
            if area>1:
                M=cv2.moments(c)
                if M["m00"]!=0:
                    aa=1
                    cx=int(M["m10"]/M["m00"])
                    cy=int(M["m01"]/M["m00"])
        if aa==0:
            cx=0
    	# 2. using the above limits, mask the green colour 
      	#    and draw the area with maximuma contour and find 
      	#    its moments.
      	# 3. Calculate the error, and apply pid control to find the
      	#    optimal speed correction and call the turn function with
      	#    that speed.

      	# apply pid law to calulate this value,use Kp and Kd variable above
      	# and tune em properly.
         
        error = cx-255/2#find error
        speed_correction = kd*(error-last_error)+kp*error
        turn(speed_correction)
        last_error=error #initialize accordingly
        k=cv2.waitKey(1) & 0xFF
    for k, v in keys.items():

            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2*joint+1, p.VELOCITY_CONTROL,targetVelocity =-1*targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)and PID_CONTROL==False):
                targetVel = 2
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
                for joint in range(1,3):
                    p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL,targetVelocity =-1* targetVel,force = maxForce)

                p.stepSimulation()
            if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)

                p.stepSimulation()
            if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-on")
                PID_CONTROL = True
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-off ,back to manual")
                PID_CONTROL = False
p.disconnect()
