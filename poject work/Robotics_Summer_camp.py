#!/usr/bin/env python
# coding: utf-8

# In[ ]:


# Import required packages
import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import matplotlib.pyplot as plt


p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -30)

huskypos = [0, 0, 1]

# Load URDFs
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
cheetah = p.loadURDF("mini_cheetah/mini_cheetah.urdf",-5,+2, huskypos[2])

target= p.loadURDF("block0.urdf",3, +2, huskypos[2])


maxForce = 55#Force of car #Newton.m
dist_init=0#initial distance
f=50#force of cheetah #Newton.m
m1=0.01
d=50
s=0.15# distance travelled by cheetah for calling runcheetah() once
l=0
l1=0 #adjust the time.sleep(l1) 
# Function to set cheetah's initial orientation(need was understood by experiment)
def setPos():
    l2=0.1
    n=0
    while(n<d):

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=8,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=9,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=10,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=12,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=13,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=14,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l2,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=0,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=1,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=2,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=4,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=5,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=6,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        n+=1
        p.stepSimulation()
# Function to run the cheetah(representing villain here)
def runCheetah():
    fr_t= p.getLinkState(cheetah, 3)
    fl_t= p.getLinkState(cheetah, 7)
    hr_t= p.getLinkState(cheetah, 11)
    hl_t= p.getLinkState(cheetah, 15)

    k=0
    n=0
    while(k<s):

        
        fr= p.calculateInverseKinematics(cheetah, 3, [fr_t[0][0]+k, fr_t[0][1], fr_t[0][2]])
        fl= p.calculateInverseKinematics(cheetah, 7, [fl_t[0][0]+k, fl_t[0][1], fl_t[0][2]])
        hr= p.calculateInverseKinematics(cheetah, 11, [hr_t[0][0]-k, hr_t[0][1], hr_t[0][2]])
        hl= p.calculateInverseKinematics(cheetah, 15, [hl_t[0][0]-k, hl_t[0][1], hl_t[0][2]])
        
        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=8,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hr[6],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=9,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hr[7],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=10,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hr[8],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=12,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hl[9],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=13,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hl[10],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=14,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=hl[11],
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=0,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=1,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=2,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=4,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=5,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=6,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        k+=m1

        p.stepSimulation()
        time.sleep(l1)
    
    n=0
    while(n<d):

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=8,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=9,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=10,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=12,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=13,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=14,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=l,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=0,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=1,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=2,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=4,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=5,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        p.setJointMotorControl2(bodyIndex=cheetah,
                                jointIndex=6,
                                controlMode =p.POSITION_CONTROL,
                                targetPosition=0,
                                force=f)

        n+=1
        p.stepSimulation()
        
        time.sleep(l1)
# Calculate the magnitude of a vector        
def mag(vec_A):
    magnitude= ((vec_A[0])**2 + (vec_A[1])**2 + (vec_A[2])**2)**(0.5)
    return magnitude
# Dot product of two vectors
def dotProduct(vect_A, vect_B): 
  
    product = 0
  
    # Loop for calculate cot product 
    for i in range(0,3): 
        product = product + vect_A[i] * vect_B[i] 
  
    return product 
# Cross product of two vectors
def crossProduct(vect_A, vect_B): 
    cross_P=[]  
    cross_P.append(vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1]) 
    cross_P.append(vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2]) 
    cross_P.append(vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0]) 
    
    return cross_P
# Calculate the speed correction using PID
def speedCorrection(vec_A, vec_B, last_error, m):
    mag_A= mag(vec_A)
    mag_B= mag(vec_B)
    kp= 10*m
    kd= 0.6* kp
    dot= dotProduct(vec_A, vec_B)
    cross= crossProduct(vec_A, vec_B)
    z= cross[2]
    sin= z/(mag_A*mag_B)
    cos= dot/(mag_A*mag_B)
    #theta= (cross[2]/abs(cross[2]))*np.arcsin(sin)
    if cos<0:
        theta= (z/abs(z))*np.pi - np.arcsin(sin)
    else:
        theta= np.arcsin(sin)
    error= theta
    if error<-0.5 and error>=-np.pi:
        speed= -sp
    elif error>0.5 and error<np.pi:
        speed= sp
    else:
        speed= kp*error + kd*(error - last_error)
    return (speed,error)
# Function to carry out movement in Auto mode
def turn(speed, sp, sx):
    print("turn")
    baseSpeed = sp
    if baseSpeed!=0:
        targetVel_R = baseSpeed + speed + sx
        targetVel_L = baseSpeed - speed + sx
    else:
        targetVel_R = 0
        targetVel_L = 0
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
    for joint in range(1,3):
        p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
    p.stepSimulation()

'''
tune the kp and kd from experiments, 
a hint set kd = 10*kp
'''

last_error=0
PID_CONTROL=False
h1=0
h2=0
setPos()
b= 0
while (1):
    keys = p.getKeyboardEvents()
    
    if b==0:
        if (PID_CONTROL):
            # calculate the speed of villain in unit/footstep
            #########
            #if h1==0:
            h1= p.getLinkState(cheetah, 0)
            runCheetah()
            #if h2==0:
            h2= p.getLinkState(cheetah, 0)
            h= mag([h1[0][0]-h2[0][0], h1[0][1]-h2[0][1], h1[0][2]-h2[0][2]])
            ##############


            rear_bump= p.getLinkState(husky,9)
            front_bump= p.getLinkState(husky,8)
            target_pos= p.getBasePositionAndOrientation(target)
            ###############
            h3= p.getLinkState(cheetah, 0)
            enemy_dist= mag([h3[0][0]-target_pos[0][0],  h3[0][0]-target_pos[0][0], 0])
############# I f eenemy comes in a distance of 4.5 unit then the car will automatically start and will come before the enemy how much far it is###########
            if enemy_dist<6:
                rear_bump= p.getLinkState(husky,9)
                front_bump= p.getLinkState(husky,8)
                v1= front_bump[0][0]-rear_bump[0][0]
                v2= front_bump[0][1]-rear_bump[0][1]
                v3= front_bump[0][2]-rear_bump[0][2]
                frontVec= np.array(list((v1, v2, v3)))
                t1= target_pos[0][0]-rear_bump[0][0]
                t2= target_pos[0][1]-rear_bump[0][1]
                t3= 0
                targetVec= np.array(list((t1, t2, t3)))
                if dist_init==0:
                    dist_init= mag(targetVec)
                    sp= int((dist_init-1.5)*10)# initial speed depend on initial distance
                    m= (sp//10)-1 # To set kp and kd according to the basespeed
                if mag(targetVec)>3:
                    sp= int((mag(targetVec)-1.5)*10)
                    m= (sp//10)-1 # To set kp and kd according to the basespeed
                else:
                    sp=0
                sx= (mag(targetVec)/3)*h-1# Dependence on speed of enemy
                print(sp)
                speed_correction, last_error=speedCorrection(targetVec, frontVec, last_error, m) 
                turn(speed_correction, sp, sx)

        for k, v in keys.items():
            if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = 20
                for joint in range(2,6):
                    p.setJointMotorControl2(husky,joint, p.VELOCITY_CONTROL, targetVelocity =targetVel, force = maxForce)

                p.stepSimulation()

            if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED) and PID_CONTROL==False):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = -20
                for joint in range(2,6):
                    p.setJointMotorControl2(husky,joint, p.VELOCITY_CONTROL, targetVelocity =targetVel, force = maxForce)

                p.stepSimulation()

            if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED) and PID_CONTROL==False):
                targetVel = 0
                for joint in range(2, 6):
                    p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)

            if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN) and PID_CONTROL==False):
                targetVel = 5
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
                targetVel = 5
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
            ### Press c to start auto mode
            if (k == ord('c') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-on/Auto mode on")
                PID_CONTROL = True
            if (k == ord('r') and (v & p.KEY_WAS_TRIGGERED)):
                print("\nPID Control-off ,back to manual")
                PID_CONTROL = False
            ### Press m to change mode            
            if (k == ord('m') and (v & p.KEY_WAS_TRIGGERED)):
                b=~b
                print("\nVisual mode")
            ### Press p to take a picture from driver's view     
            if (k == ord('p') and (v & p.KEY_IS_DOWN)):
                rear_bump= p.getLinkState(husky,9)
                front_bump= p.getLinkState(husky,8)
                v1= front_bump[0][0]-rear_bump[0][0]
                v2= front_bump[0][1]-rear_bump[0][1]
                v3= front_bump[0][2]-rear_bump[0][2]
                frontVec= np.array(list((v1, v2, v3)))
                t= (frontVec[0]**2 + frontVec[1]**2 + frontVec[2]**2)**(0.5)
                frontDrn= frontVec/t
                s1=20
                eyePos= [front_bump[0][0], front_bump[0][1], front_bump[0][2]]
                targetPos= list(frontDrn*s1)
                upVec= [0,0,1]
                view_matrix= p.computeViewMatrix(eyePos, targetPos, upVec)

                width= 512
                height= 512
                aspect_ratio= width/height
                near= 0.1
                far= 10
                fov= 120
                projection_matrix= p.computeProjectionMatrixFOV(fov, aspect_ratio, near, far)

                images= p.getCameraImage(width, height, view_matrix, projection_matrix, shadow=True, renderer= p.ER_BULLET_HARDWARE_OPENGL)
                image= images[2].reshape((height, width, 4))
                image= cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA) 
                cv2.imshow('Driver_View', image)
                cv2.waitKey(0)
                cv2.destroyAllWindows()
    ## Visual instructing mode(a little slower due to waitkey, background should be light coloured than instrucor's hand)
    else:
        #update your droid cam code for better control and visualisation
        cap= cv2.VideoCapture("http://192.168.43.53:4747/video")
        
        while(True):
            _,image= cap.read()
            #gray= cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            #mask= cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 641, 2)
            #mask= cv2.erode(mask, (5,5), iterations=10)
            image=cv2.cvtColor(image,cv2.COLOR_BGR2RGB) 
            image=cv2.rotate(image,cv2.ROTATE_90_CLOCKWISE)
            lower=np.array([80,60,50])
            high=np.array([165,126,110])
            mask=cv2.inRange(image,lower,high)
            length= mask.shape[1]
            bredth= mask.shape[0]
            ex= length//3
            screen1= mask[:, 0:ex]
            screen2= mask[:, ex:(length-ex)]
            screen3= mask[:, (length-ex):]
            c1,_= cv2.findContours(screen1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            c2,_= cv2.findContours(screen2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            c3,_= cv2.findContours(screen3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            
                
            if len(c1)!=0:
                c= sorted(c1, key= cv2.contourArea, reverse= True)[0]
                a1= cv2.contourArea(c)
            else:
                a1=0
            if len(c2)!=0:
                c= sorted(c2, key= cv2.contourArea, reverse= True)[0]
                a2= cv2.contourArea(c)
            else:
                a2=0
            if len(c3)!=0:
                c= sorted(c3, key= cv2.contourArea, reverse= True)[0]
                a3= cv2.contourArea(c)
            else:
                a3=0
            a4= max(a1, a2, a3)
            if a4==a3:
                ##speed= 40##((sp+sx)/3)*2
                speed1= 160
                speed2= -160
                print("R")

            elif a4==a1:
                #speed= -40#-((sp+sx)/3)*2
                speed1= -160
                speed2= 160
                print("L")
            else:
                speed1= 160
                speed2= 160
                print("F")
            targetVel_R = speed1
            targetVel_L = speed2
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2* joint, p.VELOCITY_CONTROL, targetVelocity =targetVel_R,force = maxForce)
            for joint in range(1,3):
                p.setJointMotorControl2(husky,2* joint+1, p.VELOCITY_CONTROL,targetVelocity =targetVel_L,force = maxForce)
            p.stepSimulation()
            mask[:,ex]=255
            mask[:, length-ex]=255
            cv2.imshow("mask", mask)
            k= cv2.waitKey(1)
            if k==27:
                b=0
                cap.release()
                cv2.destroyAllWindows()
                break
p.disconnect()


