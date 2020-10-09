import pybullet as p
import pybullet_data
import os
import time
import math

file_name = "2R_planar_robot.urdf"
p.connect(p.GUI)

p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(file_name)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.05], [0, 0, 0, 0.707])

p.setGravity(0,0,-10)
point_A = [0,-math.sqrt(3),1]
point_B = [0,math.sqrt(3),1]
p.addUserDebugLine(point_A,point_B,[1,0,0],2)
l1=1
l2=1
z=1
y=-math.sqrt(3)
dy=0.0001
while(True):

	if(l1+l2>=math.sqrt(y*y+z*z)):
		jointpos=p.calculateInverseKinematics(robot,2,[0,y,z])
		for i in range(2):
				 p.setJointMotorControl2(bodyIndex=robot,
								jointIndex=i,
								controlMode =p.POSITION_CONTROL,
								targetPosition=jointpos[i],
								force=500)

		p.stepSimulation()
		y=y+dy
	elif(l1+l2<math.sqrt(y*y+z*z)):
		y=-math.sqrt(3)
p.disconnect()