import pybullet as p
import pybullet_data
import time
i=int(input("Enter 1 for torque control  or 2 for velocity control"))

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
jointnum=p.getNumJoints(husky)
print(jointnum)
for j in range(jointnum):
    print(p.getJointInfo(husky,j))



#function to be filled to implement torque control
def Torque_control():

	# find this value to climb the ramp without sliping and topling
	optimal_torque_value = -250

	for i in range(2,6):
		p.setJointMotorControl2(husky,i,p.TORQUE_CONTROL, targetVelocity=100,force=optimal_torque_value)
	
	



#function to be filled to implement velocity control
def Velocity_control():
	maxForce = -10
	optimal_velocity_value = 10
	for i in range(2,6):
		
		p.setJointMotorControl2(husky,i,p.VELOCITY_CONTROL ,targetVelocity=optimal_velocity_value,force=maxForce)

t=0
while(1):
	t=t+1
	time.sleep(.01)
	if i==1:
		Torque_control()
		p.stepSimulation()
		
	if i==2:
		Velocity_control()
		p.stepSimulation()
		
	if t%100==0:
		print(p.getLinkState(husky,0))
		print(p.getBaseVelocity(husky))
	

p.disconnect()

