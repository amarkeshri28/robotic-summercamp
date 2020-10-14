import pybullet as p
import pybullet_data
import time
from operator import add


p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
carpos = [0, 0, 0.1]

car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
value = 3 #rad/s
maxForce = 100 #Newton
#p.applyExternalForce(car,3,[100,0,0],)
rkey=ord('r')
qKey = ord('a')
z=[0,0,0.05]

while (1):
    
    
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
        if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:
            value = value+1
        if (k==rkey and(v&p.KEY_WAS_RELEASED)):
            p.resetBaseVelocity(car,linearVelocity=[0,0,0])
            for i in range(100):
                carpos1= p.getBasePositionAndOrientation(car)
                eular_orientation=p.getEulerFromQuaternion(carpos1[1])
                p.resetBasePositionAndOrientation(car,carpos1[0],list(p.getQuaternionFromEuler(list(map(add,eular_orientation,z)))))
                time.sleep(0.00001)
                p.stepSimulation()
        



p.getContactPoints(car)

p.disconnect()
