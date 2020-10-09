import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
def Fibonacci(n): 
    if n<0: 
        print("Incorrect input") 
    # First Fibonacci number is 0 
    elif n==1: 
        return 0
    # Second Fibonacci number is 1 
    elif n==2: 
        return 1
    else: 
        return Fibonacci(n-1)+Fibonacci(n-2) 

n=2
t=-5

while True:
  x=0
 
  cso= p.getQuaternionFromEuler([0,0,0])
  y=Fibonacci(n)
  n=n+1
 
 
  for q in range(y):

      cp = [x,t,5]
      x=x+0.5
      boxId = p.loadURDF("sphere.urdf",cp, cso)
   
  for i in range (10):
      p.stepSimulation()
      time.sleep(1./240.)
  t=t+1
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
  

  
# Driver Program 
  
print(Fibonacci(9)) 
  
