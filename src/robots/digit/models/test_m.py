import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)
p.setGravity(0,0,-9.81)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
p.setAdditionalSearchPath('..')
humanoid = p.loadURDF("humanoid.urdf", [0.0, 0.0, 4.0], p.getQuaternionFromEuler([1.57,0,0]), useFixedBase=True)  

#p.setPhysicsEngineParameter(numSolverIterations=10)
#p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0) 
while (1):
  p.stepSimulation()
  time.sleep(1/240.0)
