import pybullet as p
import time

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

p.setAdditionalSearchPath('..')
humanoid = p.loadURDF("picklerick.urdf", [0,0,2.0], useFixedBase=False) 

p.setGravity(0,0,-9.81)


while (1):
  p.stepSimulation()
  time.sleep(0.001)