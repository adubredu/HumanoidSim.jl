import pybullet as p
import pybullet_planning as pyplan
import time

import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

p.setAdditionalSearchPath('..')
humanoid = p.loadURDF("picklerick.urdf", [0,0,2.0], useFixedBase=False) 

p.setGravity(0,0,-9.81)

joint_names = ["right_shoulder_pitch", "right_shoulder_yaw", 
            "right_elbow_yaw", "left_shoulder_pitch", "left_shoulder_yaw", 
            "left_elbow_yaw", "right_hip_pitch", "right_thigh_yaw", 
            "right_shin_yaw", "left_hip_pitch", "left_thigh_yaw", "left_shin_yaw"]
joint_ids = [pyplan.get_joint(humanoid, name) for name in joint_names]
print(joint_ids)

# while (1):
#   p.stepSimulation()
#   time.sleep(0.001)