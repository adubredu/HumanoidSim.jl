qbase_pos_x = 1 
qbase_pos_y = 2
qbase_pos_z = 3
qbase_yaw = 4
qbase_pitch = 5
qbase_roll = 6
qleftHipRoll = 7
qleftHipYaw = 8
qleftHipPitch = 9
qleftKnee = 10
qleftShin = 11
qleftTarsus = 12
qleftToePitch = 13
qleftToeRoll = 14
qleftShoulderRoll = 15
qleftShoulderPitch = 16
qleftShoulderYaw = 17
qleftElbow = 18
qrightHipRoll = 19
qrightHipYaw = 20
qrightHipPitch = 21
qrightKnee = 22
qrightShin = 23
qrightTarsus = 24
qrightToePitch = 25
qrightToeRoll = 26
qrightShoulderRoll = 27
qrightShoulderPitch = 28
qrightShoulderYaw = 29
qrightElbow = 30

LeftHipRoll = 1
LeftHipYaw = 2
LeftHipPitch = 3
LeftKnee = 4
LeftToeA = 5
LeftToeB = 6

RightHipRoll = 7
RightHipYaw = 8
RightHipPitch = 9
RightKnee = 10
RightToeA = 11
RightToeB = 12

LeftShoulderRoll = 13
LeftShoulderPitch = 14
LeftShoulderYaw = 15
LeftElbow = 16

RightShoulderRoll = 17
RightShoulderPitch = 18
RightShoulderYaw = 19
RightElbow = 20

LeftShin = 1
LeftTarsus = 2
LeftToePitch = 3
LeftToeRoll = 4
LeftHeelSpring = 5

RightShin = 6
RightTarsus = 7 
RightToePitch = 8
RightToeRoll = 9
RightHeelSpring = 10

NUM_MOTORS = 20
NUM_JOINTS = 10

qall_joints = ["left-hip-roll", "left-hip-yaw", "left-hip-pitch", "left-knee",
"left-shin", "left-tarsus", "left-toe-pitch", "left-toe-roll", "left-shoulder-roll",
 "left-shoulder-pitch", "left-shoulder-yaw", "left-elbow", "right-hip-roll", 
 "right-hip-yaw", "right-hip-pitch", "right-knee", "right-shin", "right-tarsus",
  "right-toe-pitch", "right-toe-roll", "right-shoulder-roll",
  "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]

name_to_index = Dict(
    "left-hip-roll" => qleftHipRoll,
    "left-hip-yaw" => qleftHipYaw, 
    "left-hip-pitch" => qleftHipPitch, 
    "left-knee" => qleftKnee, 
    "right-hip-roll" =>qrightHipRoll, 
    "right-hip-yaw" => qrightHipYaw, 
    "right-hip-pitch" => qrightHipPitch, 
    "right-knee" => qrightKnee,
    "left-shoulder-roll" =>qleftShoulderRoll, 
    "left-shoulder-pitch" => qleftShoulderPitch, 
    "left-shoulder-yaw" => qleftShoulderYaw, 
    "left-elbow" =>qleftElbow, 
    "right-shoulder-roll" => qrightShoulderRoll, 
    "right-shoulder-pitch" => qrightShoulderPitch, 
    "right-shoulder-yaw" => qrightShoulderYaw, 
    "right-elbow" => qrightElbow,
    "left-toe-A" => -1,
    "left-toe-B" => -1, 
    "right-toe-A" => -1,
    "right-toe-B" => -1
)