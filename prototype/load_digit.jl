using Revise
using HumanoidSim
using HumanoidSim.PyBullet 

p = pybullet
pyplan = pybullet_planning

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeID = p.loadURDF("plane.urdf")
digit = load_digit(p)

for i=1:10000
    p.stepSimulation()
    # q, q̇ = get_generalized_coordinates(digit)
    # @show q[1:3]
    sleep(1.0/240.0)
end

p.disconnect()