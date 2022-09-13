using Revise
using HumanoidSim
using HumanoidSim.PyBullet 
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

vis = Visualizer()
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit_vis(sim)

p = pybullet
pyplan = pybullet_planning

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeID = p.loadURDF("plane.urdf")
digit = load_digit(p, sim)
# joint_ids = [pyplan.get_joint(digit.id, name) for name in digit.joint_names]
# @show joint_ids
open(digit.sim.mvis.visualizer) 
Ts, qs, q̇s = simulate(digit, 5.0; Δt=1e-3, 
        controller=balance_position_controller, controller_mode=:position) 

p.disconnect()

# setting animation
println("***setting animation***") 
setanimation!(digit.sim.mvis, Animation(digit.sim.mvis, Ts, qs))