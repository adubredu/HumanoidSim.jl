using Revise
using HumanoidSim
using HumanoidSim.PyBullet 
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

vis = Visualizer()
initialize_arena!(vis)
sim = PicklerickSim(vis)
load_picklerick_vis(sim)

p = pybullet
pyplan = pybullet_planning

p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
planeID = p.loadURDF("plane.urdf")
picklerick = load_picklerick(p, sim)

open(picklerick.sim.mvis.visualizer) 
Ts, qs, q̇s = simulate(picklerick, 5.0; Δt=1e-3) 

p.disconnect()

# setting animation
println("***setting animation***")  
setanimation!(picklerick.sim.mvis, Animation(picklerick.sim.mvis, Ts, qs))