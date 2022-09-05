using Revise 
using HumanoidSim
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

if !(@isdefined vis)
    vis = Visualizer()
end
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit(sim)
open(sim.mvis.visualizer)

posture_controller = make_posture_controller(sim)
ts, qs, vs = simulate(sim.state, 1.0, posture_controller)
setanimation!(sim.mvis, Animation(sim.mvis, ts, qs))