using Revise 
using HumanoidSim
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

vis = Visualizer()
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit(sim)
open(sim.mvis.visualizer)

ts, qs, vs = simulate(sim.state, 2.0)
setanimation!(sim.mvis, Animation(sim.mvis, ts, qs))