using Revise 
using HumanoidSim
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

# if !(@isdefined vis)
    vis = Visualizer()
# end
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit(sim)
open(sim.mvis.visualizer)

com_goal = [0.0, 0.0, 0.96]
balance_controller = make_balance_controller(sim, com_goal)
ts, qs, vs = simulate(sim.state, 0.5, balance_controller)
setanimation!(sim.mvis, Animation(sim.mvis, ts, qs))


# render(sim.mvis.visualizer)