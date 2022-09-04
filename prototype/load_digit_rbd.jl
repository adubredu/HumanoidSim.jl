using Revise 
using HumanoidSim
using HumanoidSim.MeshCat

vis = Visualizer()
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit(sim)
open(sim.mvis.visualizer)