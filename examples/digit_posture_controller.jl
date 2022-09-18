using Revise
using HumanoidSim
using HumanoidSim.PyBullet 
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics

vis = Visualizer()
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit_vis(sim)

engine = :PyBullet
digit = load_digit(sim; engine=engine)
 
open(digit.sim.mvis.visualizer) 
Ts, qs, q̇s = simulate(digit, 5.0; Δt=1e-3, 
        controller=posture_position_controller, controller_mode=:position) 

if engine == :PyBullet digit.p.disconnect() end

# setting animation
println("***setting animation***") 
setanimation!(digit.sim.mvis, Animation(digit.sim.mvis, Ts, qs))