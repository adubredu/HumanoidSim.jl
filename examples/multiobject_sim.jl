using Revise
using HumanoidSim 
using HumanoidSim.MeshCat
using HumanoidSim.RigidBodyDynamics
using HumanoidSim.GeometryBasics
using HumanoidSim.CoordinateTransformations
using HumanoidSim.Colors

vis = Visualizer()
initialize_arena!(vis)
sim = DigitSim(vis)
load_digit_vis(sim)

object = :missile
object2 = :missile2
init_position = [5.0, 0.0, 0.5]
geom = GeometryBasics.Sphere(GeometryBasics.Point(init_position...), 0.1)
setobject!(sim.mvis.visualizer[object], geom, 
            MeshPhongMaterial(color=RGBA(0.0, 1.0, 0.0, 0.5)))
setobject!(sim.mvis.visualizer[object2], geom, 
            MeshPhongMaterial(color=RGBA(1.0, 0.0, 0.0, 0.5)))
# env
function dynamics(obj, t, env)
    if isempty(env.trajectories[obj]) 
        prev = env.init_position[obj]
    else
        prev = last(env.trajectories[obj])
    end
    cur = prev + 0.005*([-1.0, 0.0, 0.0])
    push!(env.trajectories[obj], cur)
end

env = Env([object, object2])
env.dynamics[object] = dynamics 
env.dynamics[object2] = dynamics  
env.init_position[object] = init_position
env.init_position[object2] = [5.0, 1.0, 1.0]

engine = :MuJoCo # engine = :PyBullet 
digit = load_digit(sim; engine=engine)

open(digit.sim.mvis.visualizer) 
 
Ts, qs, q̇s = simulate(digit, 5.0; Δt=1e-3, env=env) 
 
if engine == :PyBullet digit.p.disconnect() end

# # setting animation
println("***setting animation***")  
setanimation!(digit.sim.mvis, Animation(digit.sim.mvis, Ts, qs, env))