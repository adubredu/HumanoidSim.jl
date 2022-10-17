import RigidBodyDynamics.simulate
import Base.step

function step(digit::Digit)
    if digit.engine == :PyBullet
        digit.p.stepSimulation()
    elseif digit.engine == :MuJoCo
        digit.p.step()
    end
end

function simulate(digit::Digit, T::Float64; 
                        Δt=1e-3, 
                        real_time=false, controller=nothing, 
                        controller_mode=:position, data=nothing,
                        env=nothing)
    qs = SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}[]
    q̇s = SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}[]
    Ts = Float64[]
    for t=0.0:Δt:T
        q, q̇ = get_generalized_coordinates(digit)
        qstate, q̇state = update_state!(q, q̇, digit)  
        push!(qs, copy(qstate)) 
        push!(q̇s, copy(q̇state)) 
        push!(Ts, t)
        if real_time set_configuration!(digit.sim.mvis, configuration(digit.sim.state)) end                     
        if !isnothing(env)
            for object in env.objects
                f = env.dynamics[object]
                f(object, t, env)
            end
        end
        if !isnothing(controller)
            cmd = controller(q, q̇, digit;data=data) 
            if controller_mode == :position
                if digit.engine == :MuJoCo
                    printstyled("No position control using MuJoCo Physics Engine\n", color=:red)
                    break
                end
                apply_position!(cmd, 0.5*ones(length(cmd)), digit)
            elseif controller_mode == :velocity
                if digit.engine == :MuJoCo
                    printstyled("No velocity control using MuJoCo Physics Engine\n", color=:red)
                    break
                end
                apply_velocity!(cmd, digit)
            elseif controller_mode == :torque
                if digit.engine == :PyBullet
                    printstyled("No torque control using PyBullet Physics Engine\n", color=:red)
                    break
                end
                apply_torque!(cmd, digit)
            end
        end
        step(digit)
    end
    return Ts, qs, q̇s, env
end

function MeshCat.Animation(mvis::MechanismVisualizer,
    times::AbstractVector{<:Real},
    configurations::AbstractVector{<:AbstractVector{<:Real}},
    env::Env;
    fps::Integer=30)
@assert axes(times) == axes(configurations)
interpolated_configurations = interpolate((times,), configurations, Gridded(Interpolations.Linear()))
env_interps = Dict()
for obj in env.objects
    env_interps[obj] = interpolate((times,), env.trajectories[obj], Gridded(Interpolations.Linear()))
end
animation = Animation()
num_frames = floor(Int, (times[end] - first(times)) * fps)
for frame in 0 : num_frames
    time = first(times) + frame / fps
    let mvis = mvis, interpolated_configurations = interpolated_configurations, time=time
        atframe(animation,  frame) do
            set_configuration!(mvis, interpolated_configurations(time))
            for obj in env.objects
                settransform!(mvis.visualizer[obj], Translation(env_interps[obj](time)...))
            end
        end
    end
end
return animation
end