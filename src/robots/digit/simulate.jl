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
                        controller_mode=:position)
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
        if !isnothing(controller)
            cmd = controller(q, q̇, digit)
            if controller_mode == :position
                if digit.engine == :MuJoCo
                    printstyled("No position control using MuJoCo Physics Engine\n", color=:red)
                    break
                end
                apply_position!(cmd, 0.5*ones(length(cmd)), digit)
            elseif controller_mode == :velocity
                if digit.engine == :MuJoCo
                    printstyled("No position control using MuJoCo Physics Engine\n", color=:red)
                    break
                end
                apply_velocity!(cmd, digit)
            end
        end
        step(digit)
    end
    return Ts, qs, q̇s
end