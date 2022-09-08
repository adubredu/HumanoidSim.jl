import RigidBodyDynamics.simulate

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
                apply_position!(cmd, 0.5*ones(length(cmd)), digit)
            else
                apply_velocity!(cmd, digit)
            end
        end
        digit.p.stepSimulation() 
    end
    return Ts, qs, q̇s
end