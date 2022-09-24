import RigidBodyDynamics.simulate

function simulate(picklerick::Picklerick, T::Float64; 
                        Δt=1e-3, 
                        real_time=false, controller=nothing, 
                        controller_mode=:position)
    qs = SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}[]
    q̇s = SegmentedVector{JointID, Float64, Base.OneTo{JointID}, Vector{Float64}}[]
    Ts = Float64[]
    for t=0.0:Δt:T
        q, q̇ = get_generalized_coordinates(picklerick)
        qstate, q̇state = update_state!(q, q̇, picklerick)  
        push!(qs, copy(qstate)) 
        push!(q̇s, copy(q̇state)) 
        push!(Ts, t)
        if real_time set_configuration!(picklerick.sim.mvis, configuration(picklerick.sim.state)) end                     
        if !isnothing(controller)
            cmd = controller(q, q̇, picklerick)
            if controller_mode == :position
                apply_position!(cmd, 0.5*ones(length(cmd)), picklerick)
            else
                apply_velocity!(cmd, picklerick)
            end
        end
        picklerick.p.stepSimulation() 
    end
    return Ts, qs, q̇s
end