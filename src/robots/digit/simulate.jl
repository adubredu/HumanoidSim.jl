using RigidBodyDynamics:simulate

function simulate(digit::Digit, T::Float64; Δt=1e-3, real_time=false)
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
        digit.p.stepSimulation() 
    end
    return Ts, qs, q̇s
end