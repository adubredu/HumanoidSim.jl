struct Digit
    id::Int64
    p
    joint_names
    joint_ids

    function Digit(id, p)
        joint_names = SA["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
            "left-knee", "left-shin", "left-tarsus", "left-toe-pitch", 
            "left-toe-roll", "left-shoulder-roll", "left-shoulder-pitch", 
            "left-shoulder-yaw", "left-elbow", "right-hip-roll", "right-hip-yaw", 
            "right-hip-pitch", "right-knee", "right-shin", "right-tarsus", 
            "right-toe-pitch", "right-toe-roll", "right-shoulder-roll", 
            "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        joint_ids = [14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 22, 23, 24, 25, 26, 27, 28, 29, 6, 7, 8, 9]
        new(id, p, joint_names, joint_ids)
    end
end


mutable struct DigitSim 
    vis::Visualizer 
    mvis::Union{MechanismVisualizer, Nothing}
    state::Union{MechanismState, Nothing} 
    left_hand_ee::Union{Point3D, Nothing}
    right_hand_ee::Union{Point3D, Nothing}
    left_foot_ee::Union{Point3D, Nothing}
    right_foot_ee::Union{Point3D, Nothing}
    pelvis::Union{Point3D, Nothing}
    left_foot_contact_points::Union{Vector{Point3D}, Nothing}
    right_foot_contact_points::Union{Vector{Point3D}, Nothing}
    θᵣ::Union{Vector{Float64}, Nothing} 
    θ::Union{Vector{Float64}, Nothing}
    θ̇ ::Union{Vector{Float64}, Nothing}
    joints::Union{Vector, Nothing} 
    Δt::Float64
    g_left_ee::Union{Point3D, Nothing}
    g_right_ee::Union{Point3D, Nothing}
    world_frame 
    leg_indices 
    init_config
    com_goal

    function DigitSim(vis::Visualizer)
        Δt = 10E-3 
        leg_indices = [7, 9, 10, 19,21,22]
        new(vis, nothing, nothing, nothing, nothing, nothing, nothing, nothing, nothing, 
        nothing, nothing, nothing, nothing, [], Δt, nothing, nothing, nothing, leg_indices, nothing, nothing)
    end
end