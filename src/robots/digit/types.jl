mutable struct DigitSim 
    vis::Visualizer 
    mvis::Union{MechanismVisualizer, Nothing}
    state::Union{MechanismState, Nothing} 
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
    θ_min
    θ_max

    function DigitSim(vis::Visualizer)
        Δt = 10E-3 
        leg_indices = [7, 9, 10, 19,21,22]
        θ_min = [-1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.0472, -0.698, -1.0472, -1.2392, -0.35, -0.8779, -0.785, -0.6109, -1.309, -2.5307, -1.7453, -1.3526, -1.0472, -0.698, -1.57, -0.8727, -0.35, -1.2497, -0.785, -0.6109, -1.309, -2.5307, -1.7453, -1.3526]
        θ_max = [1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308,  1.0472, 0.698, 1.57, 0.8727, 0.35, 1.2497, 0.785, 0.6109, 1.309, 2.5307, 1.7453, 1.3526, 1.0472, 0.698, 1.0472, 1.2392, 0.35, 0.8779, 0.785, 0.6109, 1.309, 2.5307, 1.7453, 1.3526]
        new(vis, nothing, nothing, nothing, nothing, nothing, [], Δt, nothing, nothing, nothing, leg_indices, nothing, nothing, θ_min, θ_max)
    end
end

struct Digit
    id::Int64
    p
    joint_names::SVector{24, String}
    motor_names::SVector{20, String}
    joint_ids
    sim::DigitSim
    engine::Symbol

    function Digit(id, p, sim, engine)
        joint_names = SA["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
            "left-knee", "left-shin", "left-tarsus", "left-toe-pitch", 
            "left-toe-roll", "left-shoulder-roll", "left-shoulder-pitch", 
            "left-shoulder-yaw", "left-elbow", "right-hip-roll", "right-hip-yaw", 
            "right-hip-pitch", "right-knee", "right-shin", "right-tarsus", 
            "right-toe-pitch", "right-toe-roll", "right-shoulder-roll", 
            "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        motor_names = SA["left-hip-roll", "left-hip-yaw", "left-hip-pitch", 
        "left-knee", "left-toe-A", "left-toe-B",  "right-hip-roll", "right-hip-yaw", 
        "right-hip-pitch", "right-knee", "right-toe-A", "right-toe-B", "left-shoulder-roll", "left-shoulder-pitch", 
        "left-shoulder-yaw", "left-elbow", "right-shoulder-roll", 
        "right-shoulder-pitch", "right-shoulder-yaw", "right-elbow"]
        joint_ids = [14, 15, 16, 17, 18, 19, 20, 21, 0, 1, 2, 3, 23, 24, 25, 26, 27, 28, 29, 30, 6, 7, 8, 9]
        new(id, p, joint_names, motor_names, joint_ids, sim, engine)
    end
end

