mutable struct PicklerickSim 
    vis::Visualizer 
    mvis::Union{MechanismVisualizer, Nothing}
    state::Union{MechanismState, Nothing} 
    θᵣ::Union{Vector{Float64}, Nothing} 
    com_goal
    θ_min
    θ_max

    function PicklerickSim(vis::Visualizer) 
        θ_min = [-1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.79e+308, -1.0472, -0.698, -1.0472, -1.2392, -0.35, -0.8779, -0.785, -0.6109, -1.309, -2.5307, -1.7453, -1.3526, -1.0472, -0.698, -1.57, -0.8727, -0.35, -1.2497, -0.785, -0.6109, -1.309, -2.5307, -1.7453, -1.3526]
        θ_max = [1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308, 1.79e+308,  1.0472, 0.698, 1.57, 0.8727, 0.35, 1.2497, 0.785, 0.6109, 1.309, 2.5307, 1.7453, 1.3526, 1.0472, 0.698, 1.0472, 1.2392, 0.35, 0.8779, 0.785, 0.6109, 1.309, 2.5307, 1.7453, 1.3526]
        new(vis, nothing, nothing, nothing, nothing, θ_min, θ_max)
    end
end

struct Picklerick
    id::Int64
    p
    joint_names
    joint_ids
    sim::PicklerickSim

    function Picklerick(id, p, sim)
        joint_names = SA["right_shoulder_pitch", "right_shoulder_yaw", 
            "right_elbow_yaw", "left_shoulder_pitch", "left_shoulder_yaw", 
            "left_elbow_yaw", "right_hip_pitch", "right_thigh_yaw", 
            "right_shin_yaw", "left_hip_pitch", "left_thigh_yaw", "left_shin_yaw"]
        joint_ids = []
        new(id, p, joint_names, joint_ids, sim)
    end
end

