function update_state!(q::Vector{Float64},  q̇::Vector{Float64}, digit::Digit)
    sim = digit.sim
    state = sim.state
    mechanism = state.mechanism
    floating_base = first(out_joints(root_body(mechanism), mechanism)) 
    base_pose = q[1:7] 
    set_configuration!(state, floating_base, base_pose)
    for (i, joint) in enumerate(digit.joint_names)
        set_configuration!(state, findjoint(mechanism, joint), q[i+7])
        # set_velocity!(state, findjoint(mechanism, joint), q̇[i+6])
    end
    qstate = configuration(state)
    q̇state = velocity(state)
    return qstate, q̇state
end

function set_nominal_configuration(digit::Digit)
    default_configuration = Dict(
        "left-hip-roll" => 0.302,
        "left-hip-yaw" => -0.03,
        "left-hip-pitch" => 0.335,
        "left-knee" => 0.366,
        "left-shin" => -0.0129,
        "left-tarsus" => -0.339,
        "left-toe-pitch" => 0.149,
        "left-toe-roll" => -0.082,
        "left-shoulder-roll" => -0.15,
        "left-shoulder-pitch" => 1.08,
        "left-shoulder-yaw" => 0.0,
        "left-elbow" => -0.145,
        "right-hip-roll" => -0.308,
        "right-hip-yaw" => 0.0147,
        "right-hip-pitch" => -0.308,
        "right-knee" => -0.367,
        "right-shin" => 0.012,
        "right-tarsus" => 0.340,
        "right-toe-pitch" => -0.122,
        "right-toe-roll" => 0.0787,
        "right-shoulder-roll" => 0.15,
        "right-shoulder-pitch" => -1.1,
        "right-shoulder-yaw" => 0.0,
        "right-elbow" => 0.145
    )
    for name in digit.joint_names
        digit.p.named.data.qpos[name] = default_configuration[name]
    end
end

function apply_velocity!(v::Vector{Float64}, digit::Digit)
    p = digit.p
    robot = digit.id
    joint_indices = digit.ids
    max_forces = 1000*ones(length(v))
    p.setJointMotorControlArray(robot, joint_indices, p.VELOCITY_CONTROL, 
        targetVelocities=v, forces=max_forces)
end

function apply_position!(pos::Vector{Float64}, v::Vector{Float64}, digit::Digit)
    p = digit.p
    robot = digit.id
    joint_indices = digit.joint_ids
    max_forces = 500*ones(length(v[8:end])) 
    p.setJointMotorControlArray(robot, joint_indices, p.POSITION_CONTROL, 
        targetPositions=pos[8:end], targetVelocities=v[8:end], forces=max_forces)
end

function apply_torque!(τ::Vector{Float64}, digit::Digit)
    p = digit.p
    for (i, name) in enumerate(digit.motor_names)
        p.named.data.ctrl[name] = τ[i]
    end
end

function set_joint_positions!(θ::Vector{Float64}, digit::Digit)
    state = digit.sim.state
    mechanism = state.mechanism
    floating_base = first(out_joints(root_body(mechanism), mechanism)) 
    base_pose = [ypr_to_quat(θ[4:6])..., θ[1:3]...]
    set_configuration!(state, floating_base, base_pose)
    for (i, joint) in enumerate(digit.joint_names)
        set_configuration!(state, findjoint(mechanism, joint), θ[i+6]) 
    end
end