function update_state!(q::Vector{Float64},  q̇::Vector{Float64}, digit::Digit)
    sim = digit.sim
    state = sim.state
    mechanism = state.mechanism
    floating_base = first(out_joints(root_body(mechanism), mechanism)) 
    base_pose = [q[4], q[1:3]..., q[5:7]...] 
    set_configuration!(state, floating_base, base_pose)
    for (i, joint) in enumerate(digit.joint_names)
        set_configuration!(state, findjoint(mechanism, joint), q[i+7])
        # set_velocity!(state, findjoint(mechanism, joint), q̇[i+6])
    end
    qstate = configuration(state)
    q̇state = velocity(state)
    return qstate, q̇state
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
    max_forces = 500*ones(length(v))
    p.setJointMotorControlArray(robot, joint_indices, p.POSITION_CONTROL, 
        targetPositions=pos, targetVelocities=v, forces=max_forces)
end