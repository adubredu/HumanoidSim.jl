function get_generalized_coordinates(digit::Digit)
    if digit.engine == :PyBullet
        p = digit.p 
        base_position, base_orientation_quat = p.getBasePositionAndOrientation(digit.id) 
        base_linear_velocity, base_angular_velocity = p.getBaseVelocity(digit.id) 
        joint_states = p.getJointStates(digit.id, digit.joint_ids) 
        q = [js[0] for js in joint_states]
        q = [base_orientation_quat..., base_position..., q...]
        q̇ = [js[1] for js in joint_states]
        q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
        q = [pyconvert(Float64, a) for a in q]
        q̇ = [pyconvert(Float64, b) for b in q̇] 
        return q, q̇

    elseif digit.engine == :MuJoCo
        p = digit.p
        base_orientation_quat = p.data.qpos[3:6]
        tmp = base_orientation_quat[3]
        base_orientation_quat[3] = base_orientation_quat[0]
        base_orientation_quat[0] = tmp
        base_position = p.data.qpos[0:2]
        base_linear_velocity = p.data.qvel[0:2]
        base_angular_velocity = p.data.qvel[3:5]
        q = [p.named.data.qpos[name][0] for name in digit.joint_names] 
        q = [base_orientation_quat..., base_position..., q...]
        q̇ = [p.named.data.qvel[name][0] for name in digit.joint_names] 
        q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
        q = [pyconvert(Float64, a) for a in q]
        q̇ = [pyconvert(Float64, b) for b in q̇] 
        return q, q̇

    else
        return nothing, nothing
    end
end

function get_qall_coordinates(digit::Digit)
    if digit.engine == :PyBullet
        p = digit.p 
        base_position, base_orientation_quat = p.getBasePositionAndOrientation(digit.id) 
        base_orientation = p.getEulerFromQuaternion(base_orientation_quat)
        base_linear_velocity, base_angular_velocity = p.getBaseVelocity(digit.id) 
        joint_states = p.getJointStates(digit.id, digit.joint_ids) 
        q = [js[0] for js in joint_states]
        q = [base_position..., base_orientation...,  q...]
        q̇ = [js[1] for js in joint_states]
        q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
        q = [pyconvert(Float64, a) for a in q]
        q̇ = [pyconvert(Float64, b) for b in q̇] 
        return q, q̇

    elseif digit.engine == :MuJoCo
        p = digit.p
        base_orientation_quat = p.data.qpos[3:6] 
        base_orientation = quat_to_ypr([pyconvert(Float64, a) for a in base_orientation_quat])
        base_position = p.data.qpos[0:2]
        base_linear_velocity = p.data.qvel[0:2]
        base_angular_velocity = p.data.qvel[3:5]
        q = [p.named.data.qpos[name][0] for name in digit.joint_names] 
        q = [ base_position..., base_orientation..., q...]
        q̇ = [p.named.data.qvel[name][0] for name in digit.joint_names] 
        q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
        q = [pyconvert(Float64, a) for a in q]
        q̇ = [pyconvert(Float64, b) for b in q̇] 
        return q, q̇

    else
        return nothing, nothing
    end

end

function get_motor_positions(digit::Digit)
    pos = [pyconvert(Float64, digit.p.named.data.qpos[name][0]) 
            for name in digit.motor_names]
    return pos
end