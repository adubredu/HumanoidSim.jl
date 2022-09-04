function get_generalized_coordinates(digit::Digit)
    p = digit.p
    # pyplan = pybullet_planning
    base_position, base_orientation_quat = p.getBasePositionAndOrientation(digit.id)
    base_orientation = p.getEulerFromQuaternion(base_orientation_quat)
    base_linear_velocity, base_angular_velocity = p.getBaseVelocity(digit.id)
    #  joint_ids = [pyplan.get_joint(digit.id, name) for name in digit.joint_names]
    joint_states = p.getJointStates(digit.id, digit.joint_ids)

    q = [js[0] for js in joint_states]
    q = [base_position..., base_orientation..., q...]
    q̇ = [js[1] for js in joint_states]
    q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
    q = [pyconvert(Float64, a) for a in q]
    q̇ = [pyconvert(Float64, b) for b in q̇]  

    return q, q̇
end