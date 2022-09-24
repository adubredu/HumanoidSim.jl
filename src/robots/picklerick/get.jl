function get_generalized_coordinates(picklerick::Picklerick)
    p = picklerick.p
    # pyplan = pybullet_planning
    base_position, base_orientation_quat = p.getBasePositionAndOrientation(picklerick.id)
    # base_orientation = p.getEulerFromQuaternion(base_orientation_quat)
    base_linear_velocity, base_angular_velocity = p.getBaseVelocity(picklerick.id)
    #  joint_ids = [pyplan.get_joint(picklerick.id, name) for name in picklerick.joint_names]
    joint_states = p.getJointStates(picklerick.id, picklerick.joint_ids)

    q = [js[0] for js in joint_states]
    q = [base_orientation_quat..., base_position..., q...]
    q̇ = [js[1] for js in joint_states]
    q̇ = [base_linear_velocity..., base_angular_velocity..., q̇...]
    q = [pyconvert(Float64, a) for a in q]
    q̇ = [pyconvert(Float64, b) for b in q̇]  

    return q, q̇
end

