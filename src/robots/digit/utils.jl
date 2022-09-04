function quat_to_rpy(q::Vector{Float64})
    quat = QuatRotation(q)
    zyx = RotZYX(quat)
    ypr = [zyx.theta1, zyx.theta2, zyx.theta3]
    return ypr
end 

function rpy_to_quat(q::Vector{Float64})
    rpy = RotXYZ(q...)
    qr = QuatRotation(rpy)
    quat = [qr.q.s, qr.q.v1, qr.q.v2, qr.q.v3]
    return quat
end

function euler_configuration(state::MechanismState)
    q = configuration(state)
    quat = q[1:4]
    ypr = reverse(quat_to_rpy(quat)) 
    qall = [q[5:7]..., ypr..., q[8:end]...]
    return qall
end

function get_qall_from_state(state::MechanismState)
    q = configuration(state)
    qall = zeros(30)
    qall[1:3] = q[5:7]
    qall[4:6] = quat_to_rpy(q[1:4]) 
    qall[7] = q[8]
    qall[8] = q[12]
    qall[9] = q[16]
    qall[10] = q[20]
    qall[11] = q[24]
    qall[12] = q[26]
    qall[13] = q[28]
    qall[14] = q[30]
    qall[15] = q[9]
    qall[16] = q[13]
    qall[17] = q[17]
    qall[18] = q[21]
    qall[19] = q[10]
    qall[20] = q[14]
    qall[21] = q[18]
    qall[22] = q[22]
    qall[23] = q[25]
    qall[24] = q[27]
    qall[25] = q[29]
    qall[26] = q[31]
    qall[27] = q[11]
    qall[28] = q[15]
    qall[29] = q[19]
    qall[30] = q[23]
    return qall
end

function get_qdotall_from_state(state::MechanismState)
    q = velocity(state)
    qdotall = zeros(30)
    qdotall[1:3] = q[4:6]
    qdotall[4:6] =q[1:3]
    qdotall[7] = q[7]
    qdotall[8] = q[11]
    qdotall[9] = q[15]
    qdotall[10] = q[19]
    qdotall[11] = q[23]
    qdotall[12] = q[25]
    qdotall[13] = q[27]
    qdotall[14] = q[29]
    qdotall[15] = q[8]
    qdotall[16] = q[12]
    qdotall[17] = q[16]
    qdotall[18] = q[20]
    qdotall[19] = q[9]
    qdotall[20] = q[13]
    qdotall[21] = q[17]
    qdotall[22] = q[21]
    qdotall[23] = q[24]
    qdotall[24] = q[26]
    qdotall[25] = q[28]
    qdotall[26] = q[30]
    qdotall[27] = q[10]
    qdotall[28] = q[14]
    qdotall[29] = q[18]
    qdotall[30] = q[22]
    return qdotall
end

function get_qall_from_q(q::AbstractVector{T}) where T
    qall = zeros(T, 30)
    q = [zeros(T, 7)..., q...]
    qall[7] = q[8]
    qall[8] = q[12]
    qall[9] = q[16]
    qall[10] = q[20]
    qall[11] = q[24]
    qall[12] = q[26]
    qall[13] = q[28]
    qall[14] = q[30]
    qall[15] = q[9]
    qall[16] = q[13]
    qall[17] = q[17]
    qall[18] = q[21]
    qall[19] = q[10]
    qall[20] = q[14]
    qall[21] = q[18]
    qall[22] = q[22]
    qall[23] = q[25]
    qall[24] = q[27]
    qall[25] = q[29]
    qall[26] = q[31]
    qall[27] = q[11]
    qall[28] = q[15]
    qall[29] = q[19]
    qall[30] = q[23]
    return qall
end

function get_qstate_from_qall(qall::Vector{Float64})
    q = zeros(31)
    q[5:7] = qall[1:3]
    q[1:4] = rpy_to_quat(qall[4:6])
    q[8]  = qall[7]  
    q[12] =  qall[8]
    q[16] = qall[9]
    q[20] = qall[10]
    q[24] = qall[11]
    q[26] = qall[12]
    q[28] = -0.126#qall[13]
    q[30] = 0#qall[14]
    q[9]  = qall[15]
    q[13] = qall[16]
    q[17] = qall[17]
    q[21] = qall[18]
    q[10] = qall[19]
    q[14] = qall[20]
    q[18] = qall[21]
    q[22] = qall[22]
    q[25] = qall[23]
    q[27] = qall[24]
    q[29] = 0.126#qall[25]
    q[31] = 0#qall[26]
    q[11] = qall[27]
    q[15] = qall[28]
    q[19] = qall[29]
    q[23] = qall[30]
    return q
end