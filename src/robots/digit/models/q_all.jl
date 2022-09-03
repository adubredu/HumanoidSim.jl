function quat_to_rpy(q::Vector{Float64})
    quat = QuatRotation(q)
    a = AngleAxis(quat)
    rpy = rotation_angle(a) * rotation_axis(a)
    return rpy
end 

function rpy_to_quat(q::Vector{Float64})
    rpy = RotXYZ(q...)
    qr = QuatRotation(rpy)
    quat = [qr.q.s, qr.q.v1, qr.q.v2, qr.q.v3]
    return quat
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
    q[28] = qall[13]
    q[30] = qall[14]
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
    q[29] = qall[25]
    q[31] = qall[26]
    q[11] = qall[27]
    q[15] = qall[28]
    q[19] = qall[29]
    q[23] = qall[30]
    return q
end