function make_posture_controller_rbd(sim; Kp=1000.0, Kd=5.0)
    qref = sim.init_config[7:end]
    
    function posture_controller_rbd(torque, t, state)
        qpos = euler_configuration(state)[7:end]
        qvel = velocity(state)[7:end] 
        torque[7:end] = -Kp*(qpos-qref) #- Kd*qvel
        # @show torque
        torque
    end
end

function make_balance_controller_rbd(sim, com_goal)
    step_width = 0.27 
    function balance_controller(τ, t, state) 
        θ = get_qall_from_state(state)
        θ̇  = get_qdotall_from_state(state)
        β = 5.0*ones(24) 
        heading = θ[qbase_yaw]  
        wrap_to_pi!(heading)
        Rz = RotZ(heading)  

        # Aligned toes
        p_left_toe_world = kin.p_toe_pitch_joint_left(θ)
        p_right_toe_world = kin.p_toe_pitch_joint_right(θ)
        p_left_toe_aligned = Rz' * p_left_toe_world 
        p_right_toe_aligned = Rz' * p_right_toe_world

        # Aligned com 
        p_com_world = kin.p_COM(θ)
        v_com_world = kin.v_COM(θ, θ̇ )
        p_com_aligned =  Rz' * p_com_world
        v_com_aligned =  Rz' * v_com_world  

        # Main IK  
        com_wrt_left_aligned_des = [0.0, -0.5*step_width, com_goal[3]]
        com_wrt_right_aligned_des = [0.0, 0.5*step_width, com_goal[3]]
        goal =  [(Rz * com_wrt_left_aligned_des)..., (Rz * com_wrt_right_aligned_des)...] 

        θd = com_ik(θ, goal, sim)   
        q_motors = copy(θ)
        q_motors_des = zero(θ) 

        q_motors_des[qleftHipRoll] = θd[qleftHipRoll]
        q_motors_des[qleftHipPitch] = θd[qleftHipPitch]
        q_motors_des[qleftKnee] = θd[qleftKnee]
        q_motors_des[qrightHipRoll] = θd[qrightHipRoll]
        q_motors_des[qrightHipPitch] = θd[qrightHipPitch]
        q_motors_des[qrightKnee] = θd[qrightKnee]

        q_motors_des[qleftHipYaw] = 0.0
        q_motors_des[qrightHipYaw] = 0.0

        q_motors_des[qleftShoulderRoll] = 0.0
        q_motors_des[qleftShoulderPitch] = 0.589
        q_motors_des[qleftShoulderYaw] = 0
        q_motors_des[qleftElbow] = -0.0
        q_motors_des[qrightShoulderRoll] = 0.0
        q_motors_des[qrightShoulderPitch] = -0.589
        q_motors_des[qrightShoulderYaw] = 0
        q_motors_des[qrightElbow] = 0.0

        # q_motors_des[qleftHipRoll] = 0.337
        # q_motors_des[qleftHipPitch] = 0.0 
        # q_motors_des[qleftKnee] = 0.0
        # q_motors_des[qrightHipRoll] = -0.337
        # q_motors_des[qrightHipPitch] = 0.0
        # q_motors_des[qrightKnee] = 0.0

        q_motors_des[qleftToePitch] = -0.126
        q_motors_des[qleftToeRoll] = 0.0
        q_motors_des[qrightToePitch] = 0.126
        q_motors_des[qrightToeRoll] = 0.0

        q_motors_des[qleftShin] = 0.0
        q_motors_des[qleftTarsus] = 0.0 #-q_motors_des[qleftKnee]
        q_motors_des[qrightShin] = 0.0
        q_motors_des[qrightTarsus] = 0.0 #-q_motors_des[qrightKnee]


        com_midpoint_error = p_com_aligned - 0.5 * 
                    (p_left_toe_aligned + p_right_toe_aligned)
        # toe_pitch_error = com_midpoint_error[1] 

        # q_motors_des[qleftToePitch] =   q_motors[qleftToePitch]  + toe_pitch_error
        # q_motors_des[qleftToeRoll] = q_motors[qleftToeRoll] - toe_pitch_error
        # q_motors_des[qrightToePitch] =  q_motors[qrightToePitch] - toe_pitch_error
        # q_motors_des[qrightToeRoll] =  q_motors[qrightToeRoll]+ toe_pitch_error 

        q_motors_error = q_motors - q_motors_des  

        kp_hiproll_stand = 800   
        kp_hipyaw_stand = 500.0
        kp_hippitch_stand = 500.0
        kp_knee_stand = 800.0
        kp_toe_stand = 300.0  
        kp_knee_comp_stand = 2700
        kd_knee_comp_stand = 300

        kp_shoulderroll_stand = 100.0
        kp_shoulderpitch_stand = 100.0
        kp_shoulderyaw_stand = 100.0
        kp_elbow_stand = 100.0

        kp = 1000

        torque = zero(τ) 
        torque[1:6] .= 0.0
        
        torque[qleftHipRoll] = -kp_hiproll_stand * q_motors_error[qleftHipRoll]
        torque[qleftHipYaw] = -kp_hipyaw_stand * q_motors_error[qleftHipYaw]
        torque[qleftHipPitch] = -kp_hippitch_stand * q_motors_error[qleftHipPitch]
        torque[qleftKnee] = -kp_knee_stand * q_motors_error[qleftKnee]
        torque[qrightHipRoll] = -kp_hiproll_stand * q_motors_error[qrightHipRoll]
        torque[qrightHipYaw] = -kp_hipyaw_stand * q_motors_error[qrightHipYaw]
        torque[qrightHipPitch] = -kp_hippitch_stand * q_motors_error[qrightHipPitch]
        torque[qrightKnee] = -kp_knee_stand * q_motors_error[qrightKnee]

        torque[qleftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[qleftShoulderRoll]
        torque[qleftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[qleftShoulderPitch]
        torque[qleftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[qleftShoulderYaw]
        torque[qleftElbow] = -kp_elbow_stand * q_motors_error[qleftElbow]
        torque[qrightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[qrightShoulderRoll]
        torque[qrightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[qrightShoulderPitch]
        torque[qrightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[qrightShoulderYaw]
        torque[qrightElbow] = -kp_elbow_stand * q_motors_error[qrightElbow]

        
        torque[qleftShin] = -kp_elbow_stand * q_motors_error[qleftShin]
        torque[qleftTarsus] = -kp_knee_stand * q_motors_error[qleftTarsus]
        torque[qrightShin] = -kp_knee_stand * q_motors_error[qrightShin]
        torque[qrightTarsus] = -kp_knee_stand * q_motors_error[qrightTarsus]

        kd_toe_stand = 50;
        torque[qleftToePitch] = -kp_toe_stand * q_motors_error[qleftToePitch] #+ kd_toe_stand * v_com_aligned[1]
        torque[qleftToeRoll] = -kp_toe_stand * q_motors_error[qleftToeRoll]# - kd_toe_stand * v_com_aligned[1]
        torque[qrightToePitch] = -kp_toe_stand * q_motors_error[qrightToePitch]# - kd_toe_stand * v_com_aligned[1]
        torque[qrightToeRoll] = -kp_toe_stand * q_motors_error[qrightToeRoll] #+ kd_toe_stand * v_com_aligned[1]
        # =#
        # knee_comp
        # knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
        # torque[qleftKnee] += knee_comp
        # torque[qrightKnee] += knee_comp
        @time set_configuration!(sim.mvis, configuration(state))
        τ .= rearrange_torque(torque) 
        # @show τ  
        τ   
    end
end

function posture_position_controller(q::Vector{Float64}, q̇::Vector{Float64}, 
                                    digit::Digit)
    pos = digit.sim.θᵣ
    return pos
end

function balance_position_controller(q::Vector{Float64}, q̇::Vector{Float64},
                         digit::Digit;  com_goal=[0.0, 0.0, 0.9])
    step_width = 0.27  
    ypr = quat_to_ypr(q[1:4])
    heading = ypr[1]  
    # @show heading
    wrap_to_pi!(heading)
    Rz = RotZ(heading) 
    # @show q[5:7]
    θ = [q[5:7]..., ypr..., q[8:end]...] 
    θ̇  = [q̇[4:6]..., q̇[1:3]..., q̇[7:end]...]

    # Aligned toes
    p_left_toe_world = kin.p_toe_pitch_joint_left(θ)
    p_right_toe_world = kin.p_toe_pitch_joint_right(θ)
    p_left_toe_aligned = Rz' * p_left_toe_world 
    p_right_toe_aligned = Rz' * p_right_toe_world

    # Aligned com 
    p_com_world = kin.p_COM(θ)
    v_com_world = kin.v_COM(θ, θ̇ )
    p_com_aligned =  Rz' * p_com_world
    v_com_aligned =  Rz' * v_com_world  

    # Main IK  
    com_wrt_left_aligned_des = [0.0, -0.5*step_width, com_goal[3]]
    com_wrt_right_aligned_des = [0.0, 0.5*step_width, com_goal[3]]
    goal =  [(Rz * com_wrt_left_aligned_des)..., (Rz * com_wrt_right_aligned_des)...] 

    θd = com_ik(θ, goal, digit.sim) 
    q_motors = copy(θ)  
    q_motors_des = zero(θ) 

    q_motors_des[qleftHipRoll] = θd[qleftHipRoll]
    q_motors_des[qleftHipPitch] = θd[qleftHipPitch]
    q_motors_des[qleftKnee] = θd[qleftKnee]
    q_motors_des[qrightHipRoll] = θd[qrightHipRoll]
    q_motors_des[qrightHipPitch] = θd[qrightHipPitch]
    q_motors_des[qrightKnee] = θd[qrightKnee]

    q_motors_des[qleftHipYaw] = 0.0
    q_motors_des[qrightHipYaw] = 0.0

    q_motors_des[qleftShoulderRoll] = 0.0
    q_motors_des[qleftShoulderPitch] = 0.589
    q_motors_des[qleftShoulderYaw] = 0
    q_motors_des[qleftElbow] = -0.0
    q_motors_des[qrightShoulderRoll] = 0.0
    q_motors_des[qrightShoulderPitch] = -0.589
    q_motors_des[qrightShoulderYaw] = 0
    q_motors_des[qrightElbow] = 0.0

    # q_motors_des[qleftHipRoll] = 0.337
    # q_motors_des[qleftHipPitch] = 0.0 
    # q_motors_des[qleftKnee] = 0.0
    # q_motors_des[qrightHipRoll] = -0.337
    # q_motors_des[qrightHipPitch] = 0.0
    # q_motors_des[qrightKnee] = 0.0

    q_motors_des[qleftToePitch] = -0.126
    q_motors_des[qleftToeRoll] = 0.0
    q_motors_des[qrightToePitch] = 0.126
    q_motors_des[qrightToeRoll] = 0.0

    q_motors_des[qleftShin] = 0.0
    q_motors_des[qleftTarsus] = 0.0 #-q_motors_des[qleftKnee]
    q_motors_des[qrightShin] = 0.0
    q_motors_des[qrightTarsus] = 0.0 #-q_motors_des[qrightKnee]


    # com_midpoint_error = p_com_aligned - 0.5 * 
    #             (p_left_toe_aligned + p_right_toe_aligned)
    # toe_pitch_error = com_midpoint_error[1] 

    # q_motors_des[qleftToePitch] =   q_motors[qleftToePitch]  + toe_pitch_error
    # q_motors_des[qleftToeRoll] = q_motors[qleftToeRoll] - toe_pitch_error
    # q_motors_des[qrightToePitch] =  q_motors[qrightToePitch] - toe_pitch_error
    # q_motors_des[qrightToeRoll] =  q_motors[qrightToeRoll]+ toe_pitch_error 

    
    # =#
    # knee_comp
    # knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    # torque[qleftKnee] += knee_comp
    # torque[qrightKnee] += knee_comp
    pos = q_motors_des[7:end] 
    return pos
end