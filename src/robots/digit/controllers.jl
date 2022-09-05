function make_posture_controller(sim; Kp=1000.0, Kd=5.0)
    qref = sim.init_config[7:end]
    
    function posture_controller(τ, t, state)
        qpos = euler_configuration(state)[7:end]
        qvel = velocity(state)[7:end] 
        τ[7:end] = -Kp*(qpos-qref) - Kd*qvel
        τ
    end
end

function make_balance_controller(sim, com_goal)
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
        q_motors_des = copy(θ) 

        q_motors_des[qleftHipRoll] = θd[qleftHipRoll]
        q_motors_des[qleftHipPitch] = θd[qleftHipPitch]
        q_motors_des[qleftKnee] = θd[qleftKnee]
        q_motors_des[qrightHipRoll] = θd[qrightHipRoll]
        q_motors_des[qrightHipPitch] = θd[qrightHipPitch]
        q_motors_des[qrightKnee] = θd[qrightKnee]

        q_motors_des[qleftHipYaw] = 0.0
        q_motors_des[qrightHipYaw] = 0.0

        q_motors_des[qleftShoulderRoll] = -0.15
        q_motors_des[qleftShoulderPitch] = 1.1
        q_motors_des[qleftShoulderYaw] = 0
        q_motors_des[qleftElbow] = -0.145
        q_motors_des[qrightShoulderRoll] = 0.15
        q_motors_des[qrightShoulderPitch] = -1.1
        q_motors_des[qrightShoulderYaw] = 0
        q_motors_des[qrightElbow] = 0.145

        com_midpoint_error = p_com_aligned - 0.5 * 
                    (p_left_toe_aligned + p_right_toe_aligned)
        toe_pitch_error = com_midpoint_error[1] 

        q_motors_des[qleftToePitch] =   q_motors[qleftToePitch]  + toe_pitch_error
        q_motors_des[qleftToeRoll] = q_motors[qleftToeRoll] - toe_pitch_error
        q_motors_des[qrightToePitch] =  q_motors[qrightToePitch] - toe_pitch_error
        q_motors_des[qrightToeRoll] =  q_motors[qrightToeRoll]+ toe_pitch_error 

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

        τ[qleftHipRoll] = -kp_hiproll_stand * q_motors_error[qleftHipRoll]
        τ[qleftHipYaw] = -kp_hipyaw_stand * q_motors_error[qleftHipYaw]
        τ[qleftHipPitch] = -kp_hippitch_stand * q_motors_error[qleftHipPitch]
        τ[qleftKnee] = -kp_knee_stand * q_motors_error[qleftKnee]
        τ[qrightHipRoll] = -kp_hiproll_stand * q_motors_error[qrightHipRoll]
        τ[qrightHipYaw] = -kp_hipyaw_stand * q_motors_error[qrightHipYaw]
        τ[qrightHipPitch] = -kp_hippitch_stand * q_motors_error[qrightHipPitch]
        τ[qrightKnee] = -kp_knee_stand * q_motors_error[qrightKnee]

        τ[qleftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[qleftShoulderRoll]
        τ[qleftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[qleftShoulderPitch]
        τ[qleftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[qleftShoulderYaw]
        τ[qleftElbow] = -kp_elbow_stand * q_motors_error[qleftElbow]
        τ[qrightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[qrightShoulderRoll]
        τ[qrightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[qrightShoulderPitch]
        τ[qrightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[qrightShoulderYaw]
        τ[qrightElbow] = -kp_elbow_stand * q_motors_error[qrightElbow]

        kd_toe_stand = 50;
        τ[qleftToePitch] = -kp_toe_stand * q_motors_error[qleftToePitch] + kd_toe_stand * v_com_aligned[1]
        τ[qleftToeRoll] = -kp_toe_stand * q_motors_error[qleftToeRoll] - kd_toe_stand * v_com_aligned[1]
        τ[qrightToePitch] = -kp_toe_stand * q_motors_error[qrightToePitch] - kd_toe_stand * v_com_aligned[1]
        τ[qrightToeRoll] = -kp_toe_stand * q_motors_error[qrightToeRoll] + kd_toe_stand * v_com_aligned[1]

        # knee_comp
        knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
        τ[qleftKnee] += knee_comp
        τ[qrightKnee] += knee_comp
        τ
    end
end
