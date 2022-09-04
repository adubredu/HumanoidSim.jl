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
        θ, θ̇ , q_motors = get_generalized_coordinates(observation)
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

        θd = com_ik(θ, goal, digit)   
        q_motors_des = copy(q_motors) 

        q_motors_des[LeftHipRoll] = θd[qleftHipRoll]
        q_motors_des[LeftHipPitch] = θd[qleftHipPitch]
        q_motors_des[LeftKnee] = θd[qleftKnee]
        q_motors_des[RightHipRoll] = θd[qrightHipRoll]
        q_motors_des[RightHipPitch] = θd[qrightHipPitch]
        q_motors_des[RightKnee] = θd[qrightKnee]

        q_motors_des[LeftHipYaw] = 0.0
        q_motors_des[RightHipYaw] = 0.0

        q_motors_des[LeftShoulderRoll] = -0.15
        q_motors_des[LeftShoulderPitch] = 1.1
        q_motors_des[LeftShoulderYaw] = 0
        q_motors_des[LeftElbow] = -0.145
        q_motors_des[RightShoulderRoll] = 0.15
        q_motors_des[RightShoulderPitch] = -1.1
        q_motors_des[RightShoulderYaw] = 0
        q_motors_des[RightElbow] = 0.145

        com_midpoint_error = p_com_aligned - 0.5 * 
                    (p_left_toe_aligned + p_right_toe_aligned)
        toe_pitch_error = com_midpoint_error[1] 

        q_motors_des[LeftToeA] =   q_motors[LeftToeA]  + toe_pitch_error
        q_motors_des[LeftToeB] = q_motors[LeftToeB] - toe_pitch_error
        q_motors_des[RightToeA] =  q_motors[RightToeA] - toe_pitch_error
        q_motors_des[RightToeB] =  q_motors[RightToeB]+ toe_pitch_error 

        # LeftToeA_des = -0.147
        # LeftToeB_des = 0.113
        # RightToeA_des = 0.147
        # RightToeB_des = -0.114

        # q_motors_des[LeftToeA] =   LeftToeA_des  + toe_pitch_error
        # q_motors_des[LeftToeB] = LeftToeB_des - toe_pitch_error
        # q_motors_des[RightToeA] =  RightToeA_des - toe_pitch_error
        # q_motors_des[RightToeB] =  RightToeB_des + toe_pitch_error 

        τ = zeros(NUM_MOTORS) 
        q_motors_error = q_motors - q_motors_des  

        if real
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
        else
            kp_hiproll_stand = 500  
            kp_hipyaw_stand = 300.0
            kp_hippitch_stand = 300.0
            kp_knee_stand = 800.0
            kp_toe_stand = 200.0
            kp_knee_comp_stand = 1000
            kd_knee_comp_stand = 20

            kp_shoulderroll_stand = 100.0
            kp_shoulderpitch_stand = 100.0
            kp_shoulderyaw_stand = 100.0
            kp_elbow_stand = 100.0
        end
        τ[LeftHipRoll] = -kp_hiproll_stand * q_motors_error[LeftHipRoll]
        τ[LeftHipYaw] = -kp_hipyaw_stand * q_motors_error[LeftHipYaw]
        τ[LeftHipPitch] = -kp_hippitch_stand * q_motors_error[LeftHipPitch]
        τ[LeftKnee] = -kp_knee_stand * q_motors_error[LeftKnee]
        τ[RightHipRoll] = -kp_hiproll_stand * q_motors_error[RightHipRoll]
        τ[RightHipYaw] = -kp_hipyaw_stand * q_motors_error[RightHipYaw]
        τ[RightHipPitch] = -kp_hippitch_stand * q_motors_error[RightHipPitch]
        τ[RightKnee] = -kp_knee_stand * q_motors_error[RightKnee]

        τ[LeftShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[LeftShoulderRoll]
        τ[LeftShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[LeftShoulderPitch]
        τ[LeftShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[LeftShoulderYaw]
        τ[LeftElbow] = -kp_elbow_stand * q_motors_error[LeftElbow]
        τ[RightShoulderRoll] = -kp_shoulderroll_stand * q_motors_error[RightShoulderRoll]
        τ[RightShoulderPitch] = -kp_shoulderpitch_stand * q_motors_error[RightShoulderPitch]
        τ[RightShoulderYaw] = -kp_shoulderyaw_stand * q_motors_error[RightShoulderYaw]
        τ[RightElbow] = -kp_elbow_stand * q_motors_error[RightElbow]

        kd_toe_stand = 50;
        τ[LeftToeA] = -kp_toe_stand * q_motors_error[LeftToeA] + kd_toe_stand * v_com_aligned[1]
        τ[LeftToeB] = -kp_toe_stand * q_motors_error[LeftToeB] - kd_toe_stand * v_com_aligned[1]
        τ[RightToeA] = -kp_toe_stand * q_motors_error[RightToeA] - kd_toe_stand * v_com_aligned[1]
        τ[RightToeB] = -kp_toe_stand * q_motors_error[RightToeB] + kd_toe_stand * v_com_aligned[1]

        # knee_comp
        knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
        τ[LeftKnee] += knee_comp
        τ[RightKnee] += knee_comp
        v = zeros(NUM_MOTORS) 
        fallback_opmode = Locomotion 
        apply_command = true  
        comms.send_torque_command(fallback_opmode, apply_command, τ, v, β)
    end
end
