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
                                    digit::Digit; data=nothing)
    pos = digit.sim.θᵣ
    return pos
end

function posture_torque_controller(q::Vector{Float64}, q̇::Vector{Float64}, 
    digit::Digit; data=nothing)
    Kp = 500.0
    Kd = 0.5
    θrefs = [0.32869133647921467, -0.02792180592249217, 0.3187324455828634, 0.36118057019763633, -0.14684031092035302, 
0.11311574329868718, -0.32875125760374146, 0.02783743697915846, -0.31868450868324194, -0.3611086648482042, 0.14674060216914045, 
-0.11315409281838432, -0.15050988058637318, 1.0921200187801636, 0.00017832526659170586, 
-0.13909131109654943, 0.15051467427633533, -1.0921631619898227, -0.00017832526659170586, 0.13910089847647372]

    obs = get_motor_positions(digit)
    torques = Kp * (θrefs - obs) 
    torques = collect(torques) 
    return torques
end

function balance_torque_controller(q::Vector{Float64}, q̇::Vector{Float64},
                         digit::Digit;  com_goal=[0.0, 0.0, 0.9], data=nothing)
    θ, θ̇  = get_qall_coordinates(digit)
    q_motors = get_motor_positions(digit) 
    step_width = 0.27 
    heading = θ[qbase_yaw]   
    # heading = 0.0
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

    θd = com_ik(θ, goal, digit.sim)   
    q_motors_des = copy(q_motors)

    q_motors_des[LeftHipRoll] = θd[qleftHipRoll]
    q_motors_des[LeftHipPitch] = θd[qleftHipPitch]
    q_motors_des[LeftKnee] = θd[qleftKnee]
    q_motors_des[RightHipRoll] = θd[qrightHipRoll]
    q_motors_des[RightHipPitch] = θd[qrightHipPitch]
    q_motors_des[RightKnee] = θd[qrightKnee]

    # q_motors_des[LeftHipRoll] = 0.337
    # q_motors_des[LeftHipPitch] = 0.0
    # q_motors_des[LeftKnee] = 0.0
    # q_motors_des[RightHipRoll] = -0.337
    # q_motors_des[RightHipPitch] = 0.0
    # q_motors_des[RightKnee] = 0.0

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

    q_motors_des[LeftToeA] =  q_motors[LeftToeA]  + toe_pitch_error
    q_motors_des[LeftToeB] = q_motors[LeftToeB] - toe_pitch_error
    q_motors_des[RightToeA] = q_motors[RightToeA] - toe_pitch_error
    q_motors_des[RightToeB] = q_motors[RightToeB] + toe_pitch_error 

    # q_motors_des[LeftToeA] =  -0.15
    # q_motors_des[LeftToeB] = 0.11
    # q_motors_des[RightToeA] = 0.15
    # q_motors_des[RightToeB] = -0.11


    τ = zeros(NUM_MOTORS) 
    q_motors_error = q_motors - q_motors_des  
 
    kp_hiproll_stand = 80   
    kp_hipyaw_stand = 50.0
    kp_hippitch_stand = 50.0
    kp_knee_stand = 80.0
    kp_toe_stand = 3.0  
    kp_knee_comp_stand = 270
    kd_knee_comp_stand = 30

    kp_shoulderroll_stand = 100.0
    kp_shoulderpitch_stand = 100.0
    kp_shoulderyaw_stand = 100.0
    kp_elbow_stand = 100.0

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
    τ[LeftToeA] = -kp_toe_stand * q_motors_error[LeftToeA] #+ kd_toe_stand * v_com_aligned[1]
    τ[LeftToeB] = -kp_toe_stand * q_motors_error[LeftToeB] #- kd_toe_stand * v_com_aligned[1]
    τ[RightToeA] = -kp_toe_stand * q_motors_error[RightToeA] #- kd_toe_stand * v_com_aligned[1]
    τ[RightToeB] = -kp_toe_stand * q_motors_error[RightToeB] #+ kd_toe_stand * v_com_aligned[1]

    # knee_comp
    knee_comp = -kp_knee_comp_stand * -com_midpoint_error[2] - kd_knee_comp_stand * -v_com_aligned[2] 
    τ[LeftKnee] += knee_comp
    τ[RightKnee] += knee_comp

    return τ 
end