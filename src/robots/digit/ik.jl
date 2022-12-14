## Inverse Kinematics functions

# Solves for Leg Joint Configurations given a goal COM position
function com_ik(θ₀::Vector{Float64}, 
        com_goal::Vector{Float64},
        sim; 
        max_iter=200, 
        tolerance=1e-3, maxΔ = 0.1)
    θ = copy(θ₀)
    leg_indices = [qleftHipRoll, qleftHipPitch, qleftKnee, qrightHipRoll, qrightHipPitch, qrightKnee]  
    θ[qbase_pitch] = 0.0 
    θ[qbase_roll] = 0.0 
    θ[qleftHipYaw] = 0.0 
    θ[qrightHipYaw] = 0.0   
    iter = 1
    for i = 1:max_iter
        θ[qleftTarsus] = -θ[qleftKnee]
        θ[qrightTarsus] = -θ[qrightKnee]

        com_pos = kin.p_com_wrt_feet(θ)
        com_error = com_pos - com_goal 

        error = norm(com_error, Inf) 
        if error < tolerance 
            # printstyled("converged at iter $iter\n";color=:blue)
            break
        end 

        Jall = kin.Jp_com_wrt_feet(θ) 
        J = Jall[1:end, leg_indices]
        J[1:end, 3] -= Jall[1:end, qleftTarsus]
        J[1:end, 6] -= Jall[1:end, qrightTarsus]

        θΔ = J \ com_error
        maxofD = max(θΔ...)
        if maxofD > maxΔ
            θΔ = maxΔ * (θΔ /maxofD)
        end 
        θ[leg_indices] -= θΔ   
        θ[leg_indices] = clamp.(θ[leg_indices], sim.θ_min[leg_indices], sim.θ_max[leg_indices])
        iter+=1
    end
    if iter > max_iter printstyled("Did not converge\n"; bold=true, color=:red) end 
    return θ
end

## hands ik wrt base 
function arm_ik(θ₀::Vector{Float64}, 
        hand_goals::Vector{Vector{Float64}},
        digit; 
        max_iter=200, 
        tolerance=1e-3, maxΔ = 0.1)
    θ = copy(θ₀)
    arm_indices = [qleftShoulderRoll, qleftShoulderPitch, qleftShoulderYaw, 
            qleftElbow, qrightShoulderRoll, qrightShoulderPitch, 
            qrightShoulderYaw, qrightElbow] 

    θ[qbase_yaw] = 0.0 

    iter = 1
    for i = 1:max_iter 
        right_hand_pos = kin.p_right_hand_wrt_base(θ)
        left_hand_pos = kin.p_left_hand_wrt_base(θ)

        arm_pose = [left_hand_pos..., right_hand_pos...]
        arm_goal = [hand_goals[1]..., hand_goals[2]...]
        arm_error = arm_pose - arm_goal 

        error = norm(arm_error, Inf) 
        if error < tolerance 
            printstyled("Arm IK converged at iter $iter\n";color=:blue)
            break
        end
        Jleft = kin.Jp_left_hand_wrt_base(θ)
        Jright = kin.Jp_right_hand_wrt_base(θ) 
        Jall = vcat(Jleft, Jright)
        J = Jall[1:end, arm_indices]

        θΔ = J \ arm_error

        maxofD = max(θΔ...)
        if maxofD > maxΔ*2
            θΔ = 2 *maxΔ * (θΔ /maxofD)
        end 
        θ[arm_indices] -= θΔ   

        θ[arm_indices] = clamp.(θ[arm_indices], digit.θarm_min, digit.θarm_max)

        iter+=1
    end
    if iter >= max_iter printstyled("Arm IK Did not converge\n"; bold=true, color=:red) end 
    return θ
end

function walk_ik(θ₀::Vector, θ̇₀::Vector,  vc_des::Vector,
    swing_foot::Symbol,
    digit; 
    max_iter=200, 
    tolerance=1e-3, maxΔ = 0.1)

    θ = copy(θ₀) 
    θ̇ = copy(θ̇₀)
    if swing_foot == :left
        indices = [qleftHipRoll, qleftHipPitch, qleftKnee, qrightKnee]
        vc_func = kin.p_left_ankle_wrt_base
        Jvc_func = kin.Jp_left_ankle_wrt_base
    else
        indices = [qrightHipRoll, qrightHipPitch, qrightKnee, qleftKnee]
        vc_func = kin.p_right_ankle_wrt_base
        Jvc_func = kin.Jp_right_ankle_wrt_base
    end
    θ[qbase_yaw] = 0.0 
    iter = 1
    for i=1:max_iter
        θ[qleftTarsus] = -θ[qleftKnee]
        θ[qrightTarsus] = -θ[qrightKnee]

        vc = vc_func(θ)
        vc_error = vc - vc_des
        error = norm(vc_error, Inf)

        if error < tolerance
            printstyled("Walk IK converged at iter $iter\n";color=:blue)
            break
        end
        Jvc_all = Jvc_func(θ)
        Jvc_all[1:end, qleftKnee] -= Jvc_all[1:end, qleftTarsus]
        Jvc_all[1:end, qrightKnee] -= Jvc_all[1:end, qrightTarsus]
        Jvc = Jvc_all[:, indices]

        θΔ = Jvc \ vc_error

        maxofD = max(θΔ...)
        if maxofD > maxΔ*2
            θΔ = 2 *maxΔ * (θΔ /maxofD)
        end 
        θ[indices] -= θΔ 
        θ[indices] = clamp.(θ[indices], digit.sim.θ_min[indices], digit.sim.θ_max[indices])
        iter+=1
    end
    if iter >= max_iter printstyled("Walk IK Did not converge\n"; bold=true, color=:red) end 
    return θ
end

