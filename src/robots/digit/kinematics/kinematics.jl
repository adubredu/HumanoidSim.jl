module kinematics
 
include("p_com_wrt_feet_center.jl")
include("p_com_wrt_feet.jl")
include("p_feet_wrt_com.jl")
include("p_left_foot.jl")
include("p_right_foot.jl")
include("Jp_com_wrt_feet.jl")
include("p_left_wrist.jl") 
include("p_right_wrist.jl") 
include("p_lcp1.jl")
include("p_lcp2.jl")
include("p_lcp3.jl")
include("p_rcp1.jl")
include("p_rcp2.jl")
include("p_rcp3.jl")
include("p_left_hand_wrt_feet.jl")
include("p_right_hand_wrt_feet.jl") 
include("p_com.jl")
include("Jp_com.jl")
include("v_COM.jl")
include("p_toe_pitch_joint_left.jl")
include("p_toe_pitch_joint_right.jl")
include("J_vc_walk_LS.jl")
include("J_vc_walk_RS.jl")
include("vc_walk_LS.jl")
include("vc_walk_RS.jl")
include("p_left_toe_mid.jl")
include("Jp_left_toe_mid.jl")
include("p_right_toe_mid.jl")
include("Jp_right_toe_mid.jl")
include("p_left_ankle.jl")
include("Jp_left_ankle.jl")
include("p_right_ankle.jl")
include("Jp_right_ankle.jl")

# walking kinematics
include("Jp_com_left_wrt_right_ankle.jl")  
include("p_com_right_wrt_left_ankle.jl")
include("Jp_com_right_wrt_left_ankle.jl")  
include("p_com_wrt_left_ankle.jl")
include("Jp_com_wrt_left_ankle.jl")        
include("p_com_wrt_right_ankle.jl")
include("Jp_com_wrt_right_ankle.jl")       
include("p_left_wrt_right_ankle.jl")
include("Jp_left_wrt_right_ankle.jl")      
include("p_right_wrt_left_ankle.jl")
include("Jp_right_wrt_left_ankle.jl")


end
