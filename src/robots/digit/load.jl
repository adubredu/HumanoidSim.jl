packagepath() = joinpath(@__DIR__, "models")
urdfpath() = joinpath(packagepath(), "digit_w_grippers.urdf")

function load_digit(p, sim, init_pose=[0.0, 0.0, 0.92])
    path = joinpath(dirname(pathof(HumanoidSim)), "robots/digit/models") 
    p.setAdditionalSearchPath(path)
    id = p.loadURDF("digit_w_grippers.urdf", init_pose, useFixedBase=false)
    digit = Digit(pyconvert(Int64, id), p, sim)
    return digit
end

function mechanism(;floating=true, remove_fixed_joints=true, add_flat_ground=false)
    mechanism = RigidBodyDynamics.parse_urdf(urdfpath(); scalar_type=Float64, floating=floating, remove_fixed_tree_joints=remove_fixed_joints)
    if add_flat_ground
        frame = root_frame(mechanism)
        ground = RigidBodyDynamics.HalfSpace3D(Point3D(frame, 0.,0.,0.), FreeVector3D(frame, 0.,0.,1.))
        add_environment_primitive!(mechanism, ground)
    end
    remove_fixed_joints && remove_fixed_tree_joints!(mechanism)
    return mechanism
end

function set_nominal_state!(state::MechanismState)
    default_pose = Dict(
        "left-hip-roll" => 0.337,
        "left-hip-yaw" => 0.0,
        "left-hip-pitch" => 0.0,
        "left-knee" => 0.0,
        "left-shin" => 0.0,
        "left-tarsus" => 0.0,
        "left-toe-pitch" => -0.126,
        "left-toe-roll" => 0.0,
        "left-shoulder-roll" => 0.0,
        "left-shoulder-pitch" => 0.589,
        "left-shoulder-yaw" => 0.0,
        "left-elbow" => 0.0,
        "right-hip-roll" => -0.337,
        "right-hip-yaw" => 0.0,
        "right-hip-pitch" => 0.0,
        "right-knee" => 0.0,
        "right-shin" => 0.0,
        "right-tarsus" => 0.0,
        "right-toe-pitch" => 0.126,
        "right-toe-roll" => 0.0,
        "right-shoulder-roll" => 0.0,
        "right-shoulder-pitch" => -0.589,
        "right-shoulder-yaw" => 0.0,
        "right-elbow" => 0.0
    )
    mechanism = state.mechanism
    zero!(state)
    for k in keys(default_pose)
        set_configuration!(state, findjoint(mechanism, k), default_pose[k])
    end
    floating_base = first(out_joints(root_body(mechanism), mechanism))
    set_configuration!(state, floating_base, [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.92])
    return state
end

function load_digit_vis(sim::DigitSim)
    mech  = mechanism(add_flat_ground=true) 

    left_hand_link = findbody(mech, "left_elbow")
    right_hand_link = findbody(mech, "right_elbow")
    left_foot_link = findbody(mech, "left_toe_roll")
    right_foot_link = findbody(mech, "right_toe_roll")
    torso_link = findbody(mech, "torso")

    left_hand_ee = Point3D(default_frame(left_hand_link), 0.4, 0.0, -0.075)
    right_hand_ee = Point3D(default_frame(right_hand_link), 0.4, 0.0, -0.075)
    left_foot_ee = Point3D(default_frame(left_foot_link), 0.0, 0.0225, 0.0)
    right_foot_ee = Point3D(default_frame(right_foot_link), 0.0, -0.0225, -0.0)
    pelvis = Point3D(default_frame(torso_link), 0.0, 0.0, 0.0)

    lcontactpoints = [  [+0.05; -0.0; -0.15],
                        [+0.05; -0.11; +0.04],
                        [-0.05; -0.0; -0.15],
                        [-0.05; -0.11; +0.04]]
    rcontactpoints = [  [+0.05; +0.0; -0.15],
                        [+0.05; 0.11; +0.04],
                        [-0.05; -0.0; -0.15],
                        [-0.05; 0.11; +0.04]]
    left_foot_contact_points = [Point3D(default_frame(left_foot_link), p[1], p[2], p[3]) for p in lcontactpoints]
    right_foot_contact_points = [Point3D(default_frame(right_foot_link), p[1], p[2], p[3]) for p in rcontactpoints]

    contact_model =  RigidBodyDynamics.Contact.SoftContactModel(RigidBodyDynamics.Contact.hunt_crossley_hertz(k = 500e3), RigidBodyDynamics.Contact.ViscoelasticCoulombModel(0.8, 20e3, 100.))

    add_contact_point!(left_hand_link, RigidBodyDynamics.Contact.ContactPoint(left_hand_ee, contact_model))
    add_contact_point!(right_hand_link, RigidBodyDynamics.Contact.ContactPoint(right_hand_ee, contact_model))

    for point in left_foot_contact_points
        add_contact_point!(left_foot_link, RigidBodyDynamics.Contact.ContactPoint(point, contact_model))
    end
    for point in right_foot_contact_points
        add_contact_point!(right_foot_link, RigidBodyDynamics.Contact.ContactPoint(point, contact_model))
    end

    state = MechanismState(mech)
    # set_nominal_state!(state)

    mvis = MechanismVisualizer(state.mechanism, URDFVisuals(urdfpath(); package_path=[packagepath()]), sim.vis) 
    set_configuration!(mvis, configuration(state))
    # q_all = get_qall_from_state(state)
    # com = compute_com(q_all)
    # com_point = Point3D(root_frame(state.mechanism), com)

    # setelement!(mvis, com_point, 0.13, "COM")
    # setelement!(mvis, left_hand_ee, 0.05, "left_hand")
    # setelement!(mvis, right_hand_ee, 0.05, "right_hand") 
    # setelement!(mvis, pelvis, 0.1, "pelvis") 

    # for (i,point) in enumerate(left_foot_contact_points)
    #     setelement!(mvis, point, 0.01, "left_foot_contact_"*string(i))
    # end
    # for (i,point) in enumerate(right_foot_contact_points)
    #     setelement!(mvis, point, 0.01, "right_foot_contact_"*string(i))
    # end     
    
    sim.left_hand_ee = left_hand_ee
    sim.right_hand_ee = right_hand_ee
    sim.left_foot_ee = left_foot_ee
    sim.right_foot_ee = right_foot_ee
    sim.pelvis = pelvis
    sim.left_foot_contact_points = left_foot_contact_points
    sim.right_foot_contact_points = right_foot_contact_points
    sim.joints = [j for j in joints(state.mechanism) if typeof(j.joint_type) == Revolute{Float64}]
 
    sim.init_config = euler_configuration(state)
    
    sim.θᵣ = [0.337, 0, 0, 0, 0, 0, -0.126, 0, 0, 0.589, 0, 0, -0.337, 0, 0,
              0, 0, 0, 0.126, 0, 0, -0.589, 0, 0]
    sim.mvis = mvis 
    sim.state = state
    # sim.statecache = StateCache(sim.state.mechanism)
    sim.world_frame = root_frame(mech)
end