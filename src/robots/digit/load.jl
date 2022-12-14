packagepath() = joinpath(@__DIR__, "models")
urdfpath() = joinpath(packagepath(), "digit_w_grippers.urdf")

function load_digit(sim; init_pose=[0.0, 0.0, 0.92], 
                    engine=:MuJoCo)
    if engine == :PyBullet
        p = pybullet
        p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        path = joinpath(dirname(pathof(HumanoidSim)), "robots/digit/models") 
        p.setAdditionalSearchPath(path)
        id = p.loadURDF("digit_w_grippers.urdf", init_pose, useFixedBase=false)
        digit = Digit(pyconvert(Int64, id), p, sim, engine)

    elseif engine == :MuJoCo
        path = joinpath(dirname(pathof(HumanoidSim)), "robots/digit/models")
        robot_path = joinpath(path, "digit-v3.xml")
        physics = mujoco.Physics.from_xml_path(robot_path) 
        # physics.data.qpos[0:2] = init_pose
        digit = Digit(-1, physics, sim, engine)
        set_nominal_configuration(digit)
    end
    return digit
end

function load_mujoco_digit(p, sim, init_pose=[0.0, 0.0, 0.95])
    path = joinpath(dirname(pathof(HumanoidSim)), "robots/digit/models")
    robot_path = joinpath(path, "digit-v3.xml")
    physics = p.Physics.from_xml_path(robot_path) 
    physics.data.qpos[0:2] = init_pose
    digit = Digit(1, physics, sim)
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
        "left-shoulder-roll" => -0.15,
        "left-shoulder-pitch" => 1.1,
        "left-shoulder-yaw" => 0.0,
        "left-elbow" => -0.145,
        "right-hip-roll" => -0.337,
        "right-hip-yaw" => 0.0,
        "right-hip-pitch" => 0.0,
        "right-knee" => 0.0,
        "right-shin" => 0.0,
        "right-tarsus" => 0.0,
        "right-toe-pitch" => 0.126,
        "right-toe-roll" => 0.0,
        "right-shoulder-roll" => 0.15,
        "right-shoulder-pitch" => -1.1,
        "right-shoulder-yaw" => 0.0,
        "right-elbow" => 0.145
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

function load_digit_vis(sim::DigitSim; floating=true)
    mech  = mechanism(floating=floating, add_flat_ground=true) 
    state = MechanismState(mech) 

    mvis = MechanismVisualizer(state.mechanism, URDFVisuals(urdfpath(); package_path=[packagepath()]), sim.vis) 
    set_configuration!(mvis, configuration(state))  
    
    sim.????? = [0.337, 0, 0, 0, 0, 0, -0.126, 0, 0, 0.589, 0, 0, -0.337, 0, 0,
              0, 0, 0, 0.126, 0, 0, -0.589, 0, 0]
    sim.?????[3] = -0.1
    sim.?????[15] = -sim.?????[3]
    sim.mvis = mvis 
    sim.state = state 
    sim.world_frame = root_frame(mech)
end