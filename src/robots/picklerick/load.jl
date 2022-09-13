picklepackagepath() = joinpath(@__DIR__, "model")
pickleurdfpath() = joinpath(picklepackagepath(), "picklerick.urdf")

function load_picklerick(p, sim, init_pose=[0.0, 0.0, 2.5])
    path = joinpath(dirname(pathof(HumanoidSim)), "robots/picklerick/model") 
    p.setAdditionalSearchPath(path)
    id = p.loadURDF("picklerick.urdf", init_pose, useFixedBase=false)
    picklerick = Picklerick(pyconvert(Int64, id), p, sim)
    return picklerick
end

function pickle_mechanism(;floating=true, remove_fixed_joints=true, add_flat_ground=false)
    pickle_mechanism = RigidBodyDynamics.parse_urdf(pickleurdfpath(); scalar_type=Float64, floating=floating, remove_fixed_tree_joints=remove_fixed_joints)
    if add_flat_ground
        frame = root_frame(pickle_mechanism)
        ground = RigidBodyDynamics.HalfSpace3D(Point3D(frame, 0.,0.,0.), FreeVector3D(frame, 0.,0.,1.))
        add_environment_primitive!(pickle_mechanism, ground)
    end
    remove_fixed_joints && remove_fixed_tree_joints!(pickle_mechanism)
    return pickle_mechanism
end 

function load_picklerick_vis(sim::PicklerickSim)
    mech  = pickle_mechanism(add_flat_ground=true) 
    state = MechanismState(mech)
    # set_nominal_state!(state)

    mvis = MechanismVisualizer(state.mechanism, URDFVisuals(pickleurdfpath(); package_path=[picklepackagepath()]), sim.vis) 
    set_configuration!(mvis, configuration(state))  
    
    sim.θᵣ = [0.337, 0, 0, 0, 0, 0, -0.126, 0, 0, 0.589, 0, 0, -0.337, 0, 0,
              0, 0, 0, 0.126, 0, 0, -0.589, 0, 0]
    sim.mvis = mvis 
    sim.state = state 
end