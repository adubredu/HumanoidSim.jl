function load_digit(p, init_pose=[0.0, 0.0, 0.9])
    path = joinpath(dirname(pathof(HumanoidSim)), "robots/digit/models")
    @show path
    p.setAdditionalSearchPath(path)
    id = p.loadURDF("digit_w_grippers.urdf", init_pose, useFixedBase=false)
    digit = Digit(pyconvert(Int64, id), p)
    return digit
end