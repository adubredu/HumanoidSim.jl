mutable struct Env
    objects::Vector{Symbol}
    trajectories::Dict
    dynamics::Dict
    init_position::Dict

    function Env(objects)
        trajectories = Dict()
        dynamics = Dict()
        init_position = Dict()
        for obj in objects
            trajectories[obj]=[]
            dynamics[obj]=nothing
            init_position[obj]=nothing
        end
        new(objects, trajectories, dynamics, init_position)
    end
end