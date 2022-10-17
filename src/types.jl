mutable struct Env
    objects::Vector{Symbol}
    trajectories::Dict
    dynamics::Dict
    init_position::Dict
    geometries::Dict

    function Env(objects)
        trajectories = Dict()
        dynamics = Dict()
        init_position = Dict()
        geometries = Dict()
        for obj in objects
            trajectories[obj]=[]
            dynamics[obj]=nothing
            init_position[obj]=nothing
            geometries[obj]=[]
        end
        new(objects, trajectories, dynamics, init_position, geometries)
    end
end