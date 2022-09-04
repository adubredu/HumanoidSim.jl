module HumanoidSim

using PyBullet
using StaticArrays
using PyBullet.PythonCall
using MeshCat 
using MeshCatMechanisms
using CoordinateTransformations
using Rotations

include("robots/digit/types.jl")
include("robots/digit/get.jl")
include("robots/digit/load.jl")

export Digit 

export get_generalized_coordinates,
       load_digit

end
