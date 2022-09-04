module HumanoidSim

using PyBullet
using StaticArrays
using PyBullet.PythonCall
using GeometryBasics
using LinearAlgebra
using MeshCat 
using MeshCatMechanisms
using RigidBodyDynamics
using CoordinateTransformations
using Rotations
using Colors

include("visuals.jl")
include("robots/digit/types.jl")
include("robots/digit/get.jl")
include("robots/digit/load.jl")

export  Digit,
        DigitSim

export get_generalized_coordinates,
       load_digit

export initialize_arena!

end
