module HumanoidSim

using PyBullet
using MuJoCo
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
using Reexport
using Interpolations
using Interpolations: interpolate, Gridded, Linear

include("visuals.jl")
include("types.jl")

# digit
include("robots/digit/constants.jl")
include("robots/digit/types.jl")
include("robots/digit/utils.jl")
include("robots/digit/get.jl")
include("robots/digit/load.jl")
include("robots/digit/kinematics/kinematics.jl")
include("robots/digit/dynamics/dynamics.jl")
include("robots/digit/ik.jl")
include("robots/digit/controllers.jl")
include("robots/digit/set.jl")
include("robots/digit/simulate.jl")

# picklerick
include("robots/picklerick/types.jl")
include("robots/picklerick/get.jl")
include("robots/picklerick/set.jl")
include("robots/picklerick/load.jl")
include("robots/picklerick/simulate.jl")

export  Digit,
        DigitSim,
        Env

export Picklerick,
       PicklerickSim

export get_generalized_coordinates,
       get_motor_positions,
       get_motor_positions_from_q,
       get_qall_coordinates,
       load_digit_vis,
       load_digit, 
       update_state!,
       simulate,
       apply_velocity!,
       apply_position!,
       apply_torque!,
       set_joint_positions!

export load_picklerick_vis,
        load_picklerick        

export  make_posture_controller,
        make_balance_controller,
        posture_position_controller,
        posture_torque_controller,
        balance_torque_controller

export walk_ik


export initialize_arena!

# kinematics submodule
@reexport using .kinematics
const kin = kinematics
export kin

# dynamics submodule
@reexport using .dynamics
const dyn = dynamics
export dyn

end
