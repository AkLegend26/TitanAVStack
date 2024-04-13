module TitanAVStack

using VehicleSim
using Sockets
using Serialization
using LinearAlgebra
<<<<<<< HEAD
using DataStructures
using StaticArrays

include("example_project.jl")
include("client.jl")
=======
using StaticArrays
using RigidBodyDynamics
using Rotations
using DataStructures

include("Routing.jl")
include("control.jl")
include("decision_making.jl")
include("client.jl")
include("example_project.jl")
include("localization.jl")
>>>>>>> 7848d9e1c39286e6a3f5041a8fc83ca02e953e05

end # module TitanAVStack
