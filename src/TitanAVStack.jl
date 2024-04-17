module TitanAVStack

using VehicleSim
using Sockets
using Serialization
using LinearAlgebra
using DataStructures
using StaticArrays
using RigidBodyDynamics
using Rotations
using PolygonInbounds
using MeshCat

# Define ControlState here
mutable struct ControlState
    previous_cross_track_error::Float64
    delta_time::Float64
    error_integral::Float64

    ControlState() = new(0.0, 0.1, 0.0)  # Initialize with default values
end

include("Routing.jl")
include("perception.jl")
include("control.jl")
include("decision_making.jl")
include("client.jl")
include("example_project.jl")
include("localization.jl")

end # module TitanAVStack
