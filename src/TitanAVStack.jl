module TitanAVStack

using VehicleSim
using Sockets
using Serialization
using StaticArrays
using LinearAlgebra

include("client.jl")
include("example_project.jl")
include("localization.jl")

end # module TitanAVStack
