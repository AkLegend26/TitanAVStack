using Serialization
using Sockets
using StaticArrays

# polyline and segment structures hw3
abstract type PolylineSegment end

function perp(x)
    [-x[2], x[1]]
end

struct InitialRay <: PolylineSegment
    point::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function InitialRay(point, next_point)
        tangent = next_point - point
        tangent ./= norm(tangent)
        normal = perp(tangent)
        new(point, tangent, normal)
    end
end

struct TerminalRay <: PolylineSegment
    point::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function TerminalRay(point, prev_point)
        tangent = point - prev_point
        tangent ./= norm(tangent)
        normal = perp(tangent)
        new(point, tangent, normal)
    end
end

struct StandardSegment <: PolylineSegment
    p1::SVector{2, Float64}
    p2::SVector{2, Float64}
    tangent::SVector{2, Float64}
    normal::SVector{2, Float64}
    function StandardSegment(p1, p2)
        tangent = p2 - p1
        tangent ./= norm(tangent)
        normal = perp(tangent)
        new(p1, p2, tangent, normal)
    end
end

struct Polyline
    segments::Vector{PolylineSegment}
    function Polyline(points)
        segments = Vector{PolylineSegment}()
        N = length(points)
        @assert N ≥ 2
        initial_ray = InitialRay(points[1], points[2])
        push!(segments, initial_ray)
        for i = 1:(N-1)
            seg = StandardSegment(points[i], points[i+1])
            push!(segments, seg)
        end
        terminal_ray = TerminalRay(points[end], points[end-1])
        push!(segments, terminal_ray)
        new(segments)
    end
end

# Perception data structs
struct DetectedObject
    position::SVector{3, Float64}
    dimensions::SVector{3, Float64}
    objectType::String
    polyline::Polyline
end

struct PerceptionState
    detectedObjects::Vector{DetectedObject}
    timestamp::Float64
end

# Function to process camera data and generate detected objects with polylines
function process_camera_data(camera_data)
    detected_objects = DetectedObject[]
    for bbox in camera_data.bounding_boxes
        position = SVector{3, Float64}(rand(), rand(), 0.0)
        dimensions = SVector{3, Float64}(bbox[3] - bbox[1], bbox[4] - bbox[2], 2.0)
        points = [SVector{2, Float64}(rand(), rand()) for _ in 1:5]  # Dummy points for demonstration
        polyline = Polyline(points)
        push!(detected_objects, DetectedObject(position, dimensions, "vehicle", polyline))
    end
    detected_objects
end

# Function to update the perception state with new data from the camera channel
function update_perception_state(perception_channel::Channel{PerceptionState}, cam_channel::Channel{CameraMeasurement})
    while true
        if isready(cam_channel)
            cam_data = take!(cam_channel)
            detected_objects = process_camera_data(cam_data)
            timestamp = cam_data.time
            perception_state = PerceptionState(detected_objects, timestamp)
            put!(perception_channel, perception_state)
        end
        sleep(0.001)
    end
end

# Function to initialize channels and start the perception processing task
function start_perception_system()
    cam_channel = Channel{CameraMeasurement}(10)
    perception_channel = Channel{PerceptionState}(10)
    @async update_perception_state(perception_channel, cam_channel)
    return cam_channel, perception_channel
end
