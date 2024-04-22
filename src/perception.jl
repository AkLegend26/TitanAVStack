#Author :Ryan
# polyline and segment structures hw3
abstract type PolylineSegment end

function perp(x)
    [-x[2], x[1]]
end

struct InitialRay <: PolylineSegment
    point::MVector{2, Float64}
    tangent::MVector{2, Float64}
    normal::MVector{2, Float64}
    function InitialRay(point, next_point)
        tangent = MVector(next_point - point...)
        normalize!(tangent)  # Now possible because tangent is mutable
        normal = MVector(perp(tangent)...)
        new(MVector(point...), tangent, normal)
    end
end

struct TerminalRay <: PolylineSegment
    point::MVector{2, Float64}
    tangent::MVector{2, Float64}
    normal::MVector{2, Float64}
    function TerminalRay(point, prev_point)
        tangent = MVector(point - prev_point...)
        normalize!(tangent)  # Now possible because tangent is mutable
        normal = MVector(perp(tangent)...)
        new(MVector(point...), tangent, normal)
    end
end

struct StandardSegment <: PolylineSegment
    p1::MVector{2, Float64}
    p2::MVector{2, Float64}
    tangent::MVector{2, Float64}
    normal::MVector{2, Float64}
    function StandardSegment(p1, p2)
        tangent = MVector(p2 - p1...)
        normalize!(tangent)  # Now possible because tangent is mutable
        normal = MVector(perp(tangent)...)
        new(MVector(p1...), MVector(p2...), tangent, normal)
    end
end

# struct InitialRay <: PolylineSegment
#     point::SVector{2, Float64}
#     tangent::SVector{2, Float64}
#     normal::SVector{2, Float64}
#     function InitialRay(point, next_point)
#         tangent = next_point - point
#         tangent ./= norm(tangent)
#         normal = perp(tangent)
#         new(point, tangent, normal)
#     end
# end

# struct TerminalRay <: PolylineSegment
#     point::SVector{2, Float64}
#     tangent::SVector{2, Float64}
#     normal::SVector{2, Float64}
#     function TerminalRay(point, prev_point)
#         tangent = point - prev_point
#         tangent ./= norm(tangent)
#         normal = perp(tangent)
#         new(point, tangent, normal)
#     end
# end

# struct StandardSegment <: PolylineSegment
#     p1::SVector{2, Float64}
#     p2::SVector{2, Float64}
#     tangent::SVector{2, Float64}
#     normal::SVector{2, Float64}
#     function StandardSegment(p1, p2)
#         tangent = p2 - p1
#         tangent ./= norm(tangent)
#         normal = perp(tangent)
#         new(p1, p2, tangent, normal)
#     end
# end

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

struct DetectedObject
    position::SVector{3, Float64}
    dimensions::SVector{3, Float64}
    objectType::String
    polyline::Polyline
end

struct MyPerceptionType
    detectedObjects::Vector{DetectedObject}
    timestamp::Float64
end


# Function to process camera data and generate detected objects with polylines
function process_camera_data(camera_data)
    detected_objects = DetectedObject[]
    if isempty(camera_data.bounding_boxes)
        @info "No bounding boxes available for processing."
        return detected_objects
    end

    for bbox in camera_data.bounding_boxes
        @info "Processing bounding box" bbox=bbox
        position = SVector{3, Float64}(rand(), rand(), 0.0)  # Placeholder for actual position logic
        dimensions = SVector{3, Float64}(bbox[3] - bbox[1], bbox[4] - bbox[2], 2.0)
        
        # Ensure dimensions are valid
        if any(isnan.(dimensions)) || any(dimensions .<= 0)
            @warn "Invalid dimensions for detected object, skipping" dimensions=dimensions
            continue
        end

        points = [SVector{2, Float64}(rand(), rand()) for _ in 1:5] # fill in
        polyline = Polyline(points)
        detected_object = DetectedObject(position, dimensions, "vehicle", polyline)
        push!(detected_objects, detected_object)
        @info "Object detected" position=position dimensions=dimensions
    end

    if isempty(detected_objects)
        @info "No objects detected after processing all bounding boxes."
    else
        @info "Detected objects processed" count=length(detected_objects)
    end

    detected_objects
end


function update_perception_state(perception_channel::Channel{PerceptionState}, cam_channel::Channel{CameraMeasurement})
    @info "Perception update task initialized."
    while true
        try
            if isready(cam_channel)
                @info "Camera data is ready for processing."
                cam_data = take!(cam_channel)
                @info "Camera data taken from channel."

                detected_objects = process_camera_data(cam_data)
                @info "Camera data processed into detected objects." length=length(detected_objects)
                
                timestamp = cam_data.time
                perception_state = PerceptionState(detected_objects, timestamp)
                @info "Perception state created." timestamp=timestamp
                
                put!(perception_channel, perception_state)
                @info "Perception state put into channel."
            else
                @info "No camera measurements received, sleeping..."
                sleep(0.1)  # Adjust sleep time to prevent high CPU usage when idle
            end
        catch e
            @error "An error occurred in the perception update process" exception=(e, catch_backtrace())
            break  # You could also choose to continue here depending on error handling strategy
        end
    end
end

# Function to clear the perception channel if it's nearly full
function manage_perception_channel(perception_channel::Channel{PerceptionState})
    # Set a threshold for when to start clearing the channel (e.g., when it's 90% full)
    threshold = 0.9
    while true
        if length(perception_channel) > threshold * capacity(perception_channel)
            # Clearing the channel by consuming all items
            while isready(perception_channel)
                take!(perception_channel)  # This removes items from the channel
            end
            @info "Perception channel cleared to prevent overflow."
        end
        sleep(0.1)  # Adjust the sleep time based on the expected rate of data flow
    end
end

# start the channel management in an asynchronous task
function start_channel_manager(perception_channel::Channel{PerceptionState})
    @async manage_perception_channel(perception_channel)
end

function display_detected_objects(perception_state::MyPerceptionType)
    for obj in perception_state.detectedObjects
        println("Detected object at position: ", obj.position)
    end
end


function start_perception_system()
    cam_channel = Channel{CameraMeasurement}(10)
    perception_channel = Channel{PerceptionState}(10)
    @async update_perception_state(perception_channel, cam_channel)
    return cam_channel, perception_channel
end