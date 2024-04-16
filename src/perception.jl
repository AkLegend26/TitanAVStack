using Serialization
using Sockets
using StaticArrays

# Structs to represent perception data, possibly could include bounding boxes,
# detected objects, etc., as detected and processed from camera and other sensor data.
struct DetectedObject
    position::SVector{3, Float64}  # 3D position
    dimensions::SVector{3, Float64}  # Width, Length, Height
    objectType::String  # Type of object, e.g., "vehicle", "pedestrian"
    polyline::Polyline  # Add this to represent linear features as polylines
end


struct PerceptionState
    detectedObjects::Vector{DetectedObject}
    timestamp::Float64
end

# This function processes camera measurements to detect objects
function process_camera_data(camera_data)
    detected_objects = DetectedObject[]
    for bbox in camera_data.bounding_boxes
        # Example dummy position and dimensions based on bounding box
        position = SVector{3, Float64}(rand(), rand(), 0.0)
        dimensions = SVector{3, Float64}(bbox[3] - bbox[1], bbox[4] - bbox[2], 2.0)
        # Construct a polyline from detected points (simplified example)
        points = [SVector{2, Float64}(rand(), rand()) for _ in 1:5]  # Dummy points
        polyline = Polyline(points)
        push!(detected_objects, DetectedObject(position, dimensions, "lane_boundary", polyline))
    end
    detected_objects
end


# Main function to process incoming sensor data and update perception state
function update_perception_state(perception_channel::Channel{PerceptionState}, cam_channel::Channel{CameraMeasurement})
    while true
        if isready(cam_channel)
            cam_data = take!(cam_channel)
            detected_objects = process_camera_data(cam_data)
            timestamp = cam_data.time
            perception_state = PerceptionState(detected_objects, timestamp)
            put!(perception_channel, perception_state)
        end
        sleep(0.001)  # Adjust sleep time based on expected data rate
    end
end

# Add to perception.jl
function update_perception_with_polylines(camera_data)
    detected_objects = process_camera_data(camera_data)  # Existing function
    # Add logic to create polylines from detected features
    for obj in detected_objects
        obj.polyline = create_polyline_from_features(obj.features)  # Define this function based on your feature detection logic
    end
end


# Example function to initialize channels and start the perception processing task
function start_perception_system()
    cam_channel = Channel{CameraMeasurement}(10)
    perception_channel = Channel{PerceptionState}(10)
    @async update_perception_state(perception_channel, cam_channel)
    return cam_channel, perception_channel
end
