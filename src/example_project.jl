struct MyLocalizationType
    position::SVector{3, Float64}  # [x, y]
    orientation::SVector{4, Float64}      # θ
    velocity::SVector{3, Float64}  # [vx, vy]
    angular_velocity::SVector{3, Float64} 
end


struct MyPerceptionType
    field1::Int
    field2::Float64
end

# mutable struct ControlState
#     previous_cross_track_error::Float64
#     delta_time::Float64

#     ControlState() = new(0.0, 0.1)  # initialize with default values
# end

###Running working localization code
# function localize(gps_channel::Channel{GPSMeasurement}, imu_channel::Channel{IMUMeasurement}, localization_state_channel::Channel{MyLocalizationType})
#     @info "Localization process started."
#     ekf = ekf_initialize()  # Initialize the EKF\
#     while true
#         try
#             if isready(gps_channel)
#                 gps_measurement = take!(gps_channel)
#                 gps_data = [gps_measurement.lat, gps_measurement.long, gps_measurement.heading]
#                 ekf_update!(ekf, gps_data, h_gps, Jac_h_gps, ekf.measurement_noise_gps)
#                 #@info "Processed GPS data: ", gps_data
#             else
#                 #@info "Waiting for GPS data..."
#             end

#             if isready(imu_channel)
#                 imu_measurement = take!(imu_channel)
#                 imu_data = [imu_measurement.linear_vel..., imu_measurement.angular_vel...]
#                 ekf_update!(ekf, imu_data, h_imu, jac_h_imu, ekf.measurement_noise_imu)
#                 #@info "Processed IMU data: ", imu_data
#             else
#                 #@info "Waiting for IMU data..."
#             end

#             if isready(gps_channel) || isready(imu_channel)
#                 localized_state = MyLocalizationType(
#                     [ekf.state[1], ekf.state[2]],  # Position x, y
#                     ekf.state[3],  # Orientation (assuming θ, single scalar)
#                     [ekf.state[4], ekf.state[5]]  # Velocity vx, vy (corrected assuming ekf.state includes these)
#                 )
#                 if isready(localization_state_channel)
#                     take!(localization_state_channel)  # Clear the channel if full
#                 end
#                 put!(localization_state_channel, localized_state)
#                 #@info "Published updated localization state."
#             end

#             sleep(0.1)  # Manage loop timing
#         catch e
#             @error "Error in localization process" exception=(e, catch_backtrace())
#             break  # Optionally break or continue based on error severity
#         end
#     end
# end
### Above is wokring code

function localize(gt_channel::Channel{GroundTruthMeasurement}, localization_state_channel::Channel{MyLocalizationType})
    @info "Localization process started using ground truth data."
    while true
        try
            if isready(gt_channel)
                gt_data = take!(gt_channel)

                position = gt_data.position  # 3D position [x, y, z]
                velocity = gt_data.velocity  # 3D velocity [vx, vy, vz]
                orientation = gt_data.orientation  # Quaternion [w, x, y, z]
                angular_velocity = gt_data.angular_velocity  # Angular velocity [omega_x, omega_y, omega_z]

                localized_state = MyLocalizationType(
                    position,        # Ground truth position
                    orientation,     # Quaternion orientation
                    velocity,        # Ground truth velocity
                    angular_velocity # Ground truth angular velocity
                )

                @info "Putting data to localization_state_channel"
                if isready(localization_state_channel)
                    take!(localization_state_channel)  # Clear the channel if full
                end

                put!(localization_state_channel, localized_state)
                @info "Data put to localization_state_channel"
            else
                sleep(0.1)  # Adjust the timing based on your system's needs
            end
        catch e
            @error "Error in localization process using ground truth data" exception=(e, catch_backtrace())
            break  # Optionally break or continue based on error severity
        end
    end
end


function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
    while true
        if isready(cam_meas_channel)
            cam_data = take!(cam_meas_channel)
            @info "Camera data received for processing."
            detected_objects = process_camera_data(cam_data)
            if isempty(detected_objects)
                @info "No objects detected, skipping."
                continue
            end
            timestamp = cam_data.time  # Make sure the timestamp is correctly retrieved
            perception_state = MyPerceptionType(detected_objects, timestamp)
            put!(perception_state_channel, perception_state)
            @info "Perception data processed and put to channel."
        else
            @info "No camera measurements received, sleeping..."
            sleep(0.1)
        end
        # Optional: Check if channels are synchronized
        @info "Checking channel statuses", isready(localization_state_channel), isready(perception_state_channel)
    end
end



# function perception(cam_meas_channel, localization_state_channel, perception_state_channel)
#     @info "Perception..."
#     # set up stuff
#     while true
#         sleep(0.001)
#         fresh_cam_meas = []
#         while isready(cam_meas_channel)
#             meas = take!(cam_meas_channel)
#             push!(fresh_cam_meas, meas)
#         end

#         latest_localization_state = fetch(localization_state_channel)
        
#         # process bounding boxes / run ekf / do what you think is good

#         perception_state = MyPerceptionType(0,0.0)
#         if isready(perception_state_channel)
#             take!(perception_state_channel)
#         end
#         put!(perception_state_channel, perception_state)
#     end
#     @info "Perception out!!"
# end

# function decision_making(localization_state_channel, map_segments, socket, target_road_segment_id, control_state::ControlState)
#     @info "Decision making task started..."
#     while true 
#         try
#             if isready(localization_state_channel)
#                 latest_localization_state = fetch(localization_state_channel)
#                 @info "Finding current segment..." latest_localization_state.position
#                 yaw = extract_yaw_from_quaternion(latest_localization_state.orientation)

#                 current_segment_id = find_current_segment(latest_localization_state.position[1:2], map_segments)
#                 if current_segment_id == -1
#                     @error "No segment found for position" latest_localization_state.position
#                     sleep(1)
#                     continue
#                 end

#                 @info "Current segment found" segment_id=current_segment_id
#                 path = shortest_path(current_segment_id, target_road_segment_id, map_segments)
#                 if isempty(path)
#                     @warn "No path found" current_segment_id
#                     continue
#                 end

#                 @info "Navigating through path..." length(path)
#                 for segment_id in path
#                     navigate_segment(map_segments[segment_id], latest_localization_state, yaw, socket, control_state) 
#                 end                
#             else
#                 @info "localization channel not ready"
#                 sleep(0.1)
#             end
#         catch e
#             @error "An error occurred during the decision making process" exception=(e, catch_backtrace())
#         end
#     end
# end

function decision_making(localization_state_channel, perception_state_channel, map_segments, socket, target_road_segment_id, control_state::ControlState)
    @info "Decision making task started..."
    while true
        try
            # Wait until both localization and perception data are available
            if isready(localization_state_channel) && isready(perception_state_channel)
                latest_localization_state = fetch(localization_state_channel)
                latest_perception_state = fetch(perception_state_channel)

                @info "Data fetched from channels" localization_data=latest_localization_state.position perception_data=length(latest_perception_state.detectedObjects)

                yaw = extract_yaw_from_quaternion(latest_localization_state.orientation)
                @info "Processed yaw from orientation" yaw=yaw

                current_segment_id = find_current_segment(latest_localization_state.position[1:2], map_segments)
                if current_segment_id == -1
                    @error "No segment found for position" position=latest_localization_state.position
                    sleep(1)  # Sleep before next attempt to give time for system state change
                    continue
                end

                @info "Current segment identified" segment_id=current_segment_id
                path = shortest_path(current_segment_id, target_road_segment_id, map_segments)
                if isempty(path)
                    @warn "Pathfinding returned no path" current_segment_id=current_segment_id
                    continue
                end

                @info "Path found, navigating segments" path_length=length(path)
                for segment_id in path
                    navigate_segment(map_segments[segment_id], latest_localization_state, yaw, socket, control_state, latest_perception_state)
                end
            else
                if !isready(localization_state_channel)
                    @info "Localization channel not ready"
                end
                if !isready(perception_state_channel)
                    @info "Perception channel not ready"
                end
                sleep(0.1)  # Wait briefly to check channel readiness again
            end
        catch e
            @error "An error occurred during the decision making process" exception=(e, catch_backtrace())
            sleep(1)  # Error recovery sleep
        end
    end
    @info "Perception out!!"
end

#                 @info "Current segment found" segment_id=current_segment_id
#                 path = shortest_path(current_segment_id, target_road_segment_id, map_segments)
#                 if isempty(path)
#                     @warn "No path found" current_segment_id
#                     continue
#                 end

#                 @info "Navigating through path..." length(path)
#                 for segment_id in path
#                     navigate_segment(map_segments[segment_id], latest_localization_state, yaw, socket, control_state) 
#                 end                
#             else
#                 @info "localization channel not ready"
#                 sleep(0.1)
#             end
#         catch e
#             @error "An error occurred during the decision making process" exception=(e, catch_backtrace())
#         end
#     end
# end

function decision_making(localization_state_channel, perception_state_channel, map_segments, socket, target_road_segment_id, control_state::ControlState)
    @info "Decision making task started..."
    while true
        try
            # Wait until both localization and perception data are available
            if isready(localization_state_channel) && isready(perception_state_channel)
                latest_localization_state = fetch(localization_state_channel)
                @info "Finding current segment..."
                latest_localization_state.position
                yaw = extract_yaw_from_quaternion(latest_localization_state.orientation)
                current_segment_id = find_current_segment(latest_localization_state.position[1:2], map_segments)
                if current_segment_id == -1
                    @error "No segment found for position"
                    latest_localization_state.position
                    sleep(1)
                    continue
                end
                @info "Current segment found"
                @info current_segment_id
                segment_id = current_segment_id
                path = shortest_path(current_segment_id, target_road_segment_id, map_segments)
                if isempty(path)
                    @warn "No path found"
                    current_segment_id
                    continue
                end
                @info "Navigating through path..."
                length(path)
                @info path

                # Get the lookahead segments
                # lookahead_segments = Vector{VehicleSim.RoadSegment}()
                # for i in 2:length(path)
                #     push!(lookahead_segments, map_segments[path[i]])
                # end

                for segment_id in path
                    segment = map_segments[segment_id]
                    while true
                        result = pure_pursuit_navigate(segment, localization_state_channel,latest_localization_state, socket, yaw)
                        if result === true
                            break  # Successfully navigated the segment
                        elseif result === false
                            sleep(0.1)  # Wait before trying to navigate the segment again
                        else
                            @error "Unexpected result from pure_pursuit_navigate" result
                            break
                        end
                    end
                end
            else
                if !isready(localization_state_channel)
                    @info "Localization channel not ready"
                end
                if !isready(perception_state_channel)
                    @info "Perception channel not ready"
                end
                sleep(0.1)  # Wait briefly to check channel readiness again
            end
        catch e
            @error "An error occurred during the decision making process" exception=(e, catch_backtrace())
            sleep(1)  # Error recovery sleep
        end
        sleep(0.1)  # Check at a frequency appropriate for your application's safety requirements
    end
end



function navigate_segment(segment, localization_state, yaw, socket, control_state, perception_state)
    # Placeholder for actual navigation logic
    # Example: Use perception data to avoid obstacles
    if !isempty(perception_state.detectedObjects)
        # Logic to modify driving commands based on perceived objects
    end

    # Example driving commands based on the segment and localization data
    steering_angle = compute_steering_angle(segment, localization_state, yaw)
    target_vel = compute_target_velocity(segment, localization_state)

    cmd = (steering_angle, target_vel, true)
    serialize(socket, cmd)
end

function monitor_vehicle_safety(perception_channel, vehicle_state_channel, control_channel)
    @info "Starting vehicle safety monitoring..."
    while true
        perception_state = fetch(perception_channel)
        vehicle_state = fetch(vehicle_state_channel)
        
        for obj in perception_state.detectedObjects
            if obj.objectType == "obstacle"
                collision_risk = check_collisions(vehicle_state, obj)
                if collision_risk.is_risky
                    take_preventive_action(control_channel, collision_risk)  # Implement actions based on risk assessment
                end
            end
        end
        sleep(0.1)  # Check at a frequency appropriate for your application's safety requirements
    end
end



function isfull(ch::Channel)
    length(ch.data) ≥ ch.sz_max
end

function quaternion_angle_difference(q1::SVector{4, Float64}, q2::SVector{4, Float64})
    # Normalize the quaternions to ensure they represent valid rotations
    q1_normalized = normalize(q1)
    q2_normalized = normalize(q2)
    
    # Calculate the dot product, which is cos(theta) where theta is the angle between them
    cos_theta = dot(q1_normalized, q2_normalized)
    
    # Calculate the angle in radians
    theta = acos(clamp(cos_theta, -1.0, 1.0))  # Clamp for numerical stability
    return theta
end

function test_localization(gt_channel, localization_state_channel)
    @info "Starting localization testing task..."

    gt_list = []  # Store ground truth measurements in a list
    errors = Dict("position" => [], "velocity" => [], "orientation" => [])

    try
        while true
            sleep(0.001)  # Polling interval

            if isready(gt_channel)
                gt = take!(gt_channel)
                push!(gt_list, gt)
                #@info "Ground truth data received."
            end
            if isready(localization_state_channel) && !isempty(gt_list)
                est_state = take!(localization_state_channel)
                gt_state = last(gt_list)  # Assuming the latest ground truth is what we want to compare against

                pos_error = norm(est_state.position - gt_state.position[1:2])  # Compare only x, y
                vel_error = norm(est_state.velocity - gt_state.velocity[1:2])  # Compare velocity magnitudes
                ori_error = 0.0  # Initialize orientation error
                
                # Check and calculate orientation error
                if typeof(est_state.orientation) == SVector{4, Float64} && typeof(gt_state.orientation) == SVector{4, Float64}
                    ori_error = quaternion_angle_difference(est_state.orientation, gt_state.orientation)
                    #@info "Orientation difference computed" ori_error
                else
                    #@error "Invalid quaternion data types"
                end

                # Store the computed errors
                push!(errors["position"], pos_error)
                push!(errors["velocity"], vel_error)
                push!(errors["orientation"], ori_error)

                @info "Errors computed" position_error=pos_error velocity_error=vel_error orientation_error=ori_error
            elseif isempty(gt_list)
                @warn "No ground truth data available for comparison."
            end
        end
    catch e
        @error "An error occurred in the localization testing task" exception=(e, catch_backtrace())
    end

    # Calculate and log average errors
    avg_pos_error = mean(errors["position"])
    avg_vel_error = mean(errors["velocity"])
    avg_ori_error = mean(errors["orientation"])

    @info "Testing completed."
    @info "Average Position Error: $avg_pos_error"
    @info "Average Velocity Error: $avg_vel_error"
    @info "Average Orientation Error: $avg_ori_error"
end

function my_client(host::IPAddr=IPv4(0), port=4444)
    @info "BabyyLegend"
    #vis = Visualizer()
    #open(vis)
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.city_map()
    #VehicleSim.view_map(vis, map_segments)
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_velocity = 0.0 
    steering_angle = 0.0
    controlled = true

    client_info_string = 
        "********************
      TITANS REVING
      ********************"
    @info client_info_string

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    errormonitor(@async while isopen(socket)  # Check if the socket is still open
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false

        while isopen(socket) && bytesavailable(socket) > 0
            measurement_msg = deserialize(socket)
            received = true
            break  # Exit this inner loop if data is received
        end

        if !received
            continue  # Skip to the next iteration if no new data was received
        end

        target_map_segment = measurement_msg.target_segment
        ego_vehicle_id = measurement_msg.vehicle_id
        for meas in measurement_msg.measurements
            if meas isa GPSMeasurement
                !isfull(gps_channel) && put!(gps_channel, meas)
            elseif meas isa IMUMeasurement
                !isfull(imu_channel) && put!(imu_channel, meas)
            elseif meas isa CameraMeasurement
                !isfull(cam_channel) && put!(cam_channel, meas)
            elseif meas isa GroundTruthMeasurement
                !isfull(gt_channel) && put!(gt_channel, meas)
            elseif meas isa Int
                @info meas
            else
                @info typeof(meas)
            end
        end
    end)

    targets = 51
    errormonitor(@async localize(gt_channel, localization_state_channel))
    #@async localize(gps_channel, imu_channel, localization_state_channel)
    #@async test_localization(gt_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    # sleep(0.1)
    errormonitor(@async decision_making(localization_state_channel, map_segments, socket, targets))
end
