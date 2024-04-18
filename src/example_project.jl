struct MyLocalizationType
    position::SVector{3, Float64}  # [x, y]
    orientation::SVector{4, Float64}      # θ
    velocity::SVector{3, Float64}  # [vx, vy]
    angular_velocity::SVector{3, Float64} 
end


struct DetectedObject
    id::Int
    bbox::SVector{4, Int}  # Top, Left, Bottom, Right in pixel coordinates
    world_position::Vector{Float64}  # Position in world coordinates
    world_velocity::Vector{Float64}  # Velocity in world coordinates
    threat_level::Float64  # An assessment of collision risk
end

struct MyPerceptionType
    detected_objects::Vector{DetectedObject}
    timestamp::Float64
end


###Running working localization code
function localize(gps_channel::Channel{GPSMeasurement}, imu_channel::Channel{IMUMeasurement}, localization_state_channel::Channel{MyLocalizationType})
    @info "Localization process started."
    ekf = ekf_initialize()  # Initialize the EKF
    while true
        try
            if isready(gps_channel)
                gps_measurement = fetch(gps_channel)
                gps_data = [gps_measurement.lat, gps_measurement.long, gps_measurement.heading]
                ekf_update!(ekf, gps_data, h_gps, Jac_h_gps, ekf.measurement_noise_gps)
                #@info "Processed GPS data: ", gps_data
            else
                #@info "Waiting for GPS data..."
            end

            if isready(imu_channel)
                imu_measurement = fetch(imu_channel)
                imu_data = [imu_measurement.linear_vel..., imu_measurement.angular_vel...]
                ekf_update!(ekf, imu_data, h_imu, jac_h_imu, ekf.measurement_noise_imu)
                #@info "Processed IMU data: ", imu_data
            else
                #@info "Waiting for IMU data..."
            end

            if isready(gps_channel) || isready(imu_channel)
                localized_state = MyLocalizationType(
                ekf.state[1:3],  # Position x, y, z
                ekf.state[4:7],  # Quaternion qx, qy, qz, qw
                ekf.state[8:10],  # Velocity vx, vy, vz
                ekf.state[11:13]  # Angular velocity ωx, ωy, ωz
                )
                if isready(localization_state_channel)
                    take!(localization_state_channel)  # Clear the channel if full
                end
                put!(localization_state_channel, localized_state)
                #@info "Published updated localization state."
            end

            if length(gps_channel.data) == 1
                while length(gps_channel.data) != 0
                    take!(gps_channel)
                end
            end
            if length(imu_channel.data) == 1
                while length(imu_channel.data) != 0
                    take!(imu_channel)
                end
            end

            sleep(0.00001)  # Manage loop timing
        catch e
            @error "Error in localization process" exception=(e, catch_backtrace())
            continue  # Optionally break or continue based on error severity
        end
    end
end
### Above is wokring code

function localize(gt_channel::Channel{GroundTruthMeasurement}, localization_state_channel::Channel{MyLocalizationType}, ego_vehicle_id)
    @info "Localization process started using ground truth data."
    while true
        try
            if isready(gt_channel)
                gt_data = take!(gt_channel)
                

                # Check if the current ground truth data is for the ego vehicle
                if gt_data.vehicle_id == ego_vehicle_id
                    position = gt_data.position  # 3D position [x, y, z]
                    velocity = gt_data.velocity  # 3D velocity [vx, vy, vz]
                    orientation = gt_data.orientation  # Quaternion [w, x, y, z]
                    angular_velocity = gt_data.angular_velocity  # Angular velocity [omega_x, omega_y, omega_z]
                    #@info "Ground truth data taken" position, velocity

                    localized_state = MyLocalizationType(
                        position,        # Ground truth position
                        orientation,     # Quaternion orientation
                        velocity,        # Ground truth velocity
                        angular_velocity # Ground truth angular velocity
                    )

                    # Ensure the localization state channel is not full before putting new data
                    if isready(localization_state_channel)
                        take!(localization_state_channel)  # Clear the channel if full
                    end

                    put!(localization_state_channel, localized_state)
                    #@info "Published ground truth localization state for ego vehicle."
                else
                    #@info "Ground truth data received for other vehicle. Ignoring..."
                end
            else
                #@info "GT WASn'T READY"
                sleep(0.05)  # Adjust the timing based on your system's needs
            end
        catch e
            @error "Error in localization process using ground truth data" exception=(e, catch_backtrace())
            break  # Optionally break or continue based on error severity
        end
    end
end

function decision_making(localization_state_channel, map_segments, socket, target_road_segment_id)
    @info "Decision making task started..."
    while true
        try
            if isready(localization_state_channel)
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
                @info path
                if isempty(path)
                    @warn "No path found"
                    current_segment_id
                    continue
                end

                # Get the lookahead segments
                # lookahead_segments = Vector{VehicleSim.RoadSegment}()
                # for i in 2:length(path)
                #     push!(lookahead_segments, map_segments[path[i]])
                # end

                for segment_id in path
                    segment = map_segments[segment_id]
                    @info "Navigating segment" segment_id=segment_id
                    while true
                        result = pure_pursuit_navigate(segment, localization_state_channel, latest_localization_state, socket, yaw)
                        if result === true
                            # Check if we have reached the end of the path
                            if segment_id == last(path)
                                @info "End of path reached. Sending stop command."
                                send_commands(0.0, 0.0, socket)
                                return 
                            end
                            break  
                        elseif result === false
                            sleep(0.001)  
                        else
                            @error "Unexpected result from pure_pursuit_navigate" result
                            break
                        end
                    end
                end
            else
                sleep(0.01)
            end
        catch e
            @error "An error occurred during the decision making process" exception=(e, catch_backtrace())
        end
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
            sleep(1)  # Polling interval

            if isready(gt_channel)
                gt = take!(gt_channel)
                push!(gt_list, gt)
                #@info "Ground truth data received."
            end
            if isready(localization_state_channel) && !isempty(gt_list)
                est_state = take!(localization_state_channel)
                gt_state = last(gt_list)  # Assuming the latest ground truth is what we want to compare against

                pos_error = norm(est_state.position[1:2] - gt_state.position[1:2])  # Compare only x, y
                vel_error = norm(est_state.velocity[1:2] - gt_state.velocity[1:2])  # Compare velocity magnitudes
                ori_error = 0.0  # Initialize orientation error
                
                # Check and calculate orientation error
                if typeof(est_state.orientation) == SVector{4, Float64} && typeof(gt_state.orientation) == SVector{4, Float64}
                    @info est_state.orientation
                    @info gt_state.orientation
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

    gps_channel = Channel{GPSMeasurement}(2)
    imu_channel = Channel{IMUMeasurement}(2)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(2)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)


    client_info_string = 
        "********************
      TITANS REVING
      ********************"
    @info client_info_string

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 1 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

    errormonitor(@async while true
        # This while loop reads to the end of the socket stream (makes sure you
        # are looking at the latest messages)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
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
                @info "Received data of type $(typeof(meas))"
            end
        end
    end)

    targets = 82
    #errormonitor(@async localize(gt_channel, localization_state_channel, ego_vehicle_id))
    errormonitor(@async localize(gps_channel, imu_channel, localization_state_channel))
    #@async test_localization(gt_channel, localization_state_channel)
    @async perception(cam_channel, localization_state_channel, perception_state_channel)
    sleep(0.001)
    errormonitor(@async decision_making(localization_state_channel, map_segments, socket, targets))
end

function perception(cam_channel, localization_state_channel, perception_state_channel)
    @info "Perception process started."
    object_id_counter = 1

    while true
        fresh_cam_meas = []
    
        while isready(cam_channel)
            meas = take!(cam_channel)
            push!(fresh_cam_meas, meas)
            #@info "New camera measurement received" camera_id=meas.camera_id num_bboxes=length(meas.bounding_boxes) box=meas.bounding_boxes
        end
    
        if !isempty(fresh_cam_meas)
            latest_localization_state = fetch(localization_state_channel)
            #@info "Processing new camera measurements" count=length(fresh_cam_meas)
    
            for meas in fresh_cam_meas
                detected_objects = []
                for bbox in meas.bounding_boxes
                    try
                        position, velocity = bbox_to_world(meas, bbox, latest_localization_state)
                    catch e
                        #@error "Failed to transform bounding box to world coordinates" exception=(e, catch_backtrace())
                        continue  # Skip this bbox or handle the error appropriately
                    end                    

                    threat_level = assess_threat(position, velocity, latest_localization_state)
                    detected_object = DetectedObject(object_id_counter, bbox, position, velocity, threat_level)
                    push!(detected_objects, detected_object)
                    object_id_counter += 1
    
                   # @info "Detected object" id=detected_object.id threat_level=detected_object.threat_level position=position velocity=velocity
                end
    
                if !isempty(detected_objects)
                    perception_state = MyPerceptionType(detected_objects, meas.time)
                    if isready(perception_state_channel)
                        take!(perception_state_channel)  # Clear the channel if full
                    end
                    put!(perception_state_channel, perception_state)
                end
            end
        else
            #@info "No new camera measurements available"
        end
        sleep(0.01)  # Reduce CPU usage
    end    
end


function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
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

    client_info_string = 
        "********************
      TITANS REVING
      ********************"
    @info client_info_string

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 1 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)




    @async while isopen(socket)
        sleep(0.001)
        local measurement_msg
        received = false
        while true
            @async eof(socket)
            if bytesavailable(socket) > 0
                measurement_msg = deserialize(socket)
                received = true
            else
                break
            end
        end
        !received && continue
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
                @info "Received data of type $(typeof(meas))"
            end
        end
    end
    
    # Monitor perception state for threats and respond accordingly
    @async begin
        while true
            if isready(perception_state_channel)
                #@info "ever here"
                perception_state = take!(perception_state_channel)
                for detected_object in perception_state.detected_objects
                    if detected_object.threat_level > 0.5  # Threshold for alert
                        @warn "High threat detected!" object=detected_object
                        if detected_object.threat_level > 0.8  # Threshold for autonomous action
                            @info "Taking evasive action!"
                            # Adjust vehicle commands based on the threat
                            target_velocity = max(target_velocity - v_step, 0)  # Slow down
                            # Add steering adjustment logic if necessary
                        end
                    end
                end
            end
            sleep(0.01)  # Check time
        end
    end
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    
    client_info_string = 
        "********************
      Keyboard Control (manual mode)
      ********************
        -Press 'q' at any time to terminate vehicle.
        -Press 'i' to increase vehicle speed.
        -Press 'k' to decrease vehicle speed.
        -Press 'j' to increase steering angle (turn left).
        -Press 'l' to decrease steering angle (turn right)."
    #@info client_info_string
    errormonitor(@async localize(gt_channel, localization_state_channel, ego_vehicle_id))
    #@async test_localization(gt_channel, localization_state_channel)

    @async perception(cam_channel, localization_state_channel, perception_state_channel)

    while controlled && isopen(socket)
        key = get_c()
        if key == 'q'
            # terminate vehicle
            controlled = false
            target_velocity = 0.0
            steering_angle = 0.0
            @info "Terminating Keyboard Client."
        elseif key == 'i'
            # increase target velocity
            target_velocity += v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'k'
            # decrease forward force
            target_velocity -= v_step
            @info "Target velocity: $target_velocity"
        elseif key == 'j'
            # increase steering angle
            steering_angle += s_step
            @info "Target steering angle: $steering_angle"
        elseif key == 'l'
            # decrease steering angle
            steering_angle -= s_step
            @info "Target steering angle: $steering_angle"
        end
        
        cmd = (steering_angle, target_velocity, controlled)
        serialize(socket, cmd)
    end
end