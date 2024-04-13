struct MyLocalizationType
    position::Vector{Float64}  # [x, y]
    orientation::Float64       # θ
    velocity::Vector{Float64}  # [vx, vy]
end


struct MyPerceptionType
    field1::Int
    field2::Float64
end

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

                # Extract necessary components from ground truth data
                position_xy = [gt_data.position[1], gt_data.position[2]]  # Get the x, y components of position
                velocity_xy = [gt_data.velocity[1], gt_data.velocity[2]]  # Get the x, y components of velocity
                
                # Assuming orientation needs to be converted from quaternion to yaw angle
                # Here's a simple method to convert quaternion to yaw assuming quaternion is [w, x, y, z]
                # This conversion is simplified and specific to this scenario
                w, x, y, z = gt_data.orientation
                yaw = atan(2.0 * (w * z + x * y), 1.0 - 2.0 * (y^2 + z^2))

                localized_state = MyLocalizationType(
                    position_xy,   # Ground truth position [x, y]
                    yaw,           # Computed yaw from quaternion
                    velocity_xy    # Ground truth velocity [vx, vy]
                )

                # Ensure the localization state channel is not full before putting new data
                if isready(localization_state_channel)
                    take!(localization_state_channel)  # Clear the channel if full
                end

                put!(localization_state_channel, localized_state)
                @info "Published ground truth localization state."
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
    # set up stuff
    while true
        fresh_cam_meas = []
        while isready(cam_meas_channel)
            meas = take!(cam_meas_channel)
            push!(fresh_cam_meas, meas)
        end

        latest_localization_state = fetch(localization_state_channel)
        
        # process bounding boxes / run ekf / do what you think is good

        perception_state = MyPerceptionType(0,0.0)
        if isready(perception_state_channel)
            take!(perception_state_channel)
        end
        put!(perception_state_channel, perception_state)
    end
end

function decision_making(localization_state_channel, map_segments, socket, target_road_segment_id)
    @info "Decision making task started..."
    while true
        try
            if isready(localization_state_channel)
                latest_localization_state = take!(localization_state_channel)
                @info "Finding current segment..."
                current_segment_id = find_current_segment(latest_localization_state.position, map_segments)
                if current_segment_id == -1
                    @error "No segment found for position" position=latest_localization_state.position
                    continue  # Skip to the next iteration if no segment is found
                end
                @info "Current segment found" segment_id=current_segment_id
                @info "Calculating shortest path..."
                path = shortest_path(current_segment_id, target_road_segment_id, map_segments)
                @info "Navigating through path...", length(path)
                for segment_id in path
                    segment = map_segments[segment_id]
                    navigate_segment(segment, latest_localization_state, socket)
                end
            else
                sleep(0.1)  # Sleep to prevent tight loop if channel is empty
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
    socket = Sockets.connect(host, port)
    map_segments = VehicleSim.training_map()
    
    msg = deserialize(socket) # Visualization info
    @info msg

    gps_channel = Channel{GPSMeasurement}(32)
    imu_channel = Channel{IMUMeasurement}(32)
    cam_channel = Channel{CameraMeasurement}(32)
    gt_channel = Channel{GroundTruthMeasurement}(32)

    localization_state_channel = Channel{MyLocalizationType}(1)
    perception_state_channel = Channel{MyPerceptionType}(1)

    target_map_segment = 0 # (not a valid segment, will be overwritten by message)
    ego_vehicle_id = 0 # (not a valid id, will be overwritten by message. This is used for discerning ground-truth messages)

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
                @info typeof(meas)
            end
        end
    end)
    targets = 51
    
    @async localize(gt_channel, localization_state_channel)
    #@async localize(gps_channel, imu_channel, localization_state_channel)
    #@async test_localization(gt_channel, localization_state_channel)

    #@async perception(cam_channel, localization_state_channel, perception_state_channel)
    @async decision_making(localization_state_channel, map_segments, socket, targets)
end
