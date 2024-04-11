struct VehicleCommand
    steering_angle::Float64
    velocity::Float64
    controlled::Bool
end

function get_c()
    c = 'x'
    try
        ret = ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, true)
        ret == 0 || error("unable to switch to raw mode")
        c = read(stdin, Char)
        ccall(:jl_tty_set_mode, Int32, (Ptr{Cvoid},Int32), stdin.handle, false)
    catch e
    end
    c
end

function async_deserialize(socket::TCPSocket, timeout_seconds::Float64 = 5.0)
    result_channel = Channel{Any}(1)

    @async begin
        try
            timer = Timer(timeout_seconds) do _
                put!(result_channel, :timeout)  # Signal a timeout occurred
            end

            data = deserialize(socket)  # Attempt to deserialize data from the socket
            istaskdone(current_task()) || put!(result_channel, data)  # Put the result into the channel if not timed out

            close(timer)  # Cancel the timer if deserialization succeeds before the timeout
        catch e
            istaskdone(current_task()) || put!(result_channel, e)  # In case of an error, put the error in the channel if not timed out
        end
    end

    return result_channel
end

function wait_for_deserialize(channel::Channel, timeout::Float64)
    # Start a task to take from the channel
    take_task = @async take!(channel)
    
    # Wait for the task to complete or for the timeout
    try
        wait(take_task; timeout=timeout)
        return fetch(take_task)  # Retrieve the result if available
    catch e
        if isa(e, TaskFailedException) && isa(e.cause, TimeoutException)
            @warn "Timeout occurred while waiting for data."
            return :timeout  # Indicate a timeout occurred
        else
            @error "Unexpected error: $e"
            return e  # Return the error for further handling
        end
    end
end

function keyboard_client(host::IPAddr=IPv4(0), port=4444; v_step = 1.0, s_step = π/10)
    @info "ADAM"
    
    gps_channel = Channel{GPSMeasurement}(32)
    @info "gps received"
    imu_channel = Channel{IMUMeasurement}(32)
    @info "imu received"
    cam_channel = Channel{CameraMeasurement}(32)
    @info "cam received"
    gt_channel = Channel{GroundTruthMeasurement}(32)
    @info "gt received"
    
    localization_state_channel = Channel{MyLocalizationType}(1)
    @info "local received"
    perception_state_channel = Channel{MyPerceptionType}(1)
    @info "perception received"

    socket = Sockets.connect(host, port)
    @info "socket received"
    (peer_host, peer_port) = getpeername(socket)
    @info "peer received"
    # @async msg = deserialize(socket) # Visualization info
    # while isopen(socket) && bytesavailable(socket) > 0
    #     msg = deserialize(socket)
    #     # Process the message
    # end

    msg = async_deserialize(socket)
    if msg === nothing
        @error "Failed to receive initial message from the server within the timeout period."
        return  # or handle the error appropriately
    end

    @info "socket deserialized"
    @info msg
    ekf1 = ekf_initialize()

    while isopen(socket)
        sleep(0.001)
        state_msg_channel = async_deserialize(socket)
        try
        # Now, use this function instead of directly calling take!(state_msg_channel)
        timeout_seconds = 5.0  # Define your timeout duration
        state_msg_result = wait_for_deserialize(state_msg_channel, timeout_seconds)

        if state_msg_result === :timeout
            @error "Deserialization timed out."
            # Handle timeout (e.g., retry, abort, etc.)
        elseif isa(state_msg_result, Exception)
            @error "Error during deserialization: $state_msg_result"
            # Handle error
        else
            @info "Message received."
            # Process the successfully deserialized message here
            measurements = state_msg_result.measurements
            num_cam = 0
            num_imu = 0
            num_gps = 0
            num_gt = 0
            for meas in measurements
                if meas isa GPSMeasurement
                    !isfull(gps_channel) && put!(gps_channel, meas)
                elseif meas isa IMUMeasurement
                    @info "in imu"
                    !isfull(imu_channel) && put!(imu_channel, meas)
                    ekf_update!(ekf1, meas)
                #elseif meas isa CameraMeasurement
                #   @info "in cam"
                #  !isfull(cam_channel) && put!(cam_channel, meas)
                elseif meas isa GroundTruthMeasurement
                    @info "in gt"
                    !isfull(gt_channel) && put!(gt_channel, meas)
                end
            end
        end
                # if meas isa GroundTruthMeasurement
                #      # Ground truth - used for error measurement
                #     gt_position = meas.position[1:2]  # Assuming 2D for simplicity
                #     gt_orientation = meas.orientation
                # else
                #     # Update EKF with IMU, GPS, and Camera measurements
                #     ekf_update!(ekf, meas)
                # end

            # estimated_position = ekf.state[1:2]  # Assuming state vector format matches EKF setup
            # position_error = norm(estimated_position - gt_position)
            
            # @info "Position Error: $position_error"
            #@info "Measurements received: $num_gt gt; $num_cam cam; $num_imu imu; $num_gps gps"
        catch e
            @error "Failed to take from result_channel: $e"
        end
    
    target_velocity = 0.0
    steering_angle = 0.0
    controlled = true
    
    client_info_string = 
        "********************
      Titans Reving
      ********************"
    @info client_info_string
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
end
        
            

function example_client(host::IPAddr=IPv4(0), port=4444)
    socket = Sockets.connect(host, port)
    map_segments = training_map()
    (; chevy_base) = load_mechanism()

    @async while isopen(socket)
        state_msg = deserialize(socket)
    end
   
    shutdown = false
    persist = true
    while isopen(socket)
        position = state_msg.q[5:7]
        @info position
        if norm(position) >= 100
            shutdown = true
            persist = false
        end
        cmd = (0.0, 2.5, persist, shutdown)
        serialize(socket, cmd) 
    end

end