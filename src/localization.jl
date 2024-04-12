# Implement an EKF for vehicle localization
struct ExtendedKalmanFilter
    state::Vector{Float64} # Vehicle state: position, quaternion, velocity, angular velocity
    covariance::Matrix{Float64} # State covariance matrix
    process_noise::Matrix{Float64} # Process noise covariance
    # measurement_noise::Matrix{Float64} # Measurement noise covariance
    measurement_noise_gps::Matrix{Float64}
    measurement_noise_imu::Matrix{Float64}
    measurement_noise_cam::Matrix{Float64}
end

const L = 2.0 # Example value

# state transition function
function f_ackermann(x, u, Δt)
    # x: state vector [x, y, θ, v, ϕ] where
    # x, y: position, θ: orientation, v: velocity, ϕ: steering angle
    # u: control inputs [a, δ] where
    # a: acceleration, δ: change in steering angle

    θ = x[3]
    v = x[4]
    ϕ = x[5]
    a = u[1]
    δ = u[2]

    # Update steering angle
    ϕ += δ * Δt

    # Ackermann steering model equations
    new_x = x[1] + v * cos(θ) * Δt
    new_y = x[2] + v * sin(θ) * Δt
    new_θ = θ + (v / L) * tan(ϕ) * Δt  # L is the wheelbase length
    new_v = v + a * Δt

    return [new_x, new_y, new_θ, new_v, ϕ]
end

function h_extended(state)
    # Direct extraction of the relative position, velocity, and stop sign distance
    # from the state vector.
    dx = state[6]
    dy = state[7]
    dv = state[8]
    d_stop = state[9]

    # Here, the measurement model simply reflects the extended state components directly.
    # In a real system, this function would model how these state variables are observed
    # by the sensors (e.g., transforming relative positions from the vehicle's coordinate
    # frame to a global frame, or vice versa).
    
    return [dx, dy, dv, d_stop]
end

function Jac_hx_extended(state)
    # For simplicity, assuming direct measurement of extended states,
    # the Jacobian would look like this:
    J = zeros(4, length(state))
    
    # Derivatives of [dx, dy, dv, d_stop] with respect to themselves are 1,
    # indicating a direct measurement. All other derivatives are 0.
    J[1, 6] = 1
    J[2, 7] = 1
    J[3, 8] = 1
    J[4, 9] = 1
    
    return J
end

function h_imu(state)
    # Assuming state = [x, y, z, ωx, ωy, ωz, vx, vy, vz, ...] where
    # ωx, ωy, ωz: angular velocities around each axis
    # vx, vy, vz: velocities along each axis

    # Extract linear velocity components
    vx = state[7]
    vy = state[8]
    vz = state[9]

    # Extract angular velocity components
    ωx = state[4]
    ωy = state[5]
    ωz = state[6]

    # Return the 6-dimensional vector representing IMU measurements
    # [vx, vy, vz, ωx, ωy, ωz]
    return [vx, vy, vz, ωx, ωy, ωz]
end


function h_gps(state::Vector{Float64})
    # Assuming state = [x, y, θ, v, ϕ, ...]
    # And GPS measures x, y position
    return state[1:2]  # Return the predicted GPS measurement (x, y position)
end


function Jac_h_imu(state)
    # Assuming the state vector layout from h_imu function
    # The size of J is 6xN, where N is the number of elements in the state vector
    J = zeros(6, length(state))

    # Map the direct relationship of the state components to the IMU measurements
    # For linear velocities
    J[1, 7] = 1  # vx
    J[2, 8] = 1  # vy
    J[3, 9] = 1  # vz

    # For angular velocities
    J[4, 4] = 1  # ωx
    J[5, 5] = 1  # ωy
    J[6, 6] = 1  # ωz

    return J
end


function Jac_h_gps(state::Vector{Float64})
    # Create a matrix filled with zeros
    J = zeros(2, length(state))
    
    # The derivative of the GPS measurement (x, y) with respect to the position (x, y) is 1
    J[1, 1] = 1  # Partial derivative of x position measurement with respect to x state
    J[2, 2] = 1  # Partial derivative of y position measurement with respect to y state
    
    return J
end


function process_camera_measurement(measurement::CameraMeasurement)
    # Simplified processing; in reality, would involve more complex calculations based on camera model
    distances = [sqrt((bbox[3] - bbox[1])^2 + (bbox[4] - bbox[2])^2) for bbox in measurement.bounding_boxes]
    avg_distance = mean(distances)  # Example processing; real implementation depends on your application
    return avg_distance
end

function behavior_controller(state, controls)
    # Determine behavior based on state
    if approaching_stop_sign(state)
        controls = adjust_for_stop_sign(state, controls)
    elseif following_vehicle(state)
        controls = adjust_for_following_distance(state, controls)
    else
        controls = maintain_cruising_speed(state, controls)
    end
    return controls
end

function ekf_initialize()
    # Initial state vector dimension is 13 as per your setup.
    state = zeros(13) # For [x, y, z, θ, ωx, ωy, ωz, vx, vy, vz, ...]
    covariance = diagm(0 => ones(13))
    process_noise = diagm(0 => 0.1 .* ones(13))
    
    # Adjust measurement noise matrices to reflect the correct dimensions for each sensor.
    measurement_noise_gps = diagm(0 => [0.1, 0.1])  # GPS measures x, y, so it's 2x2.
    measurement_noise_imu = diagm(0 => [0.1, 0.1, 0.1, 0.1, 0.1, 0.1])  # IMU is 6-dimensional.
    measurement_noise_cam = diagm(0 => [0.1, 0.1, 0.01])  # Assuming Camera measurements, adjust as needed.
    
    # No general measurement_noise needed unless for a generic update, so it's either removed or specific to a case.
    # ExtendedKalmanFilter(state, covariance, process_noise, measurement_noise_gps, measurement_noise_gps, measurement_noise_imu, measurement_noise_cam)
    ExtendedKalmanFilter(state, covariance, process_noise, measurement_noise_gps, measurement_noise_imu, measurement_noise_cam)
end


function ekf_predict(ekf::ExtendedKalmanFilter, u::Vector{Float64}, Δt::Float64)
    new_state = f_ackermann(ekf.state, u, Δt)
    Fx = Jac_x_f(new_state, Δt) # You'll need to implement or adjust Jac_x_f accordingly
    new_covariance = Fx * ekf.covariance * Fx' + ekf.process_noise
    return ExtendedKalmanFilter(new_state, new_covariance, ekf.process_noise, ekf.measurement_noise_gps, ekf.measurement_noise_imu, ekf.measurement_noise_cam)
end

function ekf_update!(ekf::ExtendedKalmanFilter, measurement)
    if isa(measurement, GPSMeasurement)
        H_gps = Jac_h_gps(ekf.state)
        z_pred_gps = h_gps(ekf.state)
        gps_measurement = [measurement.lat, measurement.long]  # Assuming lat and long are properties

        @info H_gps
        @info z_pred_gps
        @info z_gps
        @info ekf.measurement_noise_gps
        
        # Perform the update calculations
        Y = gps_measurement - z_pred_gps
        S = H_gps * ekf.covariance * H_gps' + ekf.measurement_noise_gps
        K = ekf.covariance * H_gps' / S
        new_state = ekf.state + K * Y
        new_covariance = (I - K * H_gps) * ekf.covariance
        
        return ExtendedKalmanFilter(new_state, new_covariance, ekf.process_noise, ekf.measurement_noise_gps, ekf.measurement_noise_imu, ekf.measurement_noise_cam)
    elseif isa(measurement, IMUMeasurement)
        
        # IMU update logic
        H_imu = Jac_h_imu(ekf.state)  # Jacobian of the IMU measurement function
        z_pred_imu = h_imu(ekf.state)  # Predicted IMU measurement
        z_imu = [measurement.linear_vel; measurement.angular_vel]  # Actual IMU measurement

        @info H_imu
        @info z_pred_imu
        @info z_imu
        @info ekf.measurement_noise_imu

        # Instead of update_ekf!, directly perform the update calculations here
        Y = z_imu - z_pred_imu  # Measurement residual
        S = H_imu * ekf.covariance * H_imu' + ekf.measurement_noise_imu  # Residual covariance
        K = ekf.covariance * H_imu' / S  # Kalman gain
        new_state = ekf.state + K * Y  # State update
        new_covariance = (I - K * H_imu) * ekf.covariance  # Covariance update

        return ExtendedKalmanFilter(new_state, new_covariance, ekf.process_noise, ekf.measurement_noise_gps, ekf.measurement_noise_imu, ekf.measurement_noise_cam)
    elseif isa(measurement, CameraMeasurement)
        # Camera update logic
        # Again, hypothetical function calls
        # H_camera = Jac_h_camera(ekf.state) # Jacobian of the camera measurement function
        # z_pred_camera = h_camera(ekf.state) # Predicted camera measurement
        # # Camera measurement conversion is more complex, likely involving bounding box processing
        # z_camera = process_camera_measurement(measurement) # You need to implement this function
        # update_ekf!(ekf, H_camera, z_camera, z_pred_camera, ekf.measurement_noise_camera)
    elseif isa(measurement, GroundTruthMeasurement)
        # Ground truth update logic
        # Ground truth can be used for validation or as a direct state update in some cases
        # Direct state update from ground truth is not typical for EKF but shown here for completeness
        # ekf.state = [measurement.position; measurement.orientation; measurement.velocity; measurement.angular_velocity]
        # You might not directly use ground truth this way in a real system, but rather for validation
    end
end

# Generalized EKF update function to reduce repetition
function update_ekf!(ekf::ExtendedKalmanFilter, H, z, z_pred, measurement_noise)
    Y = z - z_pred # Measurement residual
    S = H * ekf.covariance * H' + measurement_noise # Residual covariance
    K = ekf.covariance * H' / S # Kalman gain
    ekf.state += K * Y # State update
    ekf.covariance = (I - K * H) * ekf.covariance # Covariance update
end

# Main localization loop that utilizes the EKF with sensor data
function run_localization_loop()
    ekf = ekf_initialize()
    while true
        Δt = 10 # Determine timestep based on system requirements
        ekf_predict!(ekf, Δt)
        # Fetch sensor measurements
        for measurement in # Sensor measurement fetching logic
            ekf_update!(ekf, measurement)
        end
        # Here, ekf.state contains the updated vehicle state
        # This can be used for vehicle control, navigation, etc.
    end
end