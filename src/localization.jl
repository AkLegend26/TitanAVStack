# Assume inclusion of Measurement structs and related functions as provided

# making sure it runs
# test code for localization code, comparing ground truth measurements with my estimates, print diagnosis logs
# drive around with keyboard client, localization error stays error

abstract type Measurement end

"""
Records lat/long measurements (expressed as first and second coordinates of map frame)
of GPS sensor.
"""
struct GPSMeasurement <: Measurement
    time::Float64
    lat::Float64
    long::Float64
    heading::Float64
end

"""
Records linear and angular velocity experienced instantaneously by IMU sensor,
in local frame of IMU.
"""
struct IMUMeasurement <: Measurement
    time::Float64
    linear_vel::SVector{3, Float64}
    angular_vel::SVector{3, Float64}
end

"""
Records list of bounding boxes represented by top-left and bottom-right pixel coordinates.
Camera_id is 1 or 2, indicating which camera produced the bounding boxes.

Pixels start from (1,1) in top-left corner of image, and proceed right (first axis) and down (second axis)
to image_width and image_height pixels.

Each pixel corresponds to a pixel_len x pixel_len patch at focal_len away from a pinhole model.
"""
struct CameraMeasurement <: Measurement
    time::Float64
    camera_id::Int
    focal_length::Float64
    pixel_length::Float64
    image_width::Int # pixels
    image_height::Int # pixels
    bounding_boxes::Vector{SVector{4, Int}}
end

"""
Records position, orientation, linear and angular velocity, and size of 3D bounding box of vehicle
specified by vehicle ID
"""
struct GroundTruthMeasurement <: Measurement
    time::Float64
    vehicle_id::Int
    position::SVector{3, Float64} # position of center of vehicle
    orientation::SVector{4, Float64} # represented as quaternion
    velocity::SVector{3, Float64}
    angular_velocity::SVector{3, Float64} # angular velocity around x,y,z axes
    size::SVector{3, Float64} # length, width, height of 3d bounding box centered at (position/orientation)
end

"""
Vehicle ID is the id of the vehicle receiving this message.
target segment is the map segment that should be driven to.
mesaurements are the latest sensor measurements.
"""
struct MeasurementMessage
    vehicle_id::Int
    target_segment::Int
    measurements::Vector{Measurement}
end

# Implement an EKF for vehicle localization
struct ExtendedKalmanFilter
    state::Vector{Float64} # Vehicle state: position, quaternion, velocity, angular velocity
    covariance::Matrix{Float64} # State covariance matrix
    process_noise::Matrix{Float64} # Process noise covariance
    measurement_noise::Matrix{Float64} # Measurement noise covariance
end

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
    # Assuming state = [x, y, θ, v, ϕ, dx, dy, dv, d_stop], extract v and a representation of angular velocity
    v = state[4]  # Linear velocity
    ϕ = state[5]  # Steering angle, used to infer angular velocity
    return [v; ϕ]  # Simplified IMU measurement model; adjust ϕ as per your system's modeling
end

function Jac_h_imu(state)
    J = zeros(2, length(state))
    J[1, 4] = 1  # Partial derivative of v with respect to itself
    J[2, 5] = 1  # Assuming direct influence of ϕ on the angular velocity component
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
    # Initial state and covariance matrix setup
    state = zeros(13) # Example initialization
    covariance = diagm(0 => ones(13))
    process_noise = diagm(0 => 0.1 .* ones(13))
    measurement_noise = diagm(0 => [0.1, 0.1, 0.01]) # GPS measurement noise as an example
    ExtendedKalmanFilter(state, covariance, process_noise, measurement_noise)
end

function ekf_predict!(ekf::ExtendedKalmanFilter, Δt::Float64)
    # Use the rigid_body_dynamics as the process model
    ekf.state = f(ekf.state, Δt)
    Fx = Jac_x_f(ekf.state, Δt) # Jacobian of the process model
    ekf.covariance = Fx * ekf.covariance * Fx' + ekf.process_noise
end

function ekf_update!(ekf::ExtendedKalmanFilter, measurement)
    if isa(measurement, GPSMeasurement)
        # GPS update logic as before
    elseif isa(measurement, IMUMeasurement)
        # IMU update logic
        # Hypothetical function calls (you need to implement these based on your system's specifics)
        H_imu = Jac_h_imu(ekf.state) # Jacobian of the IMU measurement function
        z_pred_imu = h_imu(ekf.state) # Predicted IMU measurement
        z_imu = [measurement.linear_vel; measurement.angular_vel] # Actual IMU measurement
        update_ekf!(ekf, H_imu, z_imu, z_pred_imu, ekf.measurement_noise_imu) # Use a generalized update function
    elseif isa(measurement, CameraMeasurement)
        # Camera update logic
        # Again, hypothetical function calls
        H_camera = Jac_h_camera(ekf.state) # Jacobian of the camera measurement function
        z_pred_camera = h_camera(ekf.state) # Predicted camera measurement
        # Camera measurement conversion is more complex, likely involving bounding box processing
        z_camera = process_camera_measurement(measurement) # You need to implement this function
        update_ekf!(ekf, H_camera, z_camera, z_pred_camera, ekf.measurement_noise_camera)
    elseif isa(measurement, GroundTruthMeasurement)
        # Ground truth update logic
        # Ground truth can be used for validation or as a direct state update in some cases
        # Direct state update from ground truth is not typical for EKF but shown here for completeness
        ekf.state = [measurement.position; measurement.orientation; measurement.velocity; measurement.angular_velocity]
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
