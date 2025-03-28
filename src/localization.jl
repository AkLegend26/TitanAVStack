mutable struct ExtendedKalmanFilter
    state::Vector{Float64}                # Expanded state vector 
    covariance::Matrix{Float64}           # Covariance matrix 
    process_noise::Matrix{Float64}        # Process noise covariance matrix
    measurement_noise_gps::Matrix{Float64}  # Measurement noise matrix for GPS
    measurement_noise_imu::Matrix{Float64}  # Measurement noise matrix for IMU
end

function J_R_q(q)
    qw = q[1]
    qx = q[2]
    qy = q[3]
    qz = q[4]

    dRdq1 = 2*[qw -qz qy;
             qz qw -qx;
             -qy qx qw]
    dRdq2 = 2*[qx qy qz;
               qy -qx -qw;
               qz qw -qx]
    dRdq3 = 2*[-qy qx qw;
               qx qy qz;
               -qw qz -qy]
    dRdq4 = 2*[-qz -qw qx;
               qw -qz qy;
               qx qy qz]
    (dRdq1, dRdq2, dRdq3, dRdq4)
end

function J_Tbody(x)
    J_Tbody_xyz = (zeros(3,4), zeros(3,4), zeros(3,4))
    for i = 1:3
       J_Tbody_xyz[i][i,4] = 1.0
    end
    return (J_Tbody_xyz, [[dR zeros(3)] for dR in J_R_q(x[4:7])])
end

function f_ackermann(x, Δt)
    θ = x[3]
    v = x[4]
    ω = x[11]  # Assuming ωx represents angular velocity about the vertical axis
    if ω == 0
        new_x = x[1] + v * cos(θ) * Δt
        new_y = x[2] + v * sin(θ) * Δt
    else
        new_x = x[1] + (v/ω) * (sin(θ + ω*Δt) - sin(θ))
        new_y = x[2] + (v/ω) * (-cos(θ + ω*Δt) + cos(θ))
    end
    new_θ = θ + ω * Δt
    return [new_x, new_y, new_θ, v]
end

function extract_yaw_from_quaternion(q)
    atan(2(q[1]*q[4]+q[2]*q[3]), 1-2*(q[3]^2+q[4]^2))
end

function quaternion_multiply(q1, q2)
    a, b, c, d = q1
    e, f, g, h = q2

    return [
        a*e - b*f - c*g - d*h,
        a*f + b*e + c*h - d*g,
        a*g - b*h + c*e + d*f,
        a*h + b*g - c*f + d*e
    ]
end

function quaternion_derivative(q, omega)
    omega_quat = [0; omega]  # Convert angular velocity vector to quaternion
    return 0.5 * quaternion_multiply(q, omega_quat)
end


function Rot_from_quat(q)
    qw = q[1]
    qx = q[2]
    qy = q[3]
    qz = q[4]

    R = [qw^2+qx^2-qy^2-qz^2 2(qx*qy-qw*qz) 2(qw*qy+qx*qz);
         2(qx*qy+qw*qz) qw^2-qx^2+qy^2-qz^2 2(qy*qz-qw*qx);
         2(qx*qz-qw*qy) 2(qw*qx+qy*qz) qw^2-qx^2-qy^2+qz^2]
end

function get_gps_transform()
    R_gps_to_body = one(RotMatrix{3, Float64})
    t_gps_to_body = [-3.0, 1, 2.6]
    T = [R_gps_to_body t_gps_to_body]
end

function get_body_transform(quat, loc)
    R = Rot_from_quat(quat)
    [R loc]
end

function h_gps(x)
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    xyz_body = x[1:3] # position
    q_body = x[4:7] # quaternion
    Tbody = get_body_transform(q_body, xyz_body)
    xyz_gps = Tbody * [gps_loc_body; 1]
    yaw = extract_yaw_from_quaternion(q_body)
    meas = [xyz_gps[1:2]; yaw]
end

function jac_fx(x, Δt)
    # Jacobian of the Ackermann steering model
    θ = x[3]
    v = x[4]
    J = zeros(length(x), length(x))
    J[1, 3] = -v * sin(θ) * Δt
    J[1, 4] = cos(θ) * Δt
    J[2, 3] = v * cos(θ) * Δt
    J[2, 4] = sin(θ) * Δt
    return J
end

function Jac_h_gps(x)
    T = get_gps_transform()
    gps_loc_body = T*[zeros(3); 1.0]
    xyz_body = x[1:3] # position
    q_body = x[4:7] # quaternion
    Tbody = get_body_transform(q_body, xyz_body)
    xyz_gps = Tbody * [gps_loc_body; 1]
    yaw = extract_yaw_from_quaternion(q_body)
    #meas = [xyz_gps[1:2]; yaw]
    J = zeros(3, 13)
    (J_Tbody_xyz, J_Tbody_q) = J_Tbody(x)
    for i = 1:3
        J[1:2,i] = (J_Tbody_xyz[i]*[gps_loc_body; 1])[1:2]
    end
    for i = 1:4
	J[1:2,3+i] = (J_Tbody_q[i]*[gps_loc_body; 1])[1:2]
    end
    w = q_body[1]
    x = q_body[2]
    y = q_body[3]
    z = q_body[4]
    J[3,4] = -(2 * z * (-1 + 2 * (y^2 + z^2)))/(4 * (x * y + w * z)^2 + (1 - 2 * (y^2 + z^2))^2)
    J[3,5] = -(2 * y * (-1 + 2 * (y^2 + z^2)))/(4 * (x * y + w * z)^2 + (1 - 2 * (y^2 + z^2))^2)
    J[3,6] = (2 * (x + 2 * x * y^2 + 4 * w * y * z - 2 * x * z^2))/(1 + 4 * y^4 + 8 * w * x * y * z + 4 * (-1 + w^2) * z^2 + 4 * z^4 + 4 * y^2 * (-1 + x^2 + 2 * z^2))
    J[3,7] = (2 * (w - 2 * w * y^2 + 4 * x * y * z + 2 * w * z^2))/(1 + 4 * y^4 + 8 * w * x * y * z + 4 * (-1 + w^2) * z^2 + 4 * z^4 + 4 * y^2 * (-1 + x^2 + 2 * z^2))
    J
end

function h_imu(state::Vector{Float64})
    # state vector layout is:
    # [x, y, z, qx, qy, qz, qw, vx, vy, vz, ωx, ωy, ωz]

    # Extracting linear and angular velocities
    linear_velocity = state[8:10]  # vx, vy, vz
    angular_velocity = state[11:13]  # ωx, ωy, ωz

    # Returning measurement vector
    return [linear_velocity; angular_velocity]
end


function jac_h_imu(state::Vector{Float64})
    # IMU measures 3 linear velocities and 3 angular velocities
    J = zeros(6, length(state))

    # The IMU measures linear velocity (vx, vy, vz) directly:
    J[1, 8] = 1  # derivative of vx measurement with respect to vx state
    J[2, 9] = 1  # derivative of vy measurement with respect to vy state
    J[3, 10] = 1 # derivative of vz measurement with respect to vz state

    # The IMU measures angular velocity (ωx, ωy, ωz) directly:
    J[4, 11] = 1  # derivative of ωx measurement with respect to ωx state
    J[5, 12] = 1  # derivative of ωy measurement with respect to ωy state
    J[6, 13] = 1  # derivative of ωz measurement with respect to ωz state

    return J
end

function normalize_quaternion(q)
    norm = sqrt(dot(q, q))
    if norm > 0
        return q / norm
    else
        return q  # Zero norm case
    end
end

function ekf_predict(ekf::ExtendedKalmanFilter, Δt::Float64)
    new_state = f_ackermann(ekf.state, Δt)
    Fx = jac_fx(new_state, Δt)
    new_covariance = Fx * ekf.covariance * Fx' + ekf.process_noise
    return ExtendedKalmanFilter(new_state, new_covariance, ekf.process_noise, ekf.measurement_noise_gps, ekf.measurement_noise_imu)
end

function predict_quaternion(q, omega, dt)
    # Computing the quaternion derivative
    q_dot = quaternion_derivative(q, omega)
    # New quaternion step
    q_new = q + q_dot * dt
    # Normalizing quaternion
    return normalize_quaternion(q_new)
end


function ekf_update!(ekf::ExtendedKalmanFilter, measurement, measurement_function, jac_hx, measurement_noise)
    z_pred = measurement_function(ekf.state)
    z = measurement
    H = jac_hx(ekf.state)
    Y = z - z_pred
    S = H * ekf.covariance * H' + measurement_noise + 1e-5 * I
    Δt = 0.1
    K = ekf.covariance * H' / S
    ekf.state += K * Y
    ekf.state[4:7] = predict_quaternion(ekf.state[4:7], ekf.state[11:13], Δt)  # Normalize quaternion part of the state

    ekf.covariance = (I - K * H) * ekf.covariance
end

function ekf_initialize()
    state = zeros(13)  
    state[4:7] = [1.0, 0.0, 0.0, 0.0]  # Initialize the quaternion

    covariance = diagm(0 => 0.1 * ones(13))
    # Uncertainty for the quaternion
    covariance[4:7, 4:7] *= 1  # increase the initial uncertainty of the quaternion

    process_noise = diagm(0 => 0.1 * ones(13))
    measurement_noise_gps = diagm(0 => [0.1, 0.1, 0.1])  # 
    measurement_noise_imu = diagm(0 => 0.1 * ones(6))  #

    ExtendedKalmanFilter(state, covariance, process_noise, measurement_noise_gps, measurement_noise_imu)
end


function run_localization_loop(ekf)
    Δt = 0.1  # time step
    while true
        ekf = ekf_predict(ekf, Δt)
        gps_measurement = [get_gps_x(), get_gps_y()]  # Fetch GPS data
        ekf_update!(ekf, gps_measurement, h_gps, Jac_h_gps, ekf.measurement_noise_gps)
    end
end
