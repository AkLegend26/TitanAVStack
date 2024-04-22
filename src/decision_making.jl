function find_current_segment(position, map_segments)
    @info "Finding segment for position: $position"
    best_segment_id = -1
    closest_distance = Inf

    function check_segments(segments, position)
        local_best_id = -1
        local_closest_dist = Inf

        for (id, segment) in segments
            within, distance = check_segment(position, segment)
            if within && distance < local_closest_dist
                local_closest_dist = distance
                local_best_id = id
            end
        end

        return local_best_id, local_closest_dist
    end

    best_segment_id, closest_distance = check_segments(map_segments, position)

    # If no suitable segment is found, check the children of the segments
    if best_segment_id == -1
        for (id, segment) in map_segments
            if !isempty(segment.children)
                for child_id in segment.children
                    child_segment = map_segments[child_id]
                    within, distance = check_segment(position, child_segment)
                    if within && distance < closest_distance
                        closest_distance = distance
                        best_segment_id = child_id
                    end
                end
            end
        end
    end

    if best_segment_id != -1
        @info "Segment found: $best_segment_id"
        return best_segment_id
    else
        #@error "No current segment found for position: $position"
        return -1
    end
end

function check_segment(position, segment)
    #@info "Checking segment: $(segment.id)" position=position segment_boundaries=segment.lane_boundaries
    if segment.lane_boundaries[1].curvature != 0
        return check_curved_segment(position, segment)
    else
        return check_straight_segment(position, segment.lane_boundaries)
    end
end

function check_straight_segment(position, boundaries)
    min_x = Inf
    max_x = -Inf
    min_y = Inf
    max_y = -Inf

    for boundary in boundaries
        min_x = min(min_x, boundary.pt_a[1], boundary.pt_b[1])
        max_x = max(max_x, boundary.pt_a[1], boundary.pt_b[1])
        min_y = min(min_y, boundary.pt_a[2], boundary.pt_b[2])
        max_y = max(max_y, boundary.pt_a[2], boundary.pt_b[2])
    end

    within = (min_x <= position[1]) && (position[1] <= max_x) && (min_y <= position[2]) && (position[2] <= max_y)
    distance = if within
        0
    else
        minimum([abs(position[1] - min_x), abs(position[1] - max_x), abs(position[2] - min_y), abs(position[2] - max_y)])
    end

    return within, distance
end

function check_curved_segment(position, segment)
    boundary = segment.lane_boundaries[1]
    center = calculate_curve_center(boundary)
    radius = calculate_curve_radius(boundary)
    pt_a = boundary.pt_a
    pt_b = boundary.pt_b
    angle = atan(position[2] - center[2], position[1] - center[1])
    distance = norm(position - center)

    within_angle = is_within_angle(pt_a, pt_b, center, angle)
    within_radius = radius * 0.9 <= distance <= radius * 1.1

    return within_angle && within_radius, distance - radius
end

function calculate_curve_center(boundary)
    pt_a, pt_b = boundary.pt_a, boundary.pt_b
    midpoint = (pt_a + pt_b) / 2
    vec = pt_b - pt_a
    perpendicular = [vec[2], -vec[1]]  # Rotate vector 90 degrees
    
    # Determine the direction based on the sign of curvature
    direction = sign(boundary.curvature)
    radius = abs(1.0 / boundary.curvature)
    
    # Center is `radius` distance away from midpoint in the direction of the perpendicular vector
    if direction != 0
        norm_perpendicular = normalize(perpendicular)
        center = midpoint + norm_perpendicular * radius * direction
        return center
    else
        return midpoint  # This should not happen for curved roads
    end
end

function calculate_curve_radius(boundary)
    return abs(1.0 / boundary.curvature)
end

function is_within_angle(pt_a, pt_b, center, angle)
    angle_a = atan(pt_a[2] - center[2], pt_a[1] - center[1])
    angle_b = atan(pt_b[2] - center[2], pt_b[1] - center[1])

    # Normalize angles
    angle = (angle + 2π) % (2π)
    angle_a = (angle_a + 2π) % (2π)
    angle_b = (angle_b + 2π) % (2π)

    # Ensure angle_a is less than angle_b
    if angle_a > angle_b
        angle_a, angle_b = angle_b, angle_a
    end

    # Check if angle is between angle_a and angle_b
    angle_a <= angle && angle <= angle_b
end


function point_to_segment_distance(point, segment)
    a, b = segment.pt_a, segment.pt_b
    ab = b - a
    ap = point - a
    t = clamp(dot(ap, ab) / dot(ab, ab), 0.0, 1.0)
    closest = a + t * ab
    distance = norm(point - closest)
    return distance
end

function heuristic(a, b, map_segments)
    ax, ay = calculate_segment_center(map_segments[a])
    bx, by = calculate_segment_center(map_segments[b])
    return sqrt((bx - ax)^2 + (by - ay)^2)
end


function shortest_path(start_id, target_id, map_segments)
    @info "Starting shortest path calculation"
    open_set = PriorityQueue()
    enqueue!(open_set, start_id, 0)

    came_from = Dict{Int, Int}()
    g_score = Dict{Int, Float64}(start_id => 0)
    f_score = Dict{Int, Float64}(start_id => heuristic(start_id, target_id, map_segments))

    while !isempty(open_set)
        current = dequeue!(open_set)

        if current == target_id
            return reconstruct_path(came_from, current)
        end

        for neighbor in map_segments[current].children
            tentative_g_score = g_score[current] + distance(current, neighbor, map_segments)
            if tentative_g_score < get(g_score, neighbor, Inf)
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(neighbor, target_id, map_segments)
                if !haskey(open_set, neighbor)
                    enqueue!(open_set, neighbor, f_score[neighbor])
                end
            end
        end
    end

    @info "No path found"
    return []
end

function reconstruct_path(came_from, current)
    path = []
    while haskey(came_from, current)
        pushfirst!(path, current)
        current = came_from[current]
    end
    pushfirst!(path, current)
    return path
end

function distance(current, neighbor, map_segments)
    (current_x, current_y) = calculate_segment_center(map_segments[current])
    (neighbor_x, neighbor_y) = calculate_segment_center(map_segments[neighbor])
    return sqrt((neighbor_x - current_x)^2 + (neighbor_y - current_y)^2)
end


function calculate_segment_center(segment)
    # Calculate the centroid of the segment based on its boundary points
    x_sum = y_sum = 0
    count = 0  # Total number of points
    
    for boundary in segment.lane_boundaries
        # Adding both points of the boundary
        x_sum += boundary.pt_a[1] + boundary.pt_b[1]
        y_sum += boundary.pt_a[2] + boundary.pt_b[2]
        count += 2
    end
    
    # Calculate the average of all x and y coordinates
    centroid_x = x_sum / count
    centroid_y = y_sum / count
    
    return (centroid_x, centroid_y)
end

function adjust_position_based_on_yaw(position, yaw)
    if length(position) < 2
        @error "Invalid position length"
        return position
    end
    pos = SVector{2, Float64}(position[1], position[2])
    adjusted_position = pos + SVector{2, Float64}(7*cos(yaw), 7*sin(yaw))
    #@info "Adjusted position based on yaw" original_position=pos adjusted_position=adjusted_position yaw=yaw
    return adjusted_position
end

# Function to send commands to the vehicle
function send_commands(steering_angle, velocity, socket)
    cmd = (steering_angle, velocity, true)
    serialize(socket, cmd)
    @info "command sent"
end

function is_at_segment_end(state, segment)
    # Extract position and ensure it's in a compatible format
    position = state.position[1:2]  # Assuming the position is an SVector

    # Determine the target endpoint for the segment
    target_position = segment.lane_boundaries[end].pt_b
    distance_to_target = norm(position - target_position)  # Euclidean distance in 2D

    # Define a tolerance within which we consider the segment end reached
    tolerance = 10.0  # Adjust based on your application's specific scale and needs

    # Log detailed information to help with debugging
    @info "Checking if segment end is reached" current_position=position target_position=target_position distance_to_target=distance_to_target

    # Check if within tolerance
    ans = distance_to_target < tolerance
    @info "Segment end status:" ans

    return ans
end


const WHEELBASE_LENGTH = 5.0  # meters, for example

mutable struct PIDController
    Kp::Float64
    Ki::Float64
    Kd::Float64
    previous_error::Float64
    integral::Float64
end

function update_pid(controller::PIDController, error::Float64, dt::Float64)
    derivative = (error - controller.previous_error) / dt
    controller.integral += error * dt
    controller.previous_error = error
    return controller.Kp * error + controller.Ki * controller.integral + controller.Kd * derivative
end

# Initialize the PID controllers for steering and velocity
steering_pid = PIDController(0.001, 0.01, 0.01, 0.0, 0.0)  # Tune these parameters
velocity_pid = PIDController(1.0, 0.1, 0.05, 0.0, 0.0)  # Tune these parameters
acceptable_deviation_threshold = 0.75
const SAFETY_MARGIN = 1.0
const ERROR_MARGIN = 35

function log_debug_info(segment, state, lookahead_point, steering_angle, velocity, yaw)
    @info "Segment ID: $(segment.id)"
    @info "Current Position: $(state.position)"
    @info "Velocity: $(state.velocity)"
    @info "Yaw: $yaw"
    @info "Lookahead Point: $lookahead_point"
    @info "Steering Angle: $steering_angle"
    @info "Velocity Command: $velocity"
end

function pure_pursuit_navigate(segment, localization_state_channel, state, socket, yaw)
    sleep(0.05)
    if isready(localization_state_channel)
        state = fetch(localization_state_channel)
        yaw = extract_yaw_from_quaternion(state.orientation)
    end

    @info "Evaluating vehicle position" current_position=state.position


    # Calculate lookahead distance and the lookahead point
    lookahead_distance = dynamic_lookahead(state.velocity, segment.lane_boundaries[1].curvature)

    raw_lookahead_point = compute_lookahead_point(segment, state.position, lookahead_distance)
    if !within_lane_boundaries(raw_lookahead_point, segment.lane_boundaries)
        lookahead_point = adjust_lookahead_within_boundaries(raw_lookahead_point, segment.lane_boundaries)
        @info "Adjusted lookahead point to stay within boundaries" lookahead_point
    else
        lookahead_point = raw_lookahead_point
        @info "Lookahead point is within boundaries" lookahead_point
    end

    deviation = calculate_deviation_from_center(state.position, calculate_lane_center(segment.lane_boundaries))
    @info "Checking deviation from lane center" deviation
    if deviation > ERROR_MARGIN
        @info "Deviation exceeds safety margin, vehicle too close to boundary, stopping."
        send_commands(0, 0, socket)
        return false
    else
        @info "Deviation within acceptable range" deviation
    end


    # Calculate steering angle
    steering_angle = calculate_steering(state, lookahead_point, yaw, segment.lane_boundaries[1].curvature)
    steering_angle= -steering_angle

    # Velocity control with PID adjustments
    dt = 0.1  # Simulation time step
    desired_velocity = adjust_velocity(segment, state.velocity, dt)
    velocity_correction = update_pid(velocity_pid, desired_velocity - norm(state.velocity[1:2]), dt)
    final_velocity = max(0.0, desired_velocity + velocity_correction)

    if is_at_segment_end(state, segment)
        @info "Segment end reached. Vehicle please stop if this is the last segment in the path."
        # Don't send a stop command here. Let the decision-making loop handle it.
        return true
    end

    if steering_angle != 0
        steering_angle = steering_angle/3
    end
    
    send_commands(steering_angle, final_velocity, socket)
    log_debug_info(segment, state, lookahead_point, steering_angle, final_velocity, yaw)

    return is_at_segment_end(state, segment)
end

function aligned_with_road(position, yaw, segment)
    # Check if the current heading aligns with the road's direction
    road_direction = atan(segment.lane_boundaries[end].pt_b[2] - segment.lane_boundaries[1].pt_a[2],
                           segment.lane_boundaries[end].pt_b[1] - segment.lane_boundaries[1].pt_a[1])
    return abs((yaw - road_direction + π) % (2π) - π) < 0.1  # Threshold for alignment
end

function adjust_lookahead_within_boundaries(point, lane_boundaries)
    closest_point = point
    min_distance = Inf
    for boundary in lane_boundaries
        projected_point = project_point_onto_line(point, boundary.pt_a, boundary.pt_b)
        distance = norm(point - projected_point)
        if distance < min_distance && within_segment(projected_point, boundary.pt_a, boundary.pt_b)
            min_distance = distance
            closest_point = projected_point
            @info "New closest point found within segment boundaries: $closest_point with distance $distance"
        end
    end
    if closest_point == point
        @info "No adjustment made, original point is used: $point"
    end
    return closest_point
end

function within_lane_boundaries(point, lane_boundaries)
    inside = false
    for i in 1:length(lane_boundaries)
        j = (i % length(lane_boundaries)) + 1
        if ((lane_boundaries[i].pt_a[2] > point[2]) != (lane_boundaries[j].pt_a[2] > point[2])) &&
            (point[1] < (lane_boundaries[j].pt_a[1] - lane_boundaries[i].pt_a[1]) * (point[2] - lane_boundaries[i].pt_a[2]) /
            (lane_boundaries[j].pt_a[2] - lane_boundaries[i].pt_a[2]) + lane_boundaries[i].pt_a[1])
            inside = !inside
        end
    end
    @info "Point $point boundary check against segments: " * (inside ? "inside" : "outside")
    return inside
end

function check_and_correct_course(position, lane_boundaries, current_steering_angle)
    deviation = calculate_deviation_from_center(position, calculate_lane_center(lane_boundaries))
    if abs(deviation) > acceptable_deviation_threshold
        correction = update_pid(steering_pid, deviation, 0.1)  # Assuming dt=0.1s
        return correction, false
    else
        return 0.0, true
    end
end

function dynamic_lookahead(velocity::SVector{3, Float64}, curvature::Float64)
    base_lookahead = 1.0
    speed_factor = norm(velocity[1:2]) / 10
    curvature_adjustment = max(1.0, 1 / abs(curvature))
    lookahead_distance = base_lookahead * speed_factor / curvature_adjustment
    return lookahead_distance
end

function calculate_steering(state, lookahead_point, yaw, curvature)
    @info "curvature: $(curvature)"

    if curvature == 0
        return 0  
    end

    angle_to_lookahead = atan(lookahead_point[2] - state.position[2], lookahead_point[1] - state.position[1])
    angle_difference = angle_to_lookahead - yaw
    steering_angle = atan(2 * WHEELBASE_LENGTH * sin(angle_difference) / norm(lookahead_point - state.position[1:2]))

    return steering_angle
end

function compute_lookahead_point(current_segment, current_position::SVector{3, Float64}, lookahead_distance::Float64)
    start_point = current_segment.lane_boundaries[1].pt_a
    end_point = current_segment.lane_boundaries[end].pt_b
    segment_vector = SVector(end_point[1] - start_point[1], end_point[2] - start_point[2])
    segment_unit_vector = segment_vector / norm(segment_vector)
    projected_length = dot(current_position[1:2] - start_point, segment_unit_vector)
    projected_point = start_point + projected_length * segment_unit_vector
    lookahead_point = projected_point + lookahead_distance * segment_unit_vector
    @info "projected point, lookahead point, projected length: $(projected_point) $(lookahead_point) $(projected_length)"
    return lookahead_point
end

function adjust_velocity(segment, current_velocity, dt)
    local min_required_velocity = 2.0  # Set a minimum required velocity
    local acceleration_rate = 0.5  # Increase acceleration rate
    desired_velocity = segment.speed_limit
    velocity_error = desired_velocity - norm(current_velocity[1:2])
    acceleration = clamp(velocity_error / dt, -acceleration_rate, acceleration_rate)  # Adjust clamp values

    new_velocity = norm(current_velocity[1:2]) + acceleration * dt
    if new_velocity < min_required_velocity
        new_velocity = min_required_velocity  # Ensure a minimum velocity if below threshold
    end
    return min(new_velocity, desired_velocity)
end

function correct_course(state, segment, pid_controller)
    # Calculate correction
    desired_correction = update_pid(pid_controller, position_projection, 0.1)  # Assuming dt=0.1s
    corrected_steering = apply_steering_limit(desired_correction)
    @info "PID correction: $desired_correction, limited to: $corrected_steering"
    return corrected_steering
end


function check_lane_position_and_correct(state, lane_boundaries, pid_controller)
    # Calculate closest distance to the lane boundaries
    lane_center = calculate_lane_center(lane_boundaries)
    deviation = calculate_deviation_from_center(state.position, lane_center)

    # Determine if a correction is necessary
    if abs(deviation) > acceptable_deviation_threshold
        correction = update_pid(pid_controller, deviation, 0.1)  # Adjust dt as needed
        apply_steering_correction(state, correction)
    end

    @info "Checked lane position" deviation = deviation
end

function calculate_lane_center(lane_boundaries)
    num_points = length(lane_boundaries)
    center_points = [(lane_boundaries[i].pt_a + lane_boundaries[i].pt_b) / 2 for i in 1:num_points]
    return mean(center_points)
end

function calculate_deviation_from_center(position, lane_center)
    # Simple Euclidean distance for this example, consider more complex geometric calculations as needed
    return norm(position[1:2] - lane_center[1:2])
end

function apply_steering_correction(state, correction)
    # Clamp correction to avoid excessive steering
    correction = clamp(correction, -max_steering_angle, max_steering_angle)
    new_steering_angle = state.steering_angle + correction
    state.steering_angle = new_steering_angle

    @info "Applied steering correction" correction = correction, new_angle = new_steering_angle
end

function apply_steering_limit(steering_angle)
    max_steering_angle = π / 4  # Limit to 45 degrees (about 0.785 radians)
    return clamp(steering_angle, -max_steering_angle, max_steering_angle)
end

function project_point_onto_line(point, pt_a, pt_b)
    line_vector = pt_b - pt_a
    point_vector = point - pt_a
    line_length_squared = dot(line_vector, line_vector)
    if line_length_squared == 0
        @info "Degenerate line segment from $pt_a to $pt_b used for projection."
        return pt_a
    end
    t = clamp(dot(point_vector, line_vector) / line_length_squared, 0.0, 1.0)
    projected_point = pt_a + t * line_vector
    @info "Point $point projected onto line from $pt_a to $pt_b as $projected_point"
    return projected_point
end

"""
    within_segment(point, pt_a, pt_b)

Check if a point is within the segment defined by pt_a and pt_b.
"""
function within_segment(point, pt_a, pt_b)
    # Check if point is between pt_a and pt_b inclusively
    if norm(pt_a - pt_b) == 0
        return point == pt_a
    end
    # Cross product must be zero if point is collinear, and check dot products for bounds
    collinear = (pt_b[2] - pt_a[2]) * (point[1] - pt_a[1]) == (point[2] - pt_a[2]) * (pt_b[1] - pt_a[1])
    within_bounds = minimum([pt_a[1], pt_b[1]]) <= point[1] <= maximum([pt_a[1], pt_b[1]]) &&
                    minimum([pt_a[2], pt_b[2]]) <= point[2] <= maximum([pt_a[2], pt_b[2]])
    return collinear && within_bounds
end