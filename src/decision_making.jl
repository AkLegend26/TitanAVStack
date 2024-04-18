function find_current_segment(position, map_segments, yaw)
    @info "Finding segment for position: $position"
    # @info map_segments
    best_segment_id = -1
    closest_distance = Inf

    function check_segments(segments, position)
        local_best_id = -1
        local_closest_dist = Inf

        for (id, segment) in segments
            @info "checking each segment"
            within, distance = check_segment(position, segment)
            if within && distance < local_closest_dist
                local_closest_dist = distance
                local_best_id = id
            end
        end

        return local_best_id, local_closest_dist
    end

    @info "checking segments"
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

        current_segment = map_segments[best_segment_id]

        # Use the utility functions to analyze the segment
        segment_curvature = get_segment_curvature(current_segment)
        segment_type = identify_segment_type(current_segment)
        curvature_direction = get_curvature_direction(current_segment)
        # vehicle_alignment = check_vehicle_alignment(position, yaw, current_segment)
        vehicle_alignment = check_vehicle_alignment(yaw, current_segment)


        @info "Segment Curvature: $segment_curvature"
        @info "Segment Type: $segment_type"
        @info "Curvature Direction: $curvature_direction"
        @info "Vehicle Alignment: $vehicle_alignment"

        # return [best_segment_id, segment_type, curvature_direction, vehicle_alignment]
        return best_segment_id
    else
        #@error "No current segment found for position: $position"
        return -1
    end
end

function get_segment_curvature(segment::VehicleSim.RoadSegment)
    # Check if the segment has any lane boundaries
    if isempty(segment.lane_boundaries)
        @warn "Road segment has no lane boundaries."
        return 0.0  # or any default value you prefer
    end

    # Assuming the curvature is the same for all lane boundaries,
    # we can take the curvature of the first lane boundary
    return segment.lane_boundaries[1].curvature
end

function identify_segment_type(segment::VehicleSim.RoadSegment)
    if isempty(segment.lane_boundaries)
        @warn "Road segment has no lane boundaries."
        return "Unknown"
    end

    if segment.lane_boundaries[1].curvature < 0.01  # Assume curvature close to zero is straight
        return "Straight"
    else
        return "Curved"
    end
end

function get_curvature_direction(segment::VehicleSim.RoadSegment)
    if isempty(segment.lane_boundaries)
        @warn "Road segment has no lane boundaries."
        return "Unknown"
    end

    if segment.lane_boundaries[1].curvature > 0
        return "Left"
    elseif segment.lane_boundaries[1].curvature < 0
        return "Right"
    elseif segment.lane_boundaries[1].curvature == 0
        return "Straight"
    else
        return "Unknown"
    end
end

# function check_vehicle_alignment(vehicle_pos, vehicle_heading, segment::VehicleSim.RoadSegment)
#     # Check if the segment has any lane boundaries
#     if isempty(segment.lane_boundaries)
#         @warn "Road segment has no lane boundaries."
#         return "Unknown"
#     end

#     # Calculate the average segment direction based on the first and last lane boundaries
#     first_boundary = segment.lane_boundaries[1]
#     last_boundary = segment.lane_boundaries[end]
#     segment_direction = atan(last_boundary.pt_b[2] - first_boundary.pt_a[2],
#                              last_boundary.pt_b[1] - first_boundary.pt_a[1])

#     heading_difference = abs((vehicle_heading - segment_direction + pi) % (2 * pi) - pi)

#     if heading_difference < pi / 4  # 45 degrees tolerance for alignment
#         return "Aligned"
#     else
#         return "Misaligned"
#     end
# end

function check_vehicle_alignment(vehicle_heading, segment::VehicleSim.RoadSegment)
    # Check if the segment has any lane boundaries
    if isempty(segment.lane_boundaries)
        @warn "Road segment has no lane boundaries."
        return "Unknown"
    end

    # Calculate the average segment direction based on the first and last lane boundaries
    first_boundary = segment.lane_boundaries[1]
    last_boundary = segment.lane_boundaries[end]
    segment_direction = atan(last_boundary.pt_b[2] - first_boundary.pt_a[2],
                             last_boundary.pt_b[1] - first_boundary.pt_a[1])

    heading_difference = abs((vehicle_heading - segment_direction + pi) % (2 * pi) - pi)

    if heading_difference < pi / 4  # 45 degrees tolerance for alignment
        return "Aligned"
    else
        return "Misaligned"
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

# function check_curved_segment(position, segment)


#     boundary = segment.lane_boundaries[1]
#     center = calculate_curve_center(boundary)
#     radius = calculate_curve_radius(boundary)
#     pt_a = boundary.pt_a
#     pt_b = boundary.pt_b
#     angle = atan(position[2] - center[2], position[1] - center[1])
#     distance = norm(position - center)

#     within_angle = is_within_angle(pt_a, pt_b, center, angle)
#     within_radius = radius * 0.9 <= distance <= radius * 1.1

#     return within_angle && within_radius, distance - radius
# end

function check_curved_segment(position, segment)
    # Extract the x and y components from the 3D position vector.
    pos_2d = position[1:2]

    boundary = segment.lane_boundaries[1]
    center = calculate_curve_center(boundary)
    radius = calculate_curve_radius(boundary)
    pt_a = boundary.pt_a
    pt_b = boundary.pt_b
    angle = atan(pos_2d[2] - center[2], pos_2d[1] - center[1])
    distance = norm(pos_2d - center)

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

# function calculate_curve_angles(center, pt_a, pt_b)
#     vec_a = pt_a - center
#     vec_b = pt_b - center
#     angle_a = atan(vec_a[2], vec_a[1])
#     angle_b = atan(vec_b[2], vec_b[1])
    
#     # Ensure angles are ordered correctly
#     if angle_a > angle_b
#         return angle_b, angle_a
#     else
#         return angle_a, angle_b
#     end
# end

# function navigate_segment(segment, state, yaw, socket)
#     if !isopen(socket)
#         @warn "Socket is closed, attempting to reconnect..."
#         return
#     end

#     try
#         # Validate the lane boundaries
#         if length(segment.lane_boundaries) < 2
#             @error "Invalid road segment: less than 2 lane boundaries."
#             return
#         end
#         if is_straight_segment(segment)
#             path_angle = calculate_path_angle(segment)  # Use a fixed path angle for straight segments
#         else
#             path_angle = calculate_dynamic_path_angle(segment, state.position)  # More complex handling for curved segments
#         end

#         path_error, cross_track_error = compute_path_and_cte(segment, state.position, yaw, path_angle)

#         steering_angle = calculate_steering_angle(path_error, cross_track_error, state.velocity[1])
#         max_steering_angle = π / 6  # Example: Clamp to 30 degrees
#         steering_angle = clamp(steering_angle, -max_steering_angle, max_steering_angle)
#         velocity = adjust_velocity(segment, cross_track_error)

#         # Clamp the steering angle to a max/min to prevent unrealistic steering
#         # max_steering_angle = π / 6  # Maximum steering angle, e.g., 30 degrees
#         # steering_angle = calculate_steering_angle(path_angle, cross_track_error, state.velocity[1])
#         # steering_angle = clamp(steering_angle, -max_steering_angle, max_steering_angle)

#         @info "Current state position: " state.position
#         @info "path angle:" path_angle
#         @info "Yaw: " yaw
#         @info "Path angle: " path_error
#         @info "Cross-track error: " cross_track_error
#         @info "Calculated steering angle: " steering_angle
#         @info "Calculated velocity: " velocity

#         @info "Navigating segment $(segment.id): Steering angle: $(steering_angle), Velocity: $(velocity)"

#         send_commands(steering_angle, velocity, socket)


#         return is_at_segment_end(state, segment)  
#     catch e
#         @error "An error occurred while navigating the segment" exception=(e, catch_backtrace())
#         return false
#     end
# end

# function calculate_dynamic_path_angle(segment, position)
#     # This function would calculate path angles based on segment curvature and vehicle position
#     # Placeholder for more complex path angle calculations
#     return atan(segment.lane_boundaries[end].pt_b[2] - segment.lane_boundaries[1].pt_a[2], 
#                  segment.lane_boundaries[end].pt_b[1] - segment.lane_boundaries[1].pt_a[1])
# end


# function calculate_path_angle(segment)
#     # Assuming this function now returns a consistent angle based on segment orientation
#     dir_vector = segment.lane_boundaries[end].pt_b - segment.lane_boundaries[1].pt_a
#     return atan(dir_vector[2], dir_vector[1])
# end

# function calculate_steering_angle(path_error, cross_track_error, velocity)
#     k_path = 0.05  # Further reduce path following gain
#     k_cte = 0.02   # Further reduce cross-track error gain

#     # Calculate steering adjustment based on reduced gains
#     steering_adjustment = atan(k_cte * cross_track_error / max(velocity, 0.1))  # Smoothing the response
#     adjusted_steering = path_error + steering_adjustment

#     # Limit the steering angle to prevent excessive turning
#     max_steering_angle = π / 18  # Limiting to about 10 degrees
#     return clamp(adjusted_steering, -max_steering_angle, max_steering_angle)
# end

# # function adjust_velocity(segment, cross_track_error)
# #     if any(lane_type -> lane_type == VehicleSim.stop_sign, segment.lane_types)
# #         # Special handling for stop signs
# #         return min(1.0, segment.speed_limit * 0.1)  # Example adjustment
# #     end

# #     curvature_effect = max(0.1, 1 - abs(mean([lb.curvature for lb in segment.lane_boundaries])))
# #     cte_effect = max(0, 1 - abs(cross_track_error) / 5)
# #     return segment.speed_limit * curvature_effect * cte_effect
# # end

# # Helper function to calculate the center of the current lane
# function calculate_lane_center(lane_boundaries)
#     # Assuming you're taking an average or a midpoint of some sort
#     midpoint = (lane_boundaries[1].pt_a + lane_boundaries[end].pt_b) / 2
#     return SVector{2, Float64}(midpoint[1], midpoint[2])
# end

# function compute_path_and_cte(segment, position, yaw, path_angle)
#     # Assuming a simple direct path from the start to end of segment
#     intended_path_angle = path_angle  # This should be the direction of the segment
#     path_error = intended_path_angle - yaw
#     path_error = mod(path_error + π, 2 * π) - π  # Normalize the error to be within -π to π

#     # Simple cross-track error based on perpendicular distance to the path
#     dx = cos(intended_path_angle) * (position[2] - segment.lane_boundaries[1].pt_a[2]) - sin(intended_path_angle) * (position[1] - segment.lane_boundaries[1].pt_a[1])
#     cross_track_error = abs(dx)

#     @info "Intended Path Angle: $intended_path_angle"
#     @info "Path Error: $path_error"
#     @info "Cross-Track Error: $cross_track_error"

#     return path_error, cross_track_error
# end


# # Stanley method for the steering control
# function stanley_control(path_angle, cross_track_error, velocity, yaw, k)
#     # Path error is normalized to the range [-π, π]
#     return path_angle + atan(k * cross_track_error / max(velocity, 0.1)) # Prevent division by zero
# end

# # Function to determine the velocity based on the segment properties
# function determine_velocity(segment, cross_track_error, velocity)
#     # Limit the velocity by the speed limit and potential curvature of the road
#     curvature = mean([boundary.curvature for boundary in segment.lane_boundaries])
#     max_velocity = segment.speed_limit

#     # Adjust the velocity based on the cross-track error, current velocity, and desired velocity change rate
#     velocity_factor = 1.0 - abs(cross_track_error) / 5.0  # Adjust the scaling factor as needed
#     velocity_factor = max(0.0, min(1.0, velocity_factor))
#     desired_velocity = max_velocity * velocity_factor

#     # Smooth the velocity change to avoid sudden jumps
#     velocity_change_rate = 1.0  # Adjust the rate of velocity change as needed
#     new_velocity = max(0.0, min(max_velocity, velocity + velocity_change_rate * (desired_velocity - velocity)))

#     return new_velocity
# end

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


# function is_straight_segment(segment::VehicleSim.RoadSegment)
#     # Check if all curvatures in the segment are negligible
#     return all(lb -> abs(lb.curvature) < 0.01, segment.lane_boundaries)
# end

# function compute_stanley_navigation_commands(segment, state, yaw)
#     # Determine the path's desired orientation at the closest point to the vehicle
#     lane_center = (segment.lane_boundaries[1].pt_a + segment.lane_boundaries[end].pt_b) / 2
#     path_angle = atan(lane_center[2] - state.position[2], lane_center[1] - state.position[1])
#     path_error = path_angle - yaw  # Heading error

#     # Cross-track error calculation should be norm of vector difference
#     cross_track_error = norm([lane_center[1] - state.position[1], lane_center[2] - state.position[2]])

#     # Stanley Control Law for Steering; calculate with correct division
#     k = 1.0  # Control gain
#     steering_angle = path_error + atan(k * cross_track_error / norm(state.velocity))  # Ensure state.velocity is not a vector here

#     # Velocity control could be simplified or based on PID controllers for more accuracy
#     velocity = min(segment.speed_limit, 2.0)  # Assuming a sensible constant speed for testing

#     return steering_angle, velocity
# end



# function point_within_curve(point, center, start_angle, end_angle, radius, width)
#     @info "Checking if point is within curved boundary"
#     rel_point = point - center
#     angle = atan(rel_point[2], rel_point[1])
#     distance = norm(rel_point)

#     # Normalize the angle to ensure it falls within the 0 to 2π range
#     if angle < 0
#         angle += 2 * π
#     end
#     if start_angle < 0
#         start_angle += 2 * π
#     end
#     if end_angle < 0
#         end_angle += 2 * π
#     end

#     # Check if the point's angle is between the start and end angles
#     angle_within = (start_angle <= angle <= end_angle) || (start_angle <= angle + 2 * π <= end_angle)

#     # Check if the point's distance from the center is within the radius bounds considering the road width
#     distance_within = (radius - width / 2 <= distance <= radius + width / 2)

#     result = angle_within && distance_within
#     @info "Point within curve: $result, Angle: $angle, Distance: $distance"
#     return result
# end


# Helper function to calculate the distance from a point to a line segment
# function point_to_segment_distance(point, segment)
#     #@info "Point input type and value: " typeof(point), point
#     if length(point) < 2
#         @error "Invalid point length"
#         return Inf
#     end
#     try
#         p = SVector{2, Float64}(point...)
#         a, b = segment.pt_a, segment.pt_b
#         ab = b - a
#         ap = p - a
#         t = dot(ap, ab) / dot(ab, ab)
#         t = clamp(t, 0.0, 1.0)
#         closest = a + t * ab
#         distance = norm(p - closest)
#         @info "Closest point on segment: $closest"
#         @info "Distance from point to segment: $distance"
#         return distance
#     catch e
#         @error "Failed to calculate point-to-segment distance" exception=(e, catch_backtrace())
#         return Inf
#     end
# end


# function is_within_curved_boundary(position, boundary)
#     @info "Checking if position is within curved boundary"
#     center = calculate_curve_center(boundary)
#     radius = calculate_curve_radius(boundary)
#     width = norm(boundary.pt_a - boundary.pt_b)
#     start_angle, end_angle = calculate_curve_angles(center, boundary.pt_a, boundary.pt_b)
#     result = point_within_curve(position, center, start_angle, end_angle, radius, width)
#     @info "Point within curved boundary: $result"
#     return result
# end

# function find_current_segment(position, map_segments)
#     @info "Finding segment for position: $position"
#     min_distance = Inf
#     best_segment_id = -1
#     for (id, segment) in map_segments
#         within, distance = is_within_segment(position, segment)
#         @info "Checked segment $id, within: $within, distance: $distance"
#         if within && distance < min_distance
#             min_distance = distance
#             best_segment_id = id
#         end
#     end
#     if best_segment_id != -1
#         @info "Segment found: $best_segment_id"
#         return best_segment_id
#     else
#         @error "No current segment found for position: $position"
#         return -1
#     end
# end

# function is_within_segment(position, segment)
#     if segment.lane_boundaries[1].curvature != 0
#         # Handle curved segment
#         center = calculate_curve_center(segment)
#         radius = calculate_curve_radius(segment)
#         width = norm(segment.lane_boundaries[1].pt_a - segment.lane_boundaries[1].pt_b)
#         start_angle, end_angle = calculate_curve_angles(center, segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b)
#         result = point_within_curve(position, center, start_angle, end_angle, radius, width)
#         distance = point_to_segment_distance(position, center, radius, start_angle, end_angle)  # This needs to be implemented
#         return result, distance
#     else
#         # Handle straight segment
#         distance = minimum(point_to_segment_distance(position, boundary) for boundary in segment.lane_boundaries)
#         return distance <= 0.1, distance
#     end
# end

const WHEELBASE_LENGTH = 1.1  # meters, for example
const MISALIGNMENT_CORRECTION_ANGLE = 0.0872665 # radians

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
velocity_pid = PIDController(10.0, 0.1, 0.05, 0.0, 0.0)  # Tune these parameters
acceptable_deviation_threshold = 0.75
const SAFETY_MARGIN = 1.0
const ERROR_MARGIN = 30

function log_debug_info(segment, state, lookahead_point, steering_angle, velocity, yaw)
    @info "Segment ID: $(segment.id)"
    @info "Current Position: $(state.position)"
    @info "Velocity: $(state.velocity)"
    @info "Yaw: $yaw"
    @info "Lookahead Point: $lookahead_point"
    @info "Steering Angle: $steering_angle"
    @info "Velocity Command: $velocity"
end

function pure_pursuit_navigate(segment, localization_state_channel, state, socket, yaw, map_segments)
    # sleep(0.05)
    if isready(localization_state_channel)
        state = fetch(localization_state_channel)
        yaw = extract_yaw_from_quaternion(state.orientation)
    end

    @info "Evaluating vehicle position" current_position=state.position

    # Find the current segment based on the vehicle's position
    current_segment_id = find_current_segment(state.position, map_segments, yaw)

    if current_segment_id != segment.id
        # The vehicle has transitioned to a new segment
        @info "Transitioned to new segment: $current_segment_id"
        segment = map_segments[current_segment_id]
    end

    # Calculate lookahead distance and the lookahead point
    lookahead_distance = dynamic_lookahead(state.velocity, segment.lane_boundaries[1].curvature, segment, yaw)

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
        send_commands(0.0, 0.0, socket)
        return false
    else
        @info "Deviation within acceptable range" deviation
    end


    # Calculate steering angle
    steering_angle = calculate_steering(state, lookahead_point, yaw, segment.lane_boundaries[1].curvature, segment)

    # Velocity control with PID adjustments
    dt = 0.1  # Simulation time step
    desired_velocity = adjust_velocity(segment, state.velocity, dt, yaw)
    velocity_correction = update_pid(velocity_pid, desired_velocity - norm(state.velocity[1:2]), dt)
    final_velocity = max(0.0, desired_velocity + velocity_correction)

    if is_at_segment_end(state, segment)
        @info "Segment end reached. Vehicle please stop if this is the last segment in the path."
        # Don't send a stop command here. Let the decision-making loop handle it.
        return true
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
            (point[1] < ((lane_boundaries[j].pt_a[1] - lane_boundaries[i].pt_a[1]) * (point[2] - lane_boundaries[i].pt_a[2]) /
            (lane_boundaries[j].pt_a[2] - lane_boundaries[i].pt_a[2]) + lane_boundaries[i].pt_a[1]))
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

function dynamic_lookahead(velocity::SVector{3, Float64}, curvature::Float64, segment::VehicleSim.RoadSegment, vehicle_heading)
    base_lookahead = 1.0
    speed_factor = norm(velocity[1:2]) / 10
    curvature_adjustment = max(1.0, 1 / abs(curvature))

    segment_type = identify_segment_type(segment)
    curvature_direction = get_curvature_direction(segment)
    vehicle_alignment = check_vehicle_alignment(vehicle_heading, segment)

    # Adjust based on segment type
    segment_type_factor = segment_type == "Straight" ? 1.0 : 1.5  # Increase lookahead for curved segments

    # Adjust based on curvature direction
    curvature_direction_factor = curvature_direction == "Left" ? 1.2 : 0.8  # Increase lookahead for left curves

    # Adjust based on vehicle alignment
    alignment_factor = vehicle_alignment == "Aligned" ? 1.0 : 1.5  # Increase lookahead when misaligned

    lookahead_distance = base_lookahead * speed_factor / curvature_adjustment * segment_type_factor * curvature_direction_factor * alignment_factor

    return lookahead_distance
end

function calculate_steering(state, lookahead_point, yaw, curvature, segment::VehicleSim.RoadSegment)
    @info "curvature: $(curvature)"
    if curvature == 0
        return 0
    end

    angle_to_lookahead = atan(lookahead_point[2] - state.position[2], lookahead_point[1] - state.position[1])
    angle_difference = angle_to_lookahead - yaw

    segment_type = identify_segment_type(segment)
    curvature_direction = get_curvature_direction(segment)
    vehicle_alignment = check_vehicle_alignment(yaw, segment)

    # Adjust the steering angle based on segment type, curvature direction, and vehicle alignment
    if segment_type == "straight"
        steering_angle = atan(2 * WHEELBASE_LENGTH * sin(angle_difference) / norm(lookahead_point - state.position[1:2]))
    elseif segment_type == "curved"
        if curvature_direction == "left"
            steering_angle = atan(2 * WHEELBASE_LENGTH * sin(angle_difference) / (norm(lookahead_point - state.position[1:2]) * curvature))
        elseif curvature_direction == "right"
            steering_angle = -atan(2 * WHEELBASE_LENGTH * sin(angle_difference) / (norm(lookahead_point - state.position[1:2]) * curvature))
        end
    end

    if vehicle_alignment == "misaligned"
        # Apply an additional correction to the steering angle to account for vehicle misalignment
        steering_angle += MISALIGNMENT_CORRECTION_ANGLE
    end
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

function adjust_velocity(segment, current_velocity, dt, vehicle_heading)
    local min_required_velocity = 2.0  # Set a minimum required velocity
    local acceleration_rate = 0.5  # Increase acceleration rate
    desired_velocity = segment.speed_limit
    velocity_error = desired_velocity - norm(current_velocity[1:2])
    acceleration = 0

    segment_type = identify_segment_type(segment)
    curvature_direction = get_curvature_direction(segment)
    vehicle_alignment = check_vehicle_alignment(vehicle_heading, segment)

    # Adjust the acceleration based on segment type, curvature direction, and vehicle alignment
    if segment_type == "straight"
        acceleration = clamp(velocity_error / dt, -acceleration_rate, acceleration_rate)
    elseif segment_type == "curved"
        if curvature_direction == "left"
            acceleration = clamp(velocity_error / dt, -acceleration_rate * 0.8, acceleration_rate * 0.8)
        elseif curvature_direction == "right"
            acceleration = clamp(velocity_error / dt, -acceleration_rate * 0.6, acceleration_rate * 0.6)
        end
    end
    
    if vehicle_alignment == "misaligned"
        # Apply a more conservative acceleration/deceleration for a misaligned vehicle
        acceleration = clamp(velocity_error / dt, -acceleration_rate * 0.5, acceleration_rate * 0.5)
    end
    
    new_velocity = norm(current_velocity[1:2]) + acceleration * dt
    if new_velocity < min_required_velocity
        new_velocity = min_required_velocity # Ensure a minimum velocity if below threshold
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

# function within_lane_boundaries(point, lane_boundaries)
#     count = 0
#     n = length(lane_boundaries)
#     for i in 1:n
#         j = i % n + 1
#         if ((lane_boundaries[i].pt_a[2] > point[2]) != (lane_boundaries[j].pt_a[2] > point[2])) &&
#             (point[1] < (lane_boundaries[j].pt_a[1] - lane_boundaries[i].pt_a[1]) *
#             (point[2] - lane_boundaries[i].pt_a[2]) / (lane_boundaries[j].pt_a[2] - lane_boundaries[i].pt_a[2]) +
#             lane_boundaries[i].pt_a[1])
#             count += 1
#         end
#     end
#     return count % 2 == 1
# end


# function within_lane_boundaries(position, lane_boundaries)
#     # Assuming the first half of the lane_boundaries array defines the right half of the road.
#     num_boundaries = length(lane_boundaries) ÷ 2
#     points = [b.pt_a for b in lane_boundaries[1:num_boundaries]]  # Use only the right half
#     push!(points, lane_boundaries[num_boundaries].pt_b)  # Add the endpoint of the last boundary

#     # Determine if the position is within the polygon defined by these points
#     inside = false
#     j = lastindex(points)
#     for i in 1:length(points)
#         if ((points[i][2] > position[2]) != (points[j][2] > position[2])) &&
#            (position[1] < (points[j][1] - points[i][1]) * (position[2] - points[i][2]) / 
#             (points[j][2] - points[i][2]) + points[i][1])
#             inside = !inside
#         end
#         j = i
#     end
#     return inside
# end
