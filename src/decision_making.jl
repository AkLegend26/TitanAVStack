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
        #@info "Segment found: $best_segment_id"
        return best_segment_id
    else
        #@error "No current segment found for position: $position"
        return -1
    end
end

function check_segment(position, segment)
    #@info "Checking segment: $segment"
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

function calculate_curve_angles(center, pt_a, pt_b)
    vec_a = pt_a - center
    vec_b = pt_b - center
    angle_a = atan(vec_a[2], vec_a[1])
    angle_b = atan(vec_b[2], vec_b[1])
    
    # Ensure angles are ordered correctly
    if angle_a > angle_b
        return angle_b, angle_a
    else
        return angle_a, angle_b
    end
end


function navigate_segment(segment, state, yaw, socket)
    try
        if !isopen(socket)
            @warn "Socket is closed, attempting to reconnect..."
            #uuh how to reconnec5
        end
        steering_angle, velocity = compute_stanley_navigation_commands(segment, state, yaw)
        cmd = (steering_angle, velocity, true)
        serialize(socket, cmd)
        @info "Navigating segment $(segment.id): Steering angle: $(steering_angle), Velocity: $(velocity)"
    catch e
        @error "An error occurred while navigating the segment" exception=(e, catch_backtrace())
    end
end


function compute_stanley_navigation_commands(segment, state, yaw)
    # Determine the path's desired orientation at the closest point to the vehicle
    lane_center = (segment.lane_boundaries[1].pt_a + segment.lane_boundaries[end].pt_b) / 2
    path_angle = atan(lane_center[2] - state.position[2], lane_center[1] - state.position[1])
    path_error = path_angle - yaw  # Heading error

    # Cross-track error calculation should be norm of vector difference
    cross_track_error = norm([lane_center[1] - state.position[1], lane_center[2] - state.position[2]])

    # Stanley Control Law for Steering; calculate with correct division
    k = 1.0  # Control gain
    steering_angle = path_error + atan(k * cross_track_error / norm(state.velocity))  # Ensure state.velocity is not a vector here

    # Velocity control could be simplified or based on PID controllers for more accuracy
    velocity = min(segment.speed_limit, 10.0)  # Assuming a sensible constant speed for testing

    return steering_angle, velocity
end



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