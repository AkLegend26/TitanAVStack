# Helper function to calculate the distance from a point to a line segment
function point_to_segment_distance(point, segment)
    @info "Calculating point-to-segment distance"
    try
        p = point
        a, b = segment.pt_a, segment.pt_b
        @info "Segment start (pt_a): $a, end (pt_b): $b"  # Debugging info
        if a == b
            distance = norm(p - a)
            @info "Distance from point to segment (single point): $distance"
            return distance
        end
        ab = b - a
        ap = p - a
        t = dot(ap, ab) / dot(ab, ab)
        t = clamp(t, 0.0, 1.0)
        closest = a + t * ab
        distance = norm(p - closest)
        @info "Closest point on segment: $closest"
        @info "Distance from point to segment: $distance"
        return distance
    catch e
        @error "Failed to calculate point-to-segment distance" exception=(e, catch_backtrace())
        return Inf
    end
end



function point_within_curve(point, center, start_angle, end_angle, radius, width)
    @info "Checking if point is within curved boundary"
    rel_point = point - center
    angle = atan2(rel_point[2], rel_point[1])
    distance = norm(rel_point)

    # Normalize the angle to ensure it falls within the 0 to 2π range
    if angle < 0
        angle += 2 * π
    end
    if start_angle < 0
        start_angle += 2 * π
    end
    if end_angle < 0
        end_angle += 2 * π
    end

    # Check if the point's angle is between the start and end angles
    angle_within = (start_angle <= angle <= end_angle) || (start_angle <= angle + 2 * π <= end_angle)

    # Check if the point's distance from the center is within the radius bounds considering the road width
    distance_within = (radius - width / 2 <= distance <= radius + width / 2)

    result = angle_within && distance_within
    @info "Point within curve: $result, Angle: $angle, Distance: $distance"
    return result
end

function is_within_curved_boundary(position, boundary)
    @info "Checking if position is within curved boundary"
    center = calculate_curve_center(boundary)
    radius = calculate_curve_radius(boundary)
    width = norm(boundary.pt_a - boundary.pt_b)
    start_angle, end_angle = calculate_curve_angles(center, boundary.pt_a, boundary.pt_b)
    result = point_within_curve(position, center, start_angle, end_angle, radius, width)
    @info "Point within curved boundary: $result"
    return result
end

function find_current_segment(position, map_segments)
    @info "inside find_cur_segment"
    for (id, segment) in map_segments
        if is_within_segment(position, segment)
            @info "Processing segment" segment_id=id
            return id
        end
    end
    @error "No current segment found for position: $position"
    return -1
end

# Determine if the position is within the given road segment
function is_within_segment(position, segment)
    try
        @info "Processing segment" segment_id = segment.id
        if segment.lane_boundaries[1].curvature != 0
            # Handle curved segment
            center = calculate_curve_center(segment)
            radius = calculate_curve_radius(segment)
            width = norm(segment.lane_boundaries[1].pt_a - segment.lane_boundaries[1].pt_b)
            start_angle, end_angle = calculate_curve_angles(center, segment.lane_boundaries[1].pt_a, segment.lane_boundaries[1].pt_b)
            return point_within_curve(position, center, start_angle, end_angle, radius, width)
        else
            # Handle straight segment
            return all(boundary -> point_to_segment_distance(position, boundary) <= 0.1, segment.lane_boundaries)
        end
    catch e
        @error "Error while processing segment" segment_id = segment.id, exception = (e, catch_backtrace())
        return false
    end
end

# Placeholder for the A* search algorithm to find the shortest path between two segments
function shortest_path(start_id, target_id, map_segments)
    @info "Starting shortest path calculation"
    queue = [start_id]
    came_from = Dict(start_id => nothing)

    while !isempty(queue)
        current = popfirst!(queue)
        @info "Current segment in BFS: $current"

        if current == target_id
            @info "Target segment reached: $current"
            path = []
            while current !== nothing
                push!(path, current)
                current = came_from[current]
            end
            return reverse(path)
        end

        for neighbor in map_segments[current].children
            @info "Checking neighbor: $neighbor"
            if !haskey(came_from, neighbor)
                push!(queue, neighbor)
                came_from[neighbor] = current
            end
        end
    end

    @info "No path found"
    return []
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
    angle_a = atan2(vec_a[2], vec_a[1])
    angle_b = atan2(vec_b[2], vec_b[1])
    
    # Ensure angles are ordered correctly
    if angle_a > angle_b
        return angle_b, angle_a
    else
        return angle_a, angle_b
    end
end

function compute_navigation_commands(segment, state)
    # Assuming the vehicle needs to adjust to the center of the road
    lane_center = (segment.lane_boundaries[1].pt_a + segment.lane_boundaries[end].pt_b) / 2
    heading_to_target = atan2(lane_center[2] - state.position[2], lane_center[1] - state.position[1])
    steering_angle = heading_to_target - state.orientation

    # Simplified control for velocity based on segment speed limit
    velocity = min(segment.speed_limit, norm(state.velocity))

    return steering_angle, velocity
end

function navigate_segment(segment, state, socket)
    steering_angle, velocity = compute_navigation_commands(segment, state)
    cmd = VehicleCommand(steering_angle, velocity, true)
    serialize(socket, cmd)
    @info "Navigating segment $(segment.id): Steering angle: $(steering_angle), Velocity: $(velocity)"
end