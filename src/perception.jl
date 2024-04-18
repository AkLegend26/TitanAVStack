function assess_threat(position, velocity, localization_state)
    ego_position = localization_state.position[1:2]
    ego_velocity = localization_state.velocity[1:2]
    
    relative_velocity = norm(velocity - ego_velocity)
    distance = norm(position - ego_position)

    # More sophisticated risk assessment
    risk_factor = (relative_velocity / max_speed) + (safe_distance / max(distance, 1))
    return clamp(risk_factor, 0, 1)  # Normalize between 0 and 1
end

function bbox_to_world(camera_meas, bbox, localization_state)
    @info "Transforming bbox to world coordinates", bbox=bbox
    try
        camera_id = camera_meas.camera_id
        focal_length = camera_meas.focal_length
        pixel_length = camera_meas.pixel_length
        image_width = camera_meas.image_width
        image_height = camera_meas.image_height

        cx = (bbox[1] + bbox[3]) / 2
        cy = (bbox[2] + bbox[4]) / 2
        position = [cx * pixel_length, cy * pixel_length, 0] 
        velocity = [0.0, 0.0]  

        @info "Calculated position and velocity from bbox", position=position, velocity=velocity
        return position, velocity
    catch e
        @error "Error transforming bbox to world coordinates", exception=(e, catch_backtrace())
        return [0.0, 0.0], [0.0, 0.0]  # Return default values on error
    end
end


function process_bbox(bbox, cam_meas, loc_state)
    position, velocity = bbox_to_world(cam_meas, bbox, loc_state)
    threat_level = assess_threat(position, velocity, loc_state)
    DetectedObject(object_id_counter, bbox, position, velocity, threat_level)
end

function transform_bbox_to_position(T_cam_to_world, bbox, focal_length, pixel_length, image_width, image_height)
    # Convert pixel coordinates to camera coordinates
    px, py = (bbox[1] + bbox[3]) / 2, (bbox[2] + bbox[4]) / 2
    z = focal_length  # Assuming a simple pinhole model

    x_cam = (px - image_width / 2) * pixel_length
    y_cam = (py - image_height / 2) * pixel_length

    # Camera coordinates to world coordinates
    world_coords = T_cam_to_world[1:3, 1:3] * [x_cam, y_cam, z] + T_cam_to_world[1:3, 4]

    return world_coords
end


function get_cam_to_world_transform(camera_id, localization_state)
    # Fetch camera calibration data
    T_cam_to_body = get_cam_transform(camera_id)  # Assuming this fetches a valid transformation matrix
    R_world_to_body = Rot_from_quat(localization_state.orientation)
    t_world_to_body = localization_state.position

    T_world_to_body = [R_world_to_body t_world_to_body; 0 0 0 1]
    T_cam_to_world = inv(T_world_to_body * T_cam_to_body)

    return T_cam_to_world[1:3, 1:3], T_cam_to_world[1:3, 4]
end
