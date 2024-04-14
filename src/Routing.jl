# # Sample function to print a segment and its direct connections
# function print_segment_connections(segment_id, all_segs)
#     segment = all_segs[segment_id]
#     println("Segment ID: ", segment_id)
#     println("Connected to segments: ", join(segment.children, ", "))
# end

# # Function to traverse and print all segments' connections
# # This function is for demonstration and can help understand the graph's structure
# function print_all_connections(all_segs)
#     for (id, _) in all_segs
#         print_segment_connections(id, all_segs)
#     end
# end

# # Assuming `all_segs` is your map's segments dictionary
# # print_all_connections(all_segs)

# # Simple heuristic: straight-line distance between segment midpoints
# # function heuristic_cost(seg_a::Goal, seg_b::Goal)
# #     mid_a = (seg_a.lane_boundaries[1].pt_a + seg_a.lane_boundaries[end].pt_b) / 2
# #     mid_b = (seg_b.lane_boundaries[1].pt_a + seg_b.lane_boundaries[end].pt_b) / 2
# #     return norm(mid_a - mid_b)
# # end

# # Main A* function skeleton
# function a_star_search(all_segs, start_id, goal_id)
#     openSet = PriorityQueue()
#     openSet[start_id] = 0  # Priority queue initialized with the start segment

#     cameFrom = Dict()

#     gScore = Dict{Int, Float64}(start_id => 0)
#     fScore = Dict{Int, Float64}(start_id => 0)

#     while !isempty(openSet)
#         current_id, _ = dequeue!(openSet)
#         current_seg = all_segs[current_id]

#         # Goal check
#         if current_id == goal_id
#             return reconstruct_path(cameFrom, current_id)
#         end

#         for child_id in current_seg.children
#             tentative_gScore = gScore[current_id] + 1  # Assuming uniform cost for simplicity

#             if !haskey(gScore, child_id) || tentative_gScore < gScore[child_id]
#                 cameFrom[child_id] = current_id
#                 gScore[child_id] = tentative_gScore
#                 fScore[child_id] = gScore[child_id] + 0
#                 if !haskey(openSet, child_id)
#                     enqueue!(openSet, child_id, fScore[child_id])
#                 end
#             end
#         end
#     end

#     return nothing  # Path not found
# end

# # Reconstruct the path from the cameFrom map
# function reconstruct_path(cameFrom, current_id)
#     path = [current_id]
#     while haskey(cameFrom, current_id)
#         current_id = cameFrom[current_id]
#         push!(path, current_id)
#     end
#     return reverse(path)
# end

# # Convert path segments into waypoints
# function path_to_waypoints(path, all_segs)
#     waypoints = []
#     for seg_id in path
#         seg = all_segs[seg_id]
#         goal = Goal((seg.lane_boundaries[1].pt_a + seg.lane_boundaries[end].pt_b) / 2)
#         push!(waypoints, goal)
#     end
#     return waypoints
# end
