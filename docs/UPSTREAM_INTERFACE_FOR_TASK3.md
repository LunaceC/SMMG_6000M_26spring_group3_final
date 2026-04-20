# Upstream Interface For Task 3

Authoritative upstream artifact:
- `artifacts/task2_cartesian_paths.json`

Task 3 must consume these top-level fields:
- `task1_cycle_node_order`
- `task1_weld_order`
- `obstacle_model`
- `ordered_segment_records`
- `segment_count`
- `collision_free_segment_count`
- `all_segments_valid`

Task 3 must consume these per-segment fields from `ordered_segment_records`:
- `segment_index`
- `source_label`
- `target_label`
- `source_state_id`
- `target_state_id`
- `workspace_polyline`
- `workspace_path_length`
- `cartesian_waypoints`
- `chosen_joint_waypoints`
- `validation_result`
- `joint_space_path_length`
- `local_repair_used`
- `local_repair_reason`

Task 3 must preserve these conventions:
- the Task 1 visit order and depot anchoring
- the scene and weld definitions from `scene.py` and `welds.py`
- joint units in radians
- the object and weld naming scheme
- `inflation_radius_mm = 6.0`
- `circle_polygon_sides = 48`
- `segment_count = 25`, with the first segment leaving `DEPOT` and the final segment returning to `DEPOT`
