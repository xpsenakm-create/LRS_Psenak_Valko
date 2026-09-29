# 3D Map Planner

The package loads `maps/FEI_LRS_PCD/map.pcd`, downsamples it with PCL's 3D
`VoxelGrid`, and stores occupied voxels in a sparse 3D hash grid. The planner
copies the bounded inflated grid into a dense array for 26-connected A*.
Occupancy is queried in all three coordinates; this is not a stack of 2D maps.

## Obstacles and Safety

`config/planner.yaml` supplies two solid 3D bounding boxes for the rack
collision volumes. Each box is `6.5 x 3.76 x 5.59 m`, matching the specified
Gazebo collision size, and spans all shelf levels so no path can use the
visual-mesh gaps. The centers use the two rack-row locations in the checked-in
`models/fei_lrs_racks/model.sdf`. If the simulation rack placement changes,
update these ROS parameter values. The node refuses to start without at least
one rack box, so it cannot silently plan through unconfigured shelves.

The configured `safety_radius` is 0.45 m. The drone collision body's horizontal
half-width is about 0.174 m; the remaining 0.276 m is a conservative allowance
for controller position error (0.15 m) plus an additional 0.126 m clearance
margin. These allowances are explicit engineering assumptions and should be
replaced with measured controller tracking error when available. Inflation is
a 3D sphere and is applied after filling rack boxes. Change the ROS parameter
and restart the node to compare paths at another radius.

## Planner

The node runs optimal A* by default (`heuristic_weight: 1.0`). Its 3D octile
heuristic is the exact obstacle-free cost on a 26-connected voxel grid, hence
admissible and consistent. Diagonal moves are rejected if any voxel in their
swept corner volume is blocked. The search has a wall-clock limit and returns
`NO_PATH` when the open set is exhausted, or `TIMEOUT` when its configured
limit is reached.

Line-of-sight shortcutting samples each candidate segment against the inflated
grid. Before publication, every simplified segment is independently checked
again against the sparse inflated occupancy map. Planning time, expansions,
raw and simplified waypoint counts, and path length are published as status.

## Run

Build and source the workspace, then start the ROS control node with the
parameter file:

```sh
colcon build --packages-select lrs_psenak_valko
source install/setup.bash
ros2 run lrs_psenak_valko map_planner_node --ros-args --params-file \
  src/lrs_psenak_valko/config/planner.yaml
```

Publish a `geometry_msgs/msg/PoseArray` to `/plan_request` containing exactly
two poses: start, then goal. Coordinates are in the map frame, in metres. The
collision-checked path is published on `/planned_path`; `/planning_status`
reports success or the specific failure status. `/occupancy_query` accepts a
`geometry_msgs/msg/PointStamped`; `/occupancy_result` returns `true` when the
queried point is occupied in the inflated map or outside the planning grid.
For example:

```sh
ros2 topic pub --once /plan_request geometry_msgs/msg/PoseArray \
  "{header: {frame_id: map}, poses: [{position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}, {position: {x: 12.0, y: 12.0, z: 1.5}, orientation: {w: 1.0}}]}"
```

The example endpoints must be replaced with free points inside the loaded map.
The rack definitions and all planner settings are ROS parameters in the YAML
file, not custom command-line arguments.

## Visualize and Query

RViz is part of the installed ROS environment; no extra packages are needed.
Start it in another terminal:

```sh
source install/setup.bash
rviz2
```

In RViz, set **Fixed Frame** to `map`, then add these displays:

- **PointCloud2**, topic `/map_cloud`, for the downsampled PCD.
- **PointCloud2**, topic `/inflated_occupancy`, for occupied voxel centers,
  including the filled rack boxes and their safety inflation.
- **Marker**, topic `/planned_path_marker`, for the collision-checked path line.
- **Grid**, as a spatial reference if useful.

After RViz is open, submit either plan request shown above; the map point clouds
use transient-local QoS and should appear automatically. The successful path
will appear as a green line. To query the center of a configured rack box, run:

```sh
ros2 topic echo --once /occupancy_result
```

Then in another terminal publish:

```sh
ros2 topic pub --once /occupancy_query geometry_msgs/msg/PointStamped \
  "{header: {frame_id: map}, point: {x: 4.36, y: 6.818186, z: 2.795}}"
```

The result should be `data: true`. Points outside the bounded planning grid
also return `true`, matching the planner's rule that out-of-bounds is blocked.

## Validation Results

The synthetic planner self-test produces a collision-free 3D route with 81 raw
waypoints reduced to 3, and reports `NO_PATH` for a sealed goal. The ROS node
was also exercised on the included PCD with the rack boxes from the parameter
file; all successful paths passed the independent inflated-map check.

| Start -> goal (m) | Radius (m) | Raw -> simplified | Path length (m) | Planning time (ms) |
| --- | ---: | ---: | ---: | ---: |
| (12, 4, 2) -> (16, 4, 2) | 0.45 | 28 -> 2 | 4.000 | 1.70 |
| (12, 4, 2) -> (12, 8, 2) | 0.45 | 28 -> 2 | 4.000 | 0.34 |

For a deterministic radius A/B check, the standalone test plans around the same
3D wall at both radii: 0.45 m produced a 5.777 m path; 0.75 m produced a
6.172 m path. Both plans succeeded, and the larger radius selected a visibly
longer detour. A separate sealed-goal synthetic case terminates with `NO_PATH`
rather than hanging.