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

The `safety_radius` ROS parameter controls the 3D obstacle inflation radius.
Its value is supplied in `config/planner.yaml` and can be overridden at launch.
The drone collision body's horizontal
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

From the workspace root, run the executable launcher:

```sh
./run_a1_1.sh
```

It builds the package, starts the planner, opens RViz with the occupancy and
path displays enabled, and requests a demonstration route. The final inflated
occupancy voxel centers, including rack volumes, are also written as
`map_voxels_with_racks_inflated.pcd` in the workspace root. Set the effective
3D obstacle inflation/safety radius with `./run_a1_1.sh --safety-radius 0.75`
or its alias `./run_a1_1.sh --inflation-radius 0.75` (`--radius` remains an
alias). These all set the same ROS parameter; inflation radius and safety radius
are not separate quantities in this planner. Use `./run_a1_1.sh --no-rviz`
to generate the PCD and run without opening RViz. The initial
start and goal can be supplied as coordinate triples:

```sh
./run_a1_1.sh --start 12 4 2 --goal 16 8 2
```

The planner stays running after the initial route. Publish another request to
`/plan_request` at any time to replace the displayed trajectory; no restart is
needed. Press Ctrl+C to stop the processes.

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
The launcher opens the saved view in `config/planner.rviz`. To open RViz
separately:

```sh
source install/setup.bash
rviz2
```

The saved view uses Fixed Frame `map` and displays:

- **PointCloud2**, topic `/map_cloud`, for the downsampled PCD.
- **PointCloud2**, topic `/inflated_occupancy`, for occupied voxel centers,
  shown as small, low-opacity points so the cloud does not obscure the scene.
- **Marker**, topic `/rack_boxes_marker`, for amber wireframes of the solid
  rack collision volumes.
- **Marker**, topic `/planned_path_marker`, for the collision-checked path line.
- **Grid**, as a spatial reference if useful.

Map point clouds use transient-local QoS and should appear automatically. The
successful path appears as a green line. To query the center of a configured
rack box, run:

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

The ROS node was exercised on the included PCD with the rack boxes from the
parameter file; successful paths passed the independent inflated-map check.
The final occupied voxel cloud is available as both `/inflated_occupancy` and
the saved PCD file, so the filled rack volumes can be inspected directly.

| Start -> goal (m) | Radius (m) | Raw -> simplified | Path length (m) | Planning time (ms) |
| --- | ---: | ---: | ---: | ---: |
| (12, 4, 2) -> (16, 4, 2) | 0.45 | 28 -> 2 | 4.000 | measured at runtime |
| (12, 4, 2) -> (12, 8, 2) | 0.45 | 28 -> 2 | 4.000 | measured at runtime |

Planning time varies with machine load and is printed by the node on each
request. Change `safety_radius` and repeat a route request to compare the
resulting path and occupancy visualization.