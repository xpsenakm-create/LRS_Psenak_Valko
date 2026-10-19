# Comparison flight: hangar_example.csv, safety_radius 0.30 m

Same mission and code as `safety_0.45_20261018`, only `safety_radius` lowered to 0.30 m
(2026-10-19). Files have the same meaning; `planner.yaml` is the configuration of this flight.

Result: all 7 waypoints and tasks in order, final state DONE, disarmed. Hard waypoints within
5.1–8.2 cm, soft within 0.31–0.42 m. **Minimum clearance 0.20 m** at t = 286 s,
pos (3.75, 7.46, 3.88), on the descent from the rack towards waypoint 5: less than the 0.33 m
drone radius, so the propellers would hit the rack edge. This is why the submitted configuration
uses 0.45 m. Duration 665 s at a Gazebo real-time factor of ~0.3.

Show the plots and the 3D view: `./show_a1_2.sh results/A1_2/safety_0.30_20261019`
