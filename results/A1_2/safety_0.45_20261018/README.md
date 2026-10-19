# Recorded mission: hangar_example.csv, safety_radius 0.45 m (A1.2 deliverable)

Complete unattended mission in ArduCopter SITL + Gazebo, from start to disarm after the final
landing, flown with the final code of branch `A1_2` on 2026-10-18.

| File | Content |
|---|---|
| `bag/` | rosbag (MCAP): `/mavros/state`, local pose and velocity, setpoints, FCU texts, plan requests, planned paths, planning status, markers |
| `flight_log.csv` | 20 Hz executor log (state, commanded and measured pose, map frame) |
| `planned_paths.csv` | every A* route that was flown |
| `topview.png`, `overview.png` | plots made by `plot_a1_2.py` |
| `mission.csv` | the mission that was flown |
| `planner.yaml` | planner configuration of this flight (safety_radius 0.45 m) |

Result: all 7 waypoints and tasks in order (takeoff, -, yaw180, -, landtakeoff, yaw90, land),
final state DONE, disarmed. Hard waypoints reached within 3.9–7.3 cm (tolerance 0.15 m), soft
within 0.30–0.40 m (tolerance 0.55 m). Minimum clearance to any obstacle 0.37 m (drone radius
0.33 m). Duration 404 s at a Gazebo real-time factor of ~0.3.

Show the plots and the 3D view: `./show_a1_2.sh results/A1_2/safety_0.45_20261018`
Rosbag: `ros2 bag info bag`, `ros2 bag play bag`.
