# Recorded mission: hangar_example.csv (A1.2 deliverable)

Complete unattended mission in ArduCopter SITL + Gazebo, from start to disarm after the final
landing, flown with the final code of branch `A1_2` on 2026-10-18.

| File | Content |
|---|---|
| `bag/` | rosbag (MCAP): `/mavros/state`, local pose and velocity, setpoints, FCU texts, plan requests, planned paths, planning status, markers |
| `flight_log.csv` | 20 Hz executor log (state, commanded and measured pose, map frame) |
| `planned_paths.csv` | every A* route that was flown |
| `topview.png`, `overview.png` | plots made by `plot_a1_2.py` |
| `mission.csv` | the mission that was flown |

Result: all 7 waypoints and tasks in order (takeoff, -, yaw180, -, landtakeoff, yaw90, land),
final state DONE, disarmed. Hard waypoints reached within 3.9–7.3 cm (tolerance 0.15 m), soft
within 0.30–0.40 m (tolerance 0.55 m). Minimum clearance to any obstacle 0.37 m (drone radius
0.33 m). Duration 404 s at a Gazebo real-time factor of ~0.3.

Replay / inspect:

```bash
ros2 bag info bag
ros2 bag play bag                         # e.g. with RViz: rviz2 -d src/lrs_psenak_valko/config/planner.rviz
python3 plot_a1_2.py results/hangar_example_20261018
python3 view_3d_a1_2.py results/hangar_example_20261018
```
