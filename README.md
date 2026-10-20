# LRS — Psenak, Valko: A1.1 + A1.2

Requirements: ROS 2 Jazzy, MAVROS, PCL (`sudo apt install libpcl-dev`), and the course repo
[LRS-URK](https://github.com/KocurMaros/LRS-URK) cloned in `~/LRS-URK` (Gazebo, SITL and MAVROS
scripts). Every script builds the package itself (`colcon build`).

## Run

```bash
# A1.1 — planner + RViz (no simulator)
~/Zadania/LRS_Psenak_Valko/run_a1_1.sh

# A1.2 — indoor mission (starts Gazebo, SITL, MAVROS itself; mission file is optional)
~/Zadania/LRS_Psenak_Valko/run_a1_2.sh ~/Zadania/LRS_Psenak_Valko/src/lrs_psenak_valko/missions/hangar_example.csv

# A1.3 — outdoor mission (branch A1_3)
cd ~/Zadania/LRS_Psenak_Valko && git switch A1_3 && ./run_a1_3.sh --record
```

A1.1 with another route or radius: `run_a1_1.sh --start 12 4 2 --goal 16 8 2 --safety-radius 0.75`.
After an A1.2 mission the plots and the 3D view open by themselves.

## Graphs and 3D view of a recorded flight

One command opens the top view, the overview plots and the animated 3D view
(`space` = play the route, `q` = quit):

```bash
~/Zadania/LRS_Psenak_Valko/show_a1_2.sh                                   # submitted recording (0.45 m)
~/Zadania/LRS_Psenak_Valko/show_a1_2.sh results/A1_2/safety_0.30_20261019 # comparison flight (0.30 m)
~/Zadania/LRS_Psenak_Valko/show_a1_2.sh log/a1_2_runs/<run>               # any run of your own
```

Recordings (rosbag, logs, PNG plots) are in `results/A1_2/`, one folder per safety radius.

## Documentation

| | Slovak | English |
|---|---|---|
| **A1 (all parts, for submission)** | [docs/A1/A1_dokumentacia.pdf](docs/A1/A1_dokumentacia.pdf) | [docs/A1/A1_documentation_EN.pdf](docs/A1/A1_documentation_EN.pdf) |
| A1.1 | [docs/A1_1/A1_1_mapa_a_planovanie.pdf](docs/A1_1/A1_1_mapa_a_planovanie.pdf) | [docs/A1_1/A1_1_map_and_planning_EN.pdf](docs/A1_1/A1_1_map_and_planning_EN.pdf) |
| A1.2 | [docs/A1_2/A1_2_indoor_misia.pdf](docs/A1_2/A1_2_indoor_misia.pdf) | [docs/A1_2/A1_2_indoor_mission_EN.pdf](docs/A1_2/A1_2_indoor_mission_EN.pdf) |
| A1.3 | `docs/A1_3/` on branch `A1_3` | |

Open a PDF: `xdg-open docs/A1_2/A1_2_indoor_misia.pdf`
