# How to run

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

A1.1 with another route or radius: `run_a1_1.sh --start 12 4 2 --goal 16 8 2 --safety-radius 0.45`.

## Graphs and visualisations

```bash
cd ~/Zadania/LRS_Psenak_Valko

# A1.2 plots (top view + overview) of the recorded mission, or of any run in log/a1_2_runs/
python3 plot_a1_2.py results/hangar_example_20261018

# A1.2 interactive 3D view of the same run
python3 view_3d_a1_2.py results/hangar_example_20261018

# RViz with map, racks and planned path (planner must be running)
source install/setup.bash && rviz2 -d src/lrs_psenak_valko/config/planner.rviz

# Study guide A1.1–A1.3 (browser)
xdg-open docs/A1_study_guide.html
```

## Documentation

- A1.1: [docs/A1_1/A1_1_mapa_a_planovanie.md](docs/A1_1/A1_1_mapa_a_planovanie.md) (+ PDF)
- A1.2: [docs/A1_2/A1_2_indoor_misia.md](docs/A1_2/A1_2_indoor_misia.md) (+ PDF)
- A1.3: `docs/A1_3/` on branch `A1_3`
