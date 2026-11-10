# LRS — Psenak, Valko: A2 Drone localisation with a simplified Kalman filter

An eight-state EKF fuses noisy GPS, drifting visual odometry and a biased yaw-rate gyro. The
A2 route is flown with **only this estimate** as feedback.
Assignment: [LRS-URK `assignments/assignment2/localization.md`](https://github.com/KocurMaros/LRS-URK/blob/main/assignments/assignment2/localization.md).
Detailed documentation (Slovak, team format: what / why / pros and cons, diagrams, results):
[docs/A2/A2_lokalizacia.md](docs/A2/A2_lokalizacia.md).

Requirements: ROS 2 Jazzy (or Humble), MAVROS, Eigen 3, `ros_gz_bridge`, Gazebo Harmonic, ArduPilot SITL
and the course repo [LRS-URK](https://github.com/KocurMaros/LRS-URK) (`LRS-URK` or `LRS-URK-main` next to this
repository or in `~`, `~/Desktop/LRS`; or set `LRS_URK_DIR`).
The script builds our package **and** the teacher's `localization_sensor_sim` into this workspace,
so `~/LRS-URK` stays unchanged.

## Run

One script starts everything at once (like the A1 scripts):

```bash
./run_assignment2.sh                              # windows: Gazebo GUI, SITL, MAVROS, sensors, estimator + mission (seed 42)
./run_assignment2.sh --seed 73 --record --video   # + bag, plots, summary and a2_flight.mp4 in results/A2/seed73_<date>/
./run_assignment2.sh --record --scenario          # + tools/a2_scenario.py: GPS off 15 s (middle leg), outlier, VO off 3 s
./run_assignment2.sh --record --scenario-random 7 # the same events in legs/times drawn from seed 7
./run_assignment2.sh --headless --record --scenario   # no windows, waits for the landing, closes everything
./run_assignment2.sh --characterize 30 --seed 73  # §1: drone on the ground, 30 s of sensors (no SITL/MAVROS/mission)
./run_assignment2.sh --offline                    # no simulator: unit test + offline filter test on 20 seeds
./run_assignment2.sh --all                        # the whole A2 headless (~40 min): build, test, offline,
                                                  # 3x characterisation, flights on 4 seeds + 1 random scenario,
                                                  # plots, video, results table into docs/A2/A2_lokalizacia.md
```

The same by hand (what the script does), each line in its own terminal, from `~/LRS-URK` and this workspace:

```bash
GZ_SIM_RESOURCE_PATH="$PWD/models:${GZ_SIM_RESOURCE_PATH:-}" gz sim -r -v 4 --seed 42 "$PWD/worlds/fei_lrs_gazebo_localization.world"
scripts/run_sitl.sh
scripts/run_mavros.sh
colcon build --base-paths src ~/LRS-URK/localization_sensor_sim && source install/setup.bash
ros2 launch localization_sensor_sim localization_sensors.launch.py seed:=42
ros2 launch lrs_psenak_valko a2_localization.launch.py            # mission:=false = estimator only
```

The teacher's tests (any time during the flight):

```bash
ros2 service call /localization/set_gps_enabled std_srvs/srv/SetBool "{data: false}"   # ... "{data: true}" after 15 s
ros2 service call /localization/set_vo_enabled std_srvs/srv/SetBool "{data: false}"
ros2 service call /localization/inject_gps_outlier std_srvs/srv/Trigger "{}"
ros2 service call /localization_mission/abort std_srvs/srv/Trigger                     # our abort -> LAND
```

Plots of a recorded flight: `python3 tools/a2_plot.py results/A2/<run>/bag`, video: `python3 tools/a2_video.py
results/A2/<run>/bag`, results table: `python3 tools/a2_report.py results/A2/flight_* [--doc docs/A2/A2_lokalizacia.md]`.
Sensor characterisation: `python3 tools/a2_characterize.py <bag> [<bag> ...]`.

## Estimator output

| | |
|---|---|
| **Topic** | `/estimator/odometry` (`nav_msgs/msg/Odometry`, 100 Hz, stamped with the IMU sample time) |
| **Frame** | `header.frame_id = map` (MAVROS-compatible ENU, the frame of `/localization/gps`), `child_frame_id = base_link` |
| Pose | `x, y, z` and yaw as a yaw-only quaternion; covariance from `P` (roll and pitch are not estimated: variance 1e6) |
| Twist | velocity in `base_link` (the `nav_msgs/Odometry` convention), `angular.z = w_z − b_g` |
| Diagnostics | `/estimator/debug` (state + σ, 17 values, labels in `layout`), `/estimator/innovation` (every update: sensor, status, d², gate, r), `/estimator/events` (LOST / BACK / REJECTED, latched) |

**Inputs: only** `/localization/gps`, `/localization/vo`, `/localization/imu` (sensor-data QoS).
The controller (`localization_mission`) subscribes to `/estimator/odometry` and `/mavros/state`
only. It publishes velocity + yaw-rate setpoints to `/mavros/setpoint_raw/local` and uses the
MAVROS mode/arming/takeoff services. Ground truth is read only by the offline tools in `tools/`.
Check: `ros2 node info /localization_estimator`, `ros2 node info /localization_mission`.

## Filter (ekf8.h)

State, in the required order, frame `map`:  **x = [x, y, z, vx, vy, vz, yaw, b_g]ᵀ**

**Prediction** on every IMU sample, `dt` from the header stamps (zero-order hold of the last gyro rate):

```
p   ← p + v·dt
v   ← v
yaw ← wrap(yaw + (w_z − b_g)·dt)                  wrap to [−π, π)
b_g ← b_g
P   ← F P Fᵀ + Q(dt)        F = I + dt·(∂p/∂v) − dt·(∂yaw/∂b_g)   (the model is linear)
Q   per axis: σa²·[dt³/3 dt²/2; dt²/2 dt] (p, v);   yaw: σw²·dt + σb²·dt³/3,  (yaw, b_g): −σb²·dt²/2,  b_g: σb²·dt
```

**Updates** are asynchronous: each message first brings the state to its own stamp. A stamp up to
`max_measurement_lag_s` older than the state is applied at the state time; older ones and
duplicates are dropped and counted. For every update: `r = z − h(x)`, `S = H P Hᵀ + R`,
`d² = rᵀ S⁻¹ r`, `K = P Hᵀ S⁻¹` (LDLT solve, no inverse), Joseph form
`P = (I−KH) P (I−KH)ᵀ + K R Kᵀ`, symmetrised. An update that would make `x` or `P` non-finite
is skipped and the previous state is kept.

| Measurement | h(x) | H | Notes |
|---|---|---|---|
| GPS position (5 Hz) | `p` | `[I₃ 0 0 0]` | χ² gate on d² (3 DOF) |
| VO yaw (30 Hz) | `yaw` | `e_yawᵀ` | innovation wrapped to [−π, π); the only absolute yaw source → makes `b_g` observable |
| VO body velocity (30 Hz) | `Rz(yaw)ᵀ v` | `∂h/∂v = Rz(yaw)ᵀ`, `∂h/∂yaw = [−s·vx + c·vy, −c·vx − s·vy, 0]ᵀ` | **genuine EKF measurement**; `vo.velocity_model: rotate` switches to the linear-KF approximation (`z` rotated with the yaw estimate, `H = [0 I₃ 0 0]`) |
| gyro at rest (100 Hz, **only before takeoff**) | `b_g` | `e_bgᵀ` | static alignment (zero angular rate update): standing still, the gyro sample *is* the bias. Ends for good on the first motion (low-passed VO speed > 0.15 m/s, or 3 gyro samples in a row outside the χ² gate 10.83 = real rotation) |

The VO **pose position is not used**. `vo_odom` drifts away from `map`, and without an offset
state an absolute VO position would pull the estimate along with the drift. The body velocity
carries the same short-term information without accumulating it.
**Gyro-bias observability:** the gyro integrates `w_z − b_g`, and VO yaw measures the yaw itself.
Any bias error makes the predicted yaw diverge linearly from the VO yaw, and the cross-covariance
`P(yaw, b_g)` (built by `F(yaw, b_g) = −dt`) turns those yaw innovations into a bias correction.
VO yaw however drifts (up to ~3 °/min), and through this path the drift leaks into the bias. So,
before takeoff, the **static alignment** observes the bias directly (true yaw rate = 0): offline on 20
seeds the bias error drops from 0.0004 to 0.0001 rad/s, and with VO yaw as a weak anchor
(σ 0.5 rad) the yaw error is honest: 100 % inside ±3σ instead of 59 %.

**Initialisation:** position from the first GPS fix, yaw from the first VO yaw (`vo_odom` starts
aligned with `map`), velocity 0 and bias 0 with wide uncertainty. The truth topic is never used.

## Q, R, P0, gate (config/localization.yaml)

| | Parameter | Value | Why |
|---|---|---|---|
| Q | `process_noise.accel_xy` / `accel_z` | 0.2 / 0.3 m/s²/√Hz | constant-velocity model error; tuned offline (smooth vs. VO-loss error) |
| Q | `process_noise.yaw_rate` | 0.005 rad/s/√Hz | gyro white noise 4·10⁻⁴ rad/s/√Hz + tilt coupling and unmodelled rotation (yaw σ must stay honest) |
| Q | `process_noise.gyro_bias_rw` | 2·10⁻⁵ rad/s/√s | the bias is static (pinned by the static alignment); a small walk keeps it adjustable |
| R | `gps.position_std` | 0.6, 0.6, 0.9 m | the message covariance is used when valid (`gps.use_message_covariance`) |
| R | `vo.velocity_std` | 0.2 m/s | message says 0.03 m/s; the real VO error is a slowly drifting **bias** (~0.01 m/s), a white R must be inflated or the filter becomes overconfident |
| R | `vo.yaw_std` | 0.5 rad | message says 0.3°; VO yaw drifts up to ~3 °/min (correlated), so at 30 Hz it may only anchor yaw slowly; with 0.1 rad only 59 % of the yaw errors were inside ±3σ, with 0.5 rad 100 % |
| R | `static_alignment.rate_std` | 0.005 rad/s | zero-rate update before takeoff: gyro noise 0.004 rad/s + margin |
| P0 | `initial_std.*` | pos 0.6 / 0.6 / 0.9 m, vel 0.5 m/s, yaw 0.1 rad, b_g 0.05 rad/s | one GPS fix; drone may move; VO aligned at its start; \|b_g\| ≈ 0.02 of unknown sign |
| gate | `gps.gate_d2` | **16.27** | χ² quantile, 3 DOF, 99.9 %: a valid fix is rejected with probability 0.1 % |
| gate | `gate_max_consecutive_rejects` | 5 | lock-out guard: the 6th rejection in a row is accepted (the estimate, not GPS, is then the likely culprit) |
| gate | `vo.velocity_gate_d2`, `vo.yaw_gate_d2` | 0 (off) | optional (e.g. 16.27 / 10.83) |
| gate | `static_alignment.gate_d2` | 10.83 | χ² 1 DOF 99.9 %: a gyro residual above it means rotation, 3 in a row end the static alignment |

## Missing and bad measurements

* **GPS outage**: the prediction continues; VO velocity and the gyro carry the position. P grows
  and nothing is reset. When GPS returns, the larger P gives a larger but normal Kalman gain (no snap to the raw fix).
* **VO loss**: GPS and the constant-velocity model carry the estimate; yaw is gyro-integrated
  with the estimated bias. σ_xy grows (0.1 → ~0.3 m in 3 s), and the **mission** reacts: above
  `degraded_std_xy_m` 0.2 m the route slows down, above `hold_std_xy_m` 0.28 m the carrot stops and the
  feedback gain drops to 0.3·Kp, so the drone holds instead of chasing GPS noise (a 15 s GPS outage
  stays below 0.2 m and does not slow the flight).
* **Outlier**: rejected when `d² > 16.27`; S contains P, so the gate widens correctly after an outage.
* Logged once per transition, never at sensor rate: `GPS LOST`, `GPS BACK after …`, the first update after a gap,
  every rejection with its d², and a status line every 5 s (rates from header stamps, estimate, σ, counters).

## Files

| File | |
|---|---|
| `src/lrs_psenak_valko/src/ekf8.h` | the filter: state, prediction, measurement models, gate, sensor-fusion policy (Eigen, no ROS) |
| `src/lrs_psenak_valko/src/localization_estimator_node.cpp` | ROS node: subscriptions, frame/stamp checks, timeouts, logs, outputs |
| `src/lrs_psenak_valko/src/localization_mission_node.cpp` | mission on the estimate (velocity + yaw-rate setpoints) |
| `src/lrs_psenak_valko/src/ekf8_test.cpp` | unit test (`ros2 run lrs_psenak_valko ekf8_test`) |
| `src/lrs_psenak_valko/src/ekf8_replay.cpp`, `tools/a2_offline_sim.py` | offline: the teacher's sensor model → our filter, many seeds in seconds |
| `src/lrs_psenak_valko/config/localization.yaml`, `launch/a2_localization.launch.py` | parameters, launch |
| `tools/a2_plot.py`, `tools/a2_characterize.py`, `tools/a2_scenario.py` | plots vs. truth (+ `summary.json`), §1 characterisation, scripted teacher tests (fixed or `--random` legs/times) |
| `tools/a2_video.py`, `tools/a2_report.py` | mp4 of a flight from its bag (map, errors, sensor ON/OFF, events), results table into the docs |
| `run_assignment2.sh` | starts everything at once; `--all` runs the whole assignment end to end |
| `results/A2/` | recorded flights and the characterisation runs (plots, summaries, one video; bags and logs stay local, see `.gitignore`) |
