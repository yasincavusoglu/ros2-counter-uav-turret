# ros2-counter-uav-turret

A ROS 2 simulation of an **autonomous counter-UAS (anti-drone) turret**: it detects a maneuvering aerial
target, tracks it, estimates its state, computes a **lead (intercept) aim-point**, and drives a stabilized
pan/tilt gimbal onto it — with an engagement state machine and safety interlocks. End-to-end, closed-loop,
runs headless (no hardware required).

> Built as a clean-room, generic reimplementation of the perception-and-control core I designed and led as
> team captain on an autonomous turret project (field-tested across 70+ recorded runs). The original
> competition code is private; this repository is an independent, public implementation of the same engineering.

> 🚧 **Work in progress.** Node skeletons and architecture are in place; the pipeline is being filled in
> milestone by milestone (see [Roadmap](#roadmap)).

## What it demonstrates
- **Detection → tracking → estimation → lead → stabilized control** as a real ROS 2 node graph.
- A **lead-angle (intercept) solver** — engaging a *moving* target, not a static one.
- A 6-state Kalman filter with **adaptive process noise, chi-square innovation gating and NIS telemetry**
  (it rejects bad measurements and reports its own consistency).
- C++ acceleration of the estimation hot path via
  [perception-accel](https://github.com/yasincavusoglu/perception-accel).

## Architecture

```
scenario_sim --/camera/image--> detector --/detections--> tracker --/tracks--> estimator
   (target + synthetic camera)     (YOLO)      (ByteTrack)      (6D Kalman: adaptive-Q + chi2 gating)
        ^                                                                            |
        | /gimbal/state                                                     /target/state
        |                                                                            v
   turret_plant <--/gimbal/cmd-- gimbal_control <--/aim/setpoint-- lead_solver   (intercept aim-point)
   (2nd-order dynamics)           (PID + stabilization)                 ^
                                                                   engagement_fsm  (SEARCH->LOCK->ENGAGE + safety)
                                                                        |
                                                                  hmi_telemetry  (live plots, JSONL logs, demo recording)
```

| Node | Role | Status |
|------|------|--------|
| `scenario_sim`   | maneuvering aerial target + synthetic camera + ground truth | 🚧 |
| `detector`       | YOLOv8/v11 target detection | 🚧 |
| `tracker`        | ByteTrack multi-object tracking | 🚧 |
| `estimator`      | 6D Kalman (adaptive-Q, chi² gating, NIS) — C++ core | 🚧 |
| `lead_solver`    | intercept/lead aim-point for a moving target | 🚧 |
| `gimbal_control` | pan/tilt PID + stabilization (ros2_control) | 🚧 |
| `turret_plant`   | 2nd-order gimbal dynamics (sim) | 🚧 |
| `engagement_fsm` | SEARCH → TRACK_LOCK → ENGAGE + safety interlocks | 🚧 |
| `hmi_telemetry`  | live plots, JSONL logs, demo recording | 🚧 |

## The lead-angle solver (why it matters)
Static aim fails on a moving target. Given the estimated target state and the effector time-of-flight, the
solver iterates `t_impact = range / v_effector`, predicts the target forward by `t_impact`, and repeats until
convergence — producing an aim-point *ahead* of the target.

## Run
```bash
colcon build && source install/setup.bash
ros2 launch counter_uav_turret sim.launch.py             # closed-loop sim
ros2 launch counter_uav_turret sim.launch.py record:=true    # + demo.mp4
```
Requires: ROS 2 Humble, `ultralytics`, `opencv-python`, `numpy`. (Optional: RViz2 for a 3D view.)

## Results
_To be filled with measured numbers once the loop is closed — not placeholders:_
track-lock time, aim-error RMSE (mrad), loop latency (C++ vs Python), hit rate with vs without lead.

## Roadmap
- [ ] **M1 — Perception:** `scenario_sim` + `detector` + `tracker` (detection→track on synthetic video)
- [ ] **M2 — Closed loop:** `estimator` + `lead_solver` + `gimbal_control` + `turret_plant` (stabilized lock on a moving target)
- [ ] **M3 — Showcase:** `engagement_fsm` + safety + `hmi_telemetry` + demo video + measured metrics

## Design notes & honesty
- Simulation only; the effector/ballistics model is simplified, not a validated weapon model.
- Detection uses a pretrained YOLO model (weights not included in the repo).
- The actuator interface is `ros2_control`; the original hardware system used a Fatek PLC over Modbus TCP.

## License
MIT — see [LICENSE](LICENSE).
