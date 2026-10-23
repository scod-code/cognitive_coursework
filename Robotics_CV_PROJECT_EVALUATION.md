# CV Project Evidence Sheet — Cognitive Robotics System

## Project identity

**ROS 2 Cognitive Robotics: Autonomous Perception, Navigation and Mapping System**
*Individual/coursework project, 2026*

Designed and integrated a simulated mobile-robot system that connects Gazebo, ROS 2 Humble, Nav2, YOLOv8, OctoMap and RViz. The robot navigates a mapped maze, recognises six visual classes, modifies its speed in response to traffic signs, builds a live 3D occupancy map, and can run a safety-aware autonomous survey.

## CV-ready version — recommended (three bullets)

- Engineered an end-to-end **ROS 2 Humble** cognitive-robotics stack integrating Gazebo simulation, **Nav2** (NavFn global planning and DWB control), OctoMap 3D occupancy mapping, RViz and custom Python nodes; delivered reproducible one-command manual and autonomous demo launches.
- Fine-tuned and deployed a **6-class YOLOv8n** perception model (fast/slow/stop signs, orange, tree and vehicle) on a 140-image dataset; achieved **0.845 mAP@50, 0.796 precision and 0.865 recall** after 100 CPU-trained epochs, with approximately **30 FPS** inference.
- Developed a safety-aware autonomous-mission supervisor that generates clearance-constrained candidate viewpoints from an occupancy map, ranks them using planned path length, confirms detections across multiple frames, handles stalls/timeouts/cancellations and records auditable JSON/CSV mission evidence; the saved live run completed **4 of 5** issued navigation goals before operator shutdown, and **103 Linux tests passed**.

## CV-ready version — compact (one bullet)

Built an integrated ROS 2 cognitive-robotics system in Gazebo using Nav2, YOLOv8 and OctoMap: delivered 6-class real-time perception (0.845 mAP@50), sign-aware velocity control, map-safe autonomous goal selection and logged failure recovery across 4 completed live navigation goals; verified the mission logic with 103 passing tests in Ubuntu/WSL.

## Portfolio / LinkedIn description

Built a full cognitive-robotics pipeline for a mobile robot operating in a simulated maze. I integrated perception, navigation, mapping and decision-making rather than treating them as separate demonstrations. A YOLOv8n detector identifies six semantic classes from the RGB-D camera; detection events update traffic-rule logic that publishes speed limits to Nav2. Nav2 plans and executes movement while OctoMap builds a live 3D occupancy representation for RViz.

I also implemented an autonomous survey layer with explicit safety and accountability mechanisms: footprint-aware candidate viewpoints are generated only in connected, free map space; paths are validated and ranked by planner distance; detection evidence requires three qualifying frames; and stalls, timeouts, rejected goals, TF/clock loss and competing goal sources have defined handling paths. Each run emits structured CSV event logs and a JSON outcome. Model evaluation achieved 0.845 mAP@50, 0.796 precision and 0.865 recall across six classes. In the recorded system run, 4 of 5 navigation goals succeeded, with no stalls or unplanned interventions before the operator ended the run. In Ubuntu 22.04/WSL, the three core ROS packages built successfully and all 103 ROS-free autonomous-mission tests passed.

## 30-second interview explanation

“I built an ROS 2 autonomous-robotics system that closes the loop from camera perception to robot behaviour. A fine-tuned six-class YOLOv8 model detects traffic signs and landmarks; traffic signs change Nav2’s permitted speed, while Nav2 and OctoMap provide planning, control and 3D mapping. I added a mission supervisor that chooses collision-safe viewpoints from the map, validates planned paths, requires repeated visual evidence, and fails safely when inputs or navigation are unhealthy. The deployed detector achieved 0.845 mAP@50; the recorded autonomous run completed four navigation goals without a stall; and 103 autonomous-mission tests passed in Ubuntu.”

## Evidence-backed technical scope

| Area | Demonstrable contribution |
| --- | --- |
| Robotics middleware | ROS 2 Humble packages, launch files, custom messages (`Detection`, `DetectionArray`) and `StoreLandmark` service; documented topic/frame contract. |
| Simulation and control | Gazebo cwmaze/Atlas robot, Nav2 map server, NavFn planner, DWB controller and `/atlas/cmd_vel` integration. |
| Computer vision | YOLOv8n inference node produces typed and JSON detections plus annotated images; 6-class detector trained at 640×640 for 100 epochs. |
| Behaviour layer | Sign classes map to absolute Nav2 speed limits: NORMAL 0.18 m/s, FAST 0.28 m/s, SLOW 0.05 m/s and STOP 0.01 m/s. |
| Mapping | RGB-D point cloud filtering and OctoMap occupancy updates, with RViz visualisation. |
| Autonomous reasoning | Candidate generation honours robot radius (0.18 m), safety margin (0.17 m), map connectivity, unknown-space exclusion and sign-proximity/facing exclusions; paths are ranked by planned length. |
| Robustness | Readiness checks cover clock, TF, map, pose, Nav2 servers/lifecycle, perception model/messages and competing command sources. Recovery covers rejected/aborted/cancelled goals, planner faults, stalls, timeouts, clock resets and input loss. |
| Verification | 4 dedicated ROS-free mission test modules cover state transitions, goal selection, candidate generation and mission metrics; **103 tests passed** in Ubuntu 22.04/WSL in 2.60 s. The three core ROS packages also built successfully in 23.8 s. |
| Delivery | 16 launch files, 3 ROS packages, saved RViz configurations, config assets, interface documentation and single-command demos. |

## Quantified outcomes (and the precise claim each supports)

| Metric | Value | Safe wording |
| --- | ---: | --- |
| Model classes | 6 | “Deployed six-class visual detection.” |
| Dataset split | 107 train / 22 validation / 11 test images | “Trained on a 140-image labelled dataset.” |
| Detection mAP@50 | 0.845 | “Achieved 0.845 mAP@50.” |
| Precision / recall | 0.796 / 0.865 | “Achieved 79.6% precision and 86.5% recall.” |
| Inference | ~30 FPS / ~33 ms per image on CPU | “Supported real-time CPU inference.” |
| Navigation run | 4 successful goals from 5 issued | “Completed 4 recorded navigation goals before an operator-ended run.” |
| Candidate survey | 40 safe locations / 160 poses generated | “Generated 160 orientation-specific viewpoints from 40 safe locations.” |
| Evidence confirmation | 3 qualifying frames in 3 s at ≥0.45 confidence | “Used temporal confirmation rather than a single detection.” |
| Ubuntu verification | 103/103 ROS-free tests passed in 2.60 s; 3 core packages built in 23.8 s | “Verified mission logic and ROS-package builds in Ubuntu 22.04/WSL.” |
| Codebase scope | 3 ROS packages, 16 launch files, 63 Python/launch/message/service source files, ~8,336 source lines | “Delivered a multi-package ROS 2 application with reproducible orchestration.” |

## Evidence locations

- Model metrics and limitations: `evidence/training_results_summary.md`, `evidence/results.csv`.
- Recorded autonomous mission result: `evidence/autonomy_run_20261010/mission.json` and `events.csv`.
- System interfaces: `INTERFACES.md`.
- Reproduction/demo instructions: `README.md`, `DEMO_INSTRUCTIONS.md` and `scripts/run_autonomy.sh`.
- Mission logic and tests: `wall_follower/wall_follower/mission_core.py`, `candidate_generator.py`, `goal_selection.py`, `mission_metrics.py`, and `wall_follower/test/`.
- Linux verification procedure and results: `DEMO_INSTRUCTIONS.md` (Ubuntu/WSL verification status).

## Professional evaluation

### Strengths worth advertising

1. **Systems integration, not a narrow model demo.** The project demonstrates interfaces across simulation, vision, mapping, planning, control and observability.
2. **Good engineering judgement around failure.** The autonomous layer includes readiness gates, sole-goal ownership, temporal evidence, safety clearance, cancellation, recovery and machine-readable logs.
3. **Measured perception result.** Use the detector metrics prominently; they are the best-established numerical outcome in the repository.
4. **Reproducibility and communication.** One-command launches, topic contracts and demo/submission documentation make the work easy to assess and extend.

### Qualifications to retain for credibility

- This is a **simulated** Gazebo project, not a physical-robot deployment.
- The final detection dataset is small (140 images) and synthetic/simulation-originated; do not claim real-world validation or production robustness.
- The saved autonomous run was **operator-aborted**, not a complete autonomous mission. It supports “4 successful goals, no stalls/interventions before shutdown,” not “mission completed successfully.”
- Navigation uses ground-truth odometry for localisation rather than AMCL/SLAM localisation. Describe it accurately as mapped autonomous navigation in simulation.
- The tested Ubuntu 22.04/WSL environment successfully executed all 103 ROS-free tests and built the three repository-owned core packages. The **full Gazebo system was not launch-tested in that environment** because the externally supplied `ntu_robotsim` package was not checked out/installed there; do not claim an end-to-end WSL demo launch from this verification.
- The Ubuntu test run produced one non-failing SciPy/NumPy compatibility warning (SciPy 1.8.0 expects NumPy `<1.25`, installed NumPy was 1.26.4). Align versions before presenting the environment as warning-free CI.

## Skills keywords

ROS 2 Humble · Python · Gazebo · Nav2 · NavFn · DWB · YOLOv8 · Ultralytics · OpenCV · OctoMap · RViz · RGB-D perception · occupancy grids · path planning · autonomous systems · state machines · sensor fusion interfaces · safety-aware navigation · pytest · JSON/CSV observability · Linux/colcon

