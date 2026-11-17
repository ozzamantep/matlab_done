# Underwater Drone Hide and Seek — MATLAB & Simulink + UE5/ROS 2

An autonomous multi-AUV **Hide-and-Seek** simulation framework developed for the **[MathWorks MATLAB & Simulink Challenge Project Hub (Project 27)](https://github.com/mathworks/MATLAB-Simulink-Challenge-Project-Hub/tree/main/projects/Underwater%20Drone%20Hide%20and%20Seek)** and the **[MathWorks Global Student Challenge](https://www.mathworks.com/academia/students/competitions/student-challenge/global-student-challenge.html)**.

The project combines a **ready-to-run MATLAB & Simulink R2025b+ simulation suite** (6-DOF Fossen hydrodynamics, forward-looking acoustic sonar, 3D EKF target tracking, Search/Track/Intercept state machine, APF obstacle avoidance, and Hider evasion AI) with a **ROS 2 + Unreal Engine 5** 3D digital-twin stack.

---

## Key Capabilities & Technical Stack

| Domain | Implementation |
|---|---|
| **6-DOF AUV Dynamics** | Fossen marine craft model $M\dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau + \tau_{\text{current}}$ with Archimedes buoyancy, quadratic drag, and 3D ocean currents |
| **Sensor Suite** | Forward-looking 3D acoustic sonar (occlusion + noise), hydrostatic depth/pressure, 9-DOF IMU, 4-beam DVL |
| **Perception & Tracking** | 6-state 3D Extended Kalman Filter (`EKFTargetTracker`) with covariance propagation during acoustic occlusions |
| **Autonomous Guidance & Control** | 6-DOF PID controller, 3D Lawnmower/Spiral Search, Lead-Pursuit Intercept, and Artificial Potential Field (APF) obstacle avoidance |
| **Adversarial Hider AI** | Autonomous evasion strategy using obstacle cover, tangential weaving, and arena boundary repulsion |
| **Simulink Model** | Closed-loop `UnderwaterHideAndSeek.slx` with Seeker, Hider, Sonar+EKF, and Capture Monitor blocks |
| **UE5 + ROS 2 Stack** | `UnrealProject 5.8` C++ simulation + `ROS2/underwater_ws` (`underwater_perception`, `underwater_navigation`, `underwater_interfaces`, `underwater_bringup`) |

---

## Repository Structure

```text
├── MATLAB/
│   ├── +auv/                   # BlueROV2Params.m, FossenDynamics.m (6-DOF RK4)
│   ├── +sim/                   # UnderwaterArena.m (terrain, obstacles, ocean currents)
│   ├── +sensors/               # SonarModel.m, DepthIMUDVL.m
│   ├── +perception/            # EKFTargetTracker.m
│   ├── +navigation/            # PIDController6DOF.m, SeekerAI.m, HiderAI.m
│   ├── run_hide_and_seek.m     # Interactive 3D simulation & live telemetry
│   ├── run_benchmark.m         # Monte Carlo evaluation & summary plots
│   ├── build_simulink_model.m  # Programmatic generator for UnderwaterHideAndSeek.slx
│   └── UnderwaterHideAndSeek.slx
├── ROS2/underwater_ws/src/
│   ├── underwater_interfaces/  # SonarScan.msg, SonarPing.msg, DroneState.msg, ResetMatch.srv
│   ├── underwater_perception/  # sonar_target_tracker (EKF + Search/Track/Intercept)
│   ├── underwater_navigation/  # auv_controller (6-DOF + APF) & hider_evasion_node
│   └── underwater_bringup/     # simulation.launch.py
├── UnrealProject 5.8/          # Unreal Engine 5.8 C++ underwater simulator
└── docs/                       # Architecture, installation, communication, testing guides
```

---

## Quick Start (MATLAB & Simulink)

Open MATLAB (R2025b or newer) in the `MATLAB/` folder:

1. **Run the Interactive 3D Hide-and-Seek Simulation:**
   ```matlab
   cd MATLAB
   results = run_hide_and_seek();
   ```
2. **Run the Monte Carlo Benchmark (5+ randomized trials):**
   ```matlab
   summary = run_benchmark(5);
   ```
   Generates `MATLAB/results/benchmark_summary.png` with distance profile, 3D intercept trajectory, Capture Rate, Mean Time-to-Capture, and EKF Position RMSE.
3. **Open & Run the Simulink Model:**
   ```matlab
   build_simulink_model();      % Rebuilds UnderwaterHideAndSeek.slx if needed
   out = sim('UnderwaterHideAndSeek');
   ```

---

## Benchmark Results (5 Monte Carlo Trials, MATLAB R2025b)

- **Seeker Capture Rate:** `100.0%`
- **Mean Time-to-Capture:** `103.28 s`
- **Mean EKF Position RMSE:** `0.31 m`
- **Mean Thruster Effort:** `34.67 N`

---

## Quick Start (ROS 2 + Unreal Engine 5)

1. Open `UnrealProject 5.8/UnderwaterSim.uproject` in Unreal Engine 5.8 and load `UnderwaterArena`.
2. Build and launch the ROS 2 stack (ROS 2 Jazzy / Humble):
   ```bash
   cd ROS2/underwater_ws
   colcon build
   source install/setup.bash
   ros2 launch underwater_bringup simulation.launch.py
   ```
3. See [docs/](docs/) for full setup, topic mapping, and testing instructions.
