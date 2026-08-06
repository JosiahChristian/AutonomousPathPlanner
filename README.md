# AutonomousPathPlanner: Algorithmic Obstacle Avoidance Engine

A high-performance computational autonomy pipeline implementing real-time obstacle avoidance and dynamic trajectory recalculation. This system models predictive vehicle kinematics across discrete coordinate spaces to navigate around incoming hazard grids safely.

Targeted for robotics and autonomous navigation research alignment with the **Old Dominion University (ODU) PhD in Modeling and Simulation Engineering** pipeline.

## 🤖 Autonomy Software Architecture

The software architecture enforces a clean, modular separation between the three fundamental stages of robotic autonomy:

1. **Perception Stage:** Ingests dynamic, low-latency array matrices simulating incoming LiDAR/Radar distance spatial sensor coordinates.
2. **Planning Stage:** Evaluates direct vector paths against a configurable localized safety bounding perimeter zone (\(r_{safety} = 1.5\text{m}\)). If a proximity conflict intersect is detected, the engine executes a 90° perpendicular matrix translation maneuver to bypass the obstacle.
3. **Control Stage:** Outputs the stabilized, updated waypoint trajectory map tracking parameters natively to the hardware execution register layer.

## 🛠️ Local Compilation & Deployment

The codebase is engineered using standard, modern C++ patterns, ensuring clean compilation across multiple operating system environments without external library dependencies.

### Prerequisites
* CMake 3.22+
* C++17 Compliant Compiler (GCC / MinGW / Clang)

### Execution Blueprint
Build and compile the native binary executables directly within your local PowerShell or terminal console workspace:

```bash
# 1. Configure the build environment using the local Make toolchain
cmake -B build -S . -G "MinGW Makefiles" -DCMAKE_CXX_COMPILER=g++ -DCMAKE_MAKE_PROGRAM=make

# 2. Compile the binaries to 100% completion
cmake --build build

# 3. Launch the autonomous autopilot routing application
.\build\run_planner.exe
```

## 📊 Expected Terminal Output Simulation Log
Upon execution, the terminal engine will dynamically track tracking state updates across spatial nodes:

```text
===============================================================
     ODU CYBER-PHYSICAL AUTONOMOUS NAVIGATION PIPELINE         
===============================================================
[PERCEPTION] Successfully mapped 1 dynamic radar obstacle coordinates.
[PLANNING] Computing safe vector trajectory coordinates...
[PLANNING] Collision hazard detected at (0, 3). Initiating evasive vector path translation!
[PLANNING] Collision hazard detected at (-0.875965, 2.99228). Initiating evasive vector path translation!

[CONTROL] Streaming Executed Trajectory Path Coordinates:
---------------------------------------------------------------
Waypoint 00 | Latitude Position Coordinate: (0, 0)
Waypoint 01 | Latitude Position Coordinate: (0, 1)
Waypoint 02 | Latitude Position Coordinate: (0, 2)
Waypoint 03 | Latitude Position Coordinate: (-1, 2)
Waypoint 04 | Latitude Position Coordinate: (-1.99, 2.12)
...
Waypoint 13 | Latitude Position Coordinate: (0, 10)
---------------------------------------------------------------
Target intercept accomplished safely. Mission lifecycle complete.
```

## 🛠️ Portfolio Mapping
This codebase anchors the robotics and control-systems track of a multi-domain software portfolio, demonstrating core software competencies in spatial geometry transforms, defensive pathing algorithms, and native C++ systems programming.
