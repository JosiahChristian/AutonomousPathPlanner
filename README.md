# AutonomousPathPlanner

[![C++ CI](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/c-cpp.yml)
[![Pages](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/pages.yml/badge.svg)](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/pages.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

C++ autonomous-navigation software for deterministic waypoint generation, geometric collision checks, and reactive obstacle avoidance.

**[Launch the interactive planner visualizer](https://josiahchristian.github.io/AutonomousPathPlanner/)**

## What it demonstrates

AutonomousPathPlanner implements a compact perception-to-planning pipeline. It accepts a start pose, target pose, and obstacle map; evaluates each proposed motion step against a configurable safety radius; and applies perpendicular steering when the direct path is blocked.

The project is deliberately scoped as an engineering demonstration rather than a production navigation stack. It provides a testable foundation for comparing more advanced search, optimization, and replanning strategies later.

## Capabilities

- Deterministic 2D waypoint generation
- Configurable step size, obstacle safety radius, and iteration limit
- Euclidean collision-risk evaluation
- Segment-level clearance checks that prevent waypoint tunneling
- Reactive evasive steering
- Explicit target-reached or iteration-limit termination state
- Repeatable planning calls with no hidden state mutation
- Native C++ tests integrated with CTest
- Interactive HTML5 Canvas mission visualization

## Architecture

| Layer | Responsibility | Implementation |
|---|---|---|
| Scenario | Start, target, and detected obstacles | `src/main.cpp` |
| Planning | Trajectory generation and avoidance | `PlannerEngine` |
| Validation | Behavior and safety-invariant checks | `tests/PlannerEngineTests.cpp` |
| Visualization | Animated mission and telemetry view | `visualizer/` |

The browser experience is a visualization of the planner scenario. The C++ engine remains the source implementation of planning behavior; the current web layer does not execute the native binary in-browser.

## Build and run

Requirements: a C++20 compiler and CMake 3.22 or newer.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/run_planner
```

## Test

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The suite covers direct completion, obstacle avoidance, safety clearance, deterministic reuse, and iteration-limit reporting. GitHub Actions builds and runs these checks on every push and pull request to `main`.

## Configuration example

```cpp
PlannerConfig config{
    .stepSize = 0.5,
    .safetyRadius = 1.5,
    .maxIterations = 200
};

PlannerEngine planner({0.0, 0.0}, {0.0, 10.0}, config);
planner.ingestObstacleMap({{0.0, 4.0}});
const PlanResult plan = planner.calculateSafeTrajectory();
```

## Current limitations

- Two-dimensional point-obstacle model only
- Static obstacles and a single planning agent
- Reactive steering rather than globally optimal search
- No vehicle dynamics, uncertainty model, or kinodynamic constraints
- Visualization uses a representative fixed scenario rather than live C++ output

These boundaries keep the repository honest and make the next engineering steps measurable.

## Roadmap

- Add multiple-obstacle regression cases and clearance metrics
- Introduce structured scenario input and trajectory output
- Compare A*, RRT/RRT*, and optimization-based strategies
- Measure path length, clearance, runtime, and completion rate
- Connect generated planner output to the visualization layer
- Add dynamic-obstacle replanning and vehicle constraints

## Repository structure

```text
include/       Public planner interface
src/           Planning engine and executable demo
tests/         Native behavior tests
visualizer/    GitHub Pages telemetry visualization
.github/       CI and Pages deployment workflows
```

## License

This project is available under the [MIT License](LICENSE).
