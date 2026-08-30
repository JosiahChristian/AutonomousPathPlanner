# AutonomousPathPlanner

[![C++ CI](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/c-cpp.yml)
[![Pages](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/pages.yml/badge.svg)](https://github.com/JosiahChristian/AutonomousPathPlanner/actions/workflows/pages.yml)
[![C++20](https://img.shields.io/badge/C%2B%2B-20-00599C.svg)](https://isocpp.org/)

C++ autonomous-navigation software for deterministic waypoint generation, geometric collision checks, reactive obstacle avoidance, and independently gated motion commitment.

**[Launch the interactive planner visualizer](https://josiahchristian.github.io/AutonomousPathPlanner/)**

## What it demonstrates

AutonomousPathPlanner implements a compact perception-to-planning pipeline. It accepts a start pose, target pose, and obstacle map; evaluates each proposed motion step against a configurable safety radius; and applies perpendicular steering when the direct path is blocked.

The planner now also exposes an optional, replaceable `DecisionGate` between proposal and commitment. A proposed step can be independently accepted or vetoed before it becomes part of the executed trajectory. Gate disagreement produces an explicit abstention state instead of silently selecting another action.

This design is a research-informed engineering application of completed, bounded findings from the Adaptive-Model-Gating research program. It demonstrates the architectural principle of independent validation before commitment; it does not claim that experimental statistical guarantees automatically transfer to autonomous navigation, and it does not embed mechanisms whose evidence is still open.

The project is deliberately scoped as an engineering demonstration rather than a production navigation stack. It provides a testable foundation for comparing more advanced search, optimization, replanning, and gating strategies later.

## Capabilities

- Deterministic 2D waypoint generation
- Configurable step size, obstacle safety radius, and iteration limit
- Euclidean collision-risk evaluation
- Segment-level clearance checks that prevent waypoint tunneling
- Reactive evasive steering
- Two-sided detour evaluation with collision validation
- Collision validation on every segment, including final target approach
- Optional pluggable proposal-validation gate before motion commitment
- Explicit `GateVeto` abstention with no silent candidate reselection
- Gate-veto telemetry in native and JSON result contracts
- Rejection of non-finite scenario and configuration values
- Explicit success, occupied-endpoint, blocked, gate-veto, or iteration-limit termination state
- Path length, minimum-clearance, evasive-maneuver, and veto metrics
- Repeatable planning calls with no hidden state mutation
- Native C++ tests integrated with CTest
- Interactive HTML5 Canvas mission visualization

## Architecture

| Layer | Responsibility | Implementation |
|---|---|---|
| Scenario | Start, target, and detected obstacles | `src/main.cpp` |
| Planning | Trajectory generation and avoidance | `PlannerEngine` |
| Decision gating | Independent accept/veto boundary before proposal commitment | `DecisionGate` |
| Validation | Behavior, safety-invariant, and abstention checks | `tests/` |
| Visualization | Animated mission and telemetry view | `visualizer/` |

The browser experience is a visualization of the planner scenario. The C++ engine remains the source implementation of planning behavior; the current web layer does not execute the native binary in-browser.

## Build and run

Requirements: a C++20 compiler and CMake 3.22 or newer.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
./build/run_planner
```

With CMake and Ninja installed, the equivalent preset workflow is:

```bash
cmake --preset development
cmake --build --preset development
ctest --preset development
```

Machine-readable output for integrations is available with:

```bash
./build/run_planner --json
```

The checked-in `visualizer/scenario.json` is generated from this contract and loaded by the GitHub Pages interface. The embedded JavaScript scenario remains only as an offline fallback.

## Test

```bash
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The suite covers direct completion, single- and multi-obstacle avoidance, occupied endpoints, safety clearance, metric correctness, waypoint tunneling, final-approach validation, safe-side selection, blocked scenarios, invalid input, deterministic reuse, iteration-limit reporting, gate acceptance, explicit gate veto, and prevention of silent reselection after disagreement. GitHub Actions builds and runs these checks on every push and pull request to `main`.

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

A custom decision gate can be injected through the fourth constructor argument. The planner itself remains unaware of the gate's internal mechanism; it only observes `Accept` or `Veto`, keeping future validated gating approaches replaceable.

## Current limitations

- Two-dimensional point-obstacle model only
- Static obstacles and a single planning agent
- Reactive steering rather than globally optimal search
- No vehicle dynamics, uncertainty model, or kinodynamic constraints
- No claim that model-gating research guarantees transfer unchanged to navigation
- Browser deployment consumes a generated C++ scenario snapshot rather than executing native C++ live

These boundaries keep the repository honest and make the next engineering steps measurable.

## Roadmap

- Add concrete independently implemented gate policies behind the stable `DecisionGate` interface
- Introduce structured scenario input and dynamic scenario selection
- Compare A*, RRT/RRT*, and optimization-based strategies
- Measure path length, clearance, runtime, completion rate, and abstention behavior
- Connect generated planner output to the visualization layer
- Add dynamic-obstacle replanning and vehicle constraints

## Repository structure

```text
include/       Public planner and decision-gating interfaces
src/           Planning engine and executable demo
tests/         Native behavior and gating tests
visualizer/    GitHub Pages telemetry visualization
.github/       CI and Pages deployment workflows
```

See [Architecture](docs/architecture.md) for component boundaries, invariants, research-informed design limits, and extension points. Release-facing changes are tracked in the [changelog](CHANGELOG.md).

## License

This project is available under the [Apache License 2.0](LICENSE).

Contributions are welcome through the workflow in [CONTRIBUTING.md](CONTRIBUTING.md). Potential vulnerabilities should be reported according to [SECURITY.md](SECURITY.md).
