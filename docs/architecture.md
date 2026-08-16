# Architecture

## System boundary

AutonomousPathPlanner is a deterministic two-dimensional reactive planner. It accepts scenario state, produces a finite sequence of waypoints, reports why planning terminated, and calculates basic trajectory metrics. It does not model vehicle dynamics or claim globally optimal paths.

## Processing flow

1. The caller supplies start and target positions plus planner configuration.
2. `ingestObstacleMap` validates and replaces the current static obstacle map.
3. `calculateSafeTrajectory` proposes a direct step toward the target.
4. Segment-level clearance is measured against every obstacle.
5. If blocked, left and right perpendicular detours are validated.
6. The safe detour closest to the target is selected; if neither is safe, planning terminates with `NoSafeStep`.
7. The engine records the waypoint, path length, minimum clearance, and maneuver count.
8. Planning stops on target arrival, an unsafe local configuration, or the iteration limit.

## Components

| Component | Role |
|---|---|
| `PlannerEngine` | Validates inputs and performs deterministic trajectory generation |
| `PlanResult` | Carries trajectory, termination reason, and metrics |
| `run_planner` | Demonstrates human-readable and JSON output contracts |
| `planner_tests` | Exercises nominal, adversarial, boundary, and repeatability cases |
| `visualizer/` | Animates a generated planner-result snapshot on GitHub Pages |

## Safety invariants

- Every accepted segment must meet the configured obstacle safety radius.
- Occupied start and target positions produce distinct termination states.
- An unsafe waypoint is never appended to the result.
- A blocked local configuration is reported explicitly rather than represented as success.
- Non-finite positions and configuration values are rejected before planning.
- The iteration limit bounds runtime for scenarios that do not converge.

These are software invariants for the implemented geometric model, not certification claims for a physical vehicle.

## Data contract

`run_planner --json` emits status, metrics, obstacles, and trajectory coordinates. CI regenerates this document and compares it with `visualizer/scenario.json`, preventing the deployed demonstration from drifting away from native engine output.

## Extension points

The current API leaves room for alternate planning policies, structured scenario input, dynamic obstacle maps, vehicle constraints, and benchmark harnesses. Those capabilities should be added behind explicit interfaces so the tested baseline remains available for comparison.
