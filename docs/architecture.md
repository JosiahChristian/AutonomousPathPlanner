# Architecture

## System boundary

AutonomousPathPlanner is a deterministic two-dimensional reactive planner. It accepts scenario state, produces a finite sequence of waypoints, reports why planning terminated, and calculates basic trajectory metrics. It does not model vehicle dynamics or claim globally optimal paths.

The planner also exposes an optional `DecisionGate` boundary between proposal and commitment. This is an engineering application of a completed, bounded finding from the Adaptive-Model-Gating research program: independently evaluated agreement can be used as a veto before an operational decision is committed. The software does **not** claim that the statistical guarantees observed in that research automatically transfer to autonomous navigation.

## Processing flow

1. The caller supplies start and target positions plus planner configuration and, optionally, a `DecisionGate`.
2. `ingestObstacleMap` validates and replaces the current static obstacle map.
3. `calculateSafeTrajectory` proposes a direct step toward the target.
4. Segment-level clearance is measured against every obstacle.
5. If blocked, left and right perpendicular detours are validated.
6. The safe detour closest to the target is selected; if neither is safe, planning terminates with `NoSafeStep`.
7. Before the proposed step is committed, the optional decision gate independently evaluates the proposal.
8. Gate disagreement produces `GateVeto`; the proposed motion is not appended and the planner does not silently reselect another candidate.
9. Accepted steps are committed and the engine records path length, minimum clearance, maneuver count, and gate-veto telemetry.
10. Planning stops on target arrival, an unsafe local configuration, gate veto, or the iteration limit.

## Components

| Component | Role |
|---|---|
| `PlannerEngine` | Validates inputs and performs deterministic trajectory generation |
| `DecisionGate` | Replaceable proposal-validation boundary that can accept or veto before commitment |
| `PlanResult` | Carries trajectory, termination reason, and metrics, including gate vetoes |
| `run_planner` | Demonstrates human-readable and JSON output contracts |
| `planner_tests` | Exercises nominal, adversarial, boundary, and repeatability cases |
| `decision_gate_tests` | Verifies accepted proposals preserve behavior and vetoed proposals are withheld without reselection |
| `visualizer/` | Animates a generated planner-result snapshot on GitHub Pages |

## Safety invariants

- Every accepted segment must meet the configured obstacle safety radius.
- Occupied start and target positions produce distinct termination states.
- An unsafe waypoint is never appended to the result.
- A blocked local configuration is reported explicitly rather than represented as success.
- A gate-vetoed proposal is never appended to the trajectory.
- Gate disagreement stops the current decision path rather than silently choosing a different candidate.
- Non-finite positions and configuration values are rejected before planning.
- The iteration limit bounds runtime for scenarios that do not converge.

These are software invariants for the implemented geometric model, not certification claims for a physical vehicle.

## Research-informed design boundary

The gating interface intentionally separates a durable engineering principle from any one research mechanism. The core planner knows only that a proposal can be independently accepted or vetoed. A future validated mechanism can therefore replace or extend a gate implementation without rewriting trajectory generation or weakening the baseline planner.

This repository currently demonstrates the architecture, abstention behavior, telemetry, and testability of that principle. It does not embed the still-evaluating Experiment 065 M0 cross-architecture consensus mechanism, and it does not generalize model-gating robustness claims beyond the completed evidence boundary.

## Data contract

`run_planner --json` emits status, metrics, obstacles, and trajectory coordinates. Gate telemetry is included in the metrics contract. CI regenerates this document and compares it with `visualizer/scenario.json`, preventing the deployed demonstration from drifting away from native engine output.

## Extension points

The API leaves room for alternate planning policies, structured scenario input, dynamic obstacle maps, vehicle constraints, benchmark harnesses, and additional `DecisionGate` implementations. Those capabilities should be added behind explicit interfaces so the tested baseline remains available for comparison.
