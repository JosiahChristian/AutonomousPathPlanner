# AutonomousPathPlanner

C++ autonomous navigation software for geometric collision evaluation, obstacle avoidance, and dynamic trajectory modification.

## Overview

AutonomousPathPlanner implements a compact perception-to-planning workflow for autonomous navigation. The planner ingests obstacle coordinates, evaluates the proposed path for collision risk, and modifies its trajectory when the direct route intersects an obstacle safety region.

The native C++ planner remains separate from the browser-based visualization layer so algorithmic logic and presentation can evolve independently.

## Core Capabilities

- Obstacle-map ingestion
- Geometric collision-risk evaluation
- Direct trajectory generation
- Dynamic evasive steering
- Configurable obstacle safety radius
- Modular C++ planning components
- CMake-based builds
- Interactive browser visualization

## Architecture

Environment / Obstacles
          |
     Obstacle Map
          |
    Planner Engine
          |
 Collision Evaluation
          |
 Trajectory Adjustment
          |
     Safe Path
          |
    Visualization

## Technology

**Core:** C++  
**Build:** CMake  
**Algorithms:** Euclidean geometry, collision evaluation, trajectory planning  
**Visualization:** JavaScript, HTML5 Canvas  
**Domain:** autonomous systems, robotics, navigation

## Planner Behavior

The planner begins with a direct step toward the target. Before accepting each step, it checks the proposed coordinate against the configured obstacle safety radius.

If the direct step creates a collision risk, the planner applies a perpendicular steering adjustment and continues trajectory generation from the modified position.

## Build

    cmake -S . -B build
    cmake --build build

## Design Priorities

The project keeps obstacle representation, collision reasoning, planning behavior, and visualization conceptually separated.

Continued engineering work will prioritize deterministic planning tests, edge-case handling, stronger trajectory validation, performance measurement, and richer planning strategies before expanding the interface surface.

## Live Visualization

[Launch the AutonomousPathPlanner visualizer](https://josiahchristian.github.io/AutonomousPathPlanner/)

The browser visualizer provides an interactive view of planner behavior while the C++ implementation remains the primary planning engine.