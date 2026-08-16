# Changelog

All notable changes to this project are documented here.

## Unreleased

### Added

- Configurable planner parameters and explicit termination reasons
- Segment-level collision validation and two-sided evasive steering
- Path-length, minimum-clearance, and maneuver-count metrics
- C++ behavior, boundary, and regression tests
- Machine-readable planner output and generated visualizer scenario
- GCC, Clang, sanitizer, and scenario-synchronization CI checks
- Contribution, security, architecture, and reproducible-build documentation

### Changed

- Refactored the planner into a reusable CMake library
- Aligned the GitHub Pages scenario with current native planner behavior

### Fixed

- Prevented obstacle tunneling between widely spaced waypoints
- Prevented unchecked final target segments
- Prevented unsafe detour acceptance and hidden state mutation
