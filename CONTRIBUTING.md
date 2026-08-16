# Contributing

Contributions should preserve the planner's deterministic behavior, explicit safety boundaries, and separation between the native engine and browser visualization.

## Development workflow

1. Create a focused branch from `main`.
2. Configure a test build with `cmake -S . -B build -DBUILD_TESTING=ON`.
3. Build with `cmake --build build --parallel`.
4. Run `ctest --test-dir build --output-on-failure`.
5. If planner output changes, regenerate the visualizer snapshot with `./build/run_planner --json > visualizer/scenario.json`.
6. Open a pull request describing the behavioral change and its validation.

## Expectations

- Add regression coverage for planning or safety changes.
- Compile cleanly under the repository warning policy.
- Keep public interfaces small and documented.
- Do not describe the reactive planner as globally optimal or production-certified.
- Keep generated scenario data synchronized with the C++ output contract.

## Reporting defects

Include the start, target, configuration, obstacle coordinates, actual termination state, and expected behavior. Security-sensitive reports should follow `SECURITY.md` instead of a public issue.
