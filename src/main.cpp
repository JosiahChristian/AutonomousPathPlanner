#include "PlannerEngine.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <string_view>

namespace {
const char* terminationName(TerminationReason reason) {
    switch (reason) {
    case TerminationReason::TargetReached:
        return "target_reached";
    case TerminationReason::StartInCollision:
        return "start_in_collision";
    case TerminationReason::TargetInCollision:
        return "target_in_collision";
    case TerminationReason::NoSafeStep:
        return "no_safe_step";
    case TerminationReason::IterationLimitReached:
        return "iteration_limit_reached";
    }
    return "unknown";
}

void writeJson(const PlanResult& plan) {
    std::cout << std::setprecision(15)
              << "{\n  \"status\": \"" << terminationName(plan.terminationReason) << "\",\n"
              << "  \"metrics\": {\n"
              << "    \"pathLength\": " << plan.pathLength << ",\n"
              << "    \"minimumClearance\": ";
    if (std::isfinite(plan.minimumClearance)) {
        std::cout << plan.minimumClearance;
    } else {
        std::cout << "null";
    }
    std::cout << ",\n    \"evasiveManeuvers\": " << plan.evasiveManeuvers
              << "\n  },\n  \"obstacles\": [{\"x\": 0, \"y\": 4}],\n"
              << "  \"trajectory\": [\n";
    for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
        const auto& point = plan.trajectory[index];
        std::cout << "    {\"x\": " << point.x << ", \"y\": " << point.y << "}"
                  << (index + 1 < plan.trajectory.size() ? "," : "") << '\n';
    }
    std::cout << "  ]\n}\n";
}
}

int main(int argc, char* argv[]) {
    const bool jsonOutput = argc == 2 && std::string_view(argv[1]) == "--json";
    if (argc > 1 && !jsonOutput) {
        std::cerr << "Usage: run_planner [--json]\n";
        return 2;
    }

    // Initialize: Start at (0,0) trying to steer cleanly to target endpoint (0, 10)
    PlannerEngine autoPilot({0.0, 0.0}, {0.0, 10.0});
    std::vector<Position> lidarRadarFeeds = {{0.0, 4.0}};
    autoPilot.ingestObstacleMap(lidarRadarFeeds);
    const PlanResult plan = autoPilot.calculateSafeTrajectory();

    if (jsonOutput) {
        writeJson(plan);
        return plan.reachedTarget() ? 0 : 1;
    }

    std::cout << "===============================================================\n";
    std::cout << "     CYBER-PHYSICAL AUTONOMOUS NAVIGATION PIPELINE         \n";
    std::cout << "===============================================================\n";

    std::cout << "\n[CONTROL] Streaming Executed Trajectory Path Coordinates:\n";
    std::cout << "---------------------------------------------------------------\n";
    for (std::size_t i = 0; i < plan.trajectory.size(); ++i) {
        std::cout << "Waypoint " << i << " | Latitude Position Coordinate: ("
                  << plan.trajectory[i].x << ", " << plan.trajectory[i].y << ")\n";
    }
    std::cout << "---------------------------------------------------------------\n";
    std::cout << "Path length: " << plan.pathLength << " coordinate units\n";
    std::cout << "Minimum obstacle clearance: " << plan.minimumClearance << " coordinate units\n";
    std::cout << "Evasive maneuvers: " << plan.evasiveManeuvers << '\n';
    if (!plan.reachedTarget()) {
        const char* reason = terminationName(plan.terminationReason);
        std::cerr << "Planner stopped before reaching the target: " << reason << ".\n";
        return 1;
    }

    std::cout << "Target intercept accomplished safely. Mission lifecycle complete.\n";
    return 0;
}
