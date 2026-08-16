#include "PlannerEngine.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
bool nearlyEqual(double left, double right, double tolerance = 1e-9) {
    return std::abs(left - right) <= tolerance;
}

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}
}

int main() {
    try {
        PlannerEngine direct({0.0, 0.0}, {0.0, 3.0});
        const auto directPlan = direct.calculateSafeTrajectory();
        require(directPlan.reachedTarget(), "direct plan should reach target");
        require(directPlan.trajectory.size() == 4, "direct plan should contain four positions");
        require(nearlyEqual(directPlan.trajectory.back().y, 3.0), "direct plan target mismatch");

        PlannerEngine avoidance({0.0, 0.0}, {0.0, 10.0});
        avoidance.ingestObstacleMap({{0.0, 4.0}});
        const auto firstPlan = avoidance.calculateSafeTrajectory();
        const auto secondPlan = avoidance.calculateSafeTrajectory();
        require(firstPlan.reachedTarget(), "avoidance plan should reach target");
        require(firstPlan.trajectory.size() == secondPlan.trajectory.size(), "planner should be repeatable");

        bool deviated = false;
        for (const auto& point : firstPlan.trajectory) {
            deviated = deviated || !nearlyEqual(point.x, 0.0);
            require(std::hypot(point.x, point.y - 4.0) >= 1.5, "trajectory entered obstacle safety radius");
        }
        require(deviated, "blocked plan should deviate from direct centerline");

        PlannerEngine limited({0.0, 0.0}, {0.0, 10.0}, {.stepSize = 1.0, .safetyRadius = 1.5, .maxIterations = 2});
        require(!limited.calculateSafeTrajectory().reachedTarget(), "limited plan should report incomplete termination");

        std::cout << "All planner tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Planner test failure: " << error.what() << '\n';
        return 1;
    }
}
