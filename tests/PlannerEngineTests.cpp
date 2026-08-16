#include "PlannerEngine.hpp"

#include <cmath>
#include <iostream>
#include <limits>
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
        require(nearlyEqual(directPlan.pathLength, 3.0), "direct plan length mismatch");
        require(directPlan.evasiveManeuvers == 0, "direct plan should not report avoidance");

        PlannerEngine alreadyAtTarget({2.0, 3.0}, {2.0, 3.0});
        const auto stationaryPlan = alreadyAtTarget.calculateSafeTrajectory();
        require(stationaryPlan.reachedTarget(), "stationary plan should report target reached");
        require(stationaryPlan.trajectory.size() == 1, "stationary plan should not duplicate its position");

        PlannerEngine avoidance({0.0, 0.0}, {0.0, 10.0});
        avoidance.ingestObstacleMap({{0.0, 4.0}});
        const auto firstPlan = avoidance.calculateSafeTrajectory();
        const auto secondPlan = avoidance.calculateSafeTrajectory();
        require(firstPlan.reachedTarget(), "avoidance plan should reach target");
        require(firstPlan.trajectory.size() == secondPlan.trajectory.size(), "planner should be repeatable");
        require(firstPlan.pathLength > 10.0, "avoidance path should exceed direct distance");
        require(firstPlan.minimumClearance >= 1.5, "avoidance metrics should preserve clearance");
        require(firstPlan.evasiveManeuvers > 0, "avoidance plan should report evasive maneuvers");

        bool deviated = false;
        for (const auto& point : firstPlan.trajectory) {
            deviated = deviated || !nearlyEqual(point.x, 0.0);
            require(std::hypot(point.x, point.y - 4.0) >= 1.5, "trajectory entered obstacle safety radius");
        }
        require(deviated, "blocked plan should deviate from direct centerline");

        PlannerEngine coarseStep(
            {0.0, 0.0},
            {0.0, 10.0},
            {.stepSize = 4.0, .safetyRadius = 1.0, .maxIterations = 20}
        );
        coarseStep.ingestObstacleMap({{0.0, 2.0}});
        const auto coarsePlan = coarseStep.calculateSafeTrajectory();
        require(coarsePlan.trajectory.size() > 1, "coarse plan should generate a movement");
        require(
            !nearlyEqual(coarsePlan.trajectory[1].x, 0.0),
            "segment collision check should prevent waypoint tunneling"
        );

        PlannerEngine oneSided({0.0, 0.0}, {0.0, 5.0}, {.stepSize = 1.0, .safetyRadius = 0.75});
        oneSided.ingestObstacleMap({{0.0, 1.0}, {-0.75, 0.0}});
        const auto oneSidedPlan = oneSided.calculateSafeTrajectory();
        require(oneSidedPlan.trajectory.size() > 1, "one-sided scenario should move");
        require(oneSidedPlan.trajectory[1].x > 0.0, "planner should select the safe detour side");

        PlannerEngine blocked({0.0, 0.0}, {0.0, 5.0}, {.stepSize = 1.0, .safetyRadius = 0.75});
        blocked.ingestObstacleMap({{0.0, 1.0}, {-0.75, 0.0}, {0.75, 0.0}});
        const auto blockedPlan = blocked.calculateSafeTrajectory();
        require(!blockedPlan.reachedTarget(), "blocked scenario must not report success");
        require(
            blockedPlan.terminationReason == TerminationReason::NoSafeStep,
            "blocked scenario should report no safe step"
        );
        require(blockedPlan.trajectory.size() == 1, "blocked plan must not append an unsafe waypoint");

        PlannerEngine invalidStart({0.0, 0.0}, {0.0, 5.0}, {.safetyRadius = 1.0});
        invalidStart.ingestObstacleMap({{0.25, 0.0}});
        const auto invalidStartPlan = invalidStart.calculateSafeTrajectory();
        require(
            invalidStartPlan.terminationReason == TerminationReason::StartInCollision,
            "occupied start should be reported explicitly"
        );

        PlannerEngine invalidTarget({0.0, 0.0}, {0.0, 5.0}, {.safetyRadius = 1.0});
        invalidTarget.ingestObstacleMap({{0.25, 5.0}});
        const auto invalidTargetPlan = invalidTarget.calculateSafeTrajectory();
        require(
            invalidTargetPlan.terminationReason == TerminationReason::TargetInCollision,
            "occupied target should be reported explicitly"
        );

        PlannerEngine guardedEndpoint(
            {0.0, 0.0},
            {0.0, 0.5},
            {.stepSize = 1.0, .safetyRadius = 0.2, .maxIterations = 1}
        );
        guardedEndpoint.ingestObstacleMap({{0.0, 0.25}});
        const auto guardedEndpointPlan = guardedEndpoint.calculateSafeTrajectory();
        require(
            !guardedEndpointPlan.reachedTarget(),
            "final target segment must receive collision validation"
        );

        bool rejectedInvalidCoordinate = false;
        try {
            PlannerEngine invalid(
                {std::numeric_limits<double>::quiet_NaN(), 0.0},
                {1.0, 1.0}
            );
        } catch (const std::invalid_argument&) {
            rejectedInvalidCoordinate = true;
        }
        require(rejectedInvalidCoordinate, "planner should reject non-finite coordinates");

        PlannerEngine limited({0.0, 0.0}, {0.0, 10.0}, {.stepSize = 1.0, .safetyRadius = 1.5, .maxIterations = 2});
        require(!limited.calculateSafeTrajectory().reachedTarget(), "limited plan should report incomplete termination");

        std::cout << "All planner tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Planner test failure: " << error.what() << '\n';
        return 1;
    }
}
