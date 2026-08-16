#include "PlannerEngine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

PlannerEngine::PlannerEngine(Position start, Position target, PlannerConfig plannerConfig)
    : startPos(start), targetPos(target), config(plannerConfig) {
    if (config.stepSize <= 0.0) {
        throw std::invalid_argument("stepSize must be greater than zero");
    }
    if (config.safetyRadius < 0.0) {
        throw std::invalid_argument("safetyRadius cannot be negative");
    }
    if (config.maxIterations == 0) {
        throw std::invalid_argument("maxIterations must be greater than zero");
    }
}

void PlannerEngine::ingestObstacleMap(const std::vector<Position>& detectedObstacles) {
    obstacles = detectedObstacles;
}

bool PlannerEngine::isCollisionRisk(Position from, Position to) const {
    const double segmentX = to.x - from.x;
    const double segmentY = to.y - from.y;
    const double segmentLengthSquared = segmentX * segmentX + segmentY * segmentY;

    for (const auto& obs : obstacles) {
        double projection = 0.0;
        if (segmentLengthSquared > 0.0) {
            projection = ((obs.x - from.x) * segmentX + (obs.y - from.y) * segmentY)
                / segmentLengthSquared;
            projection = std::clamp(projection, 0.0, 1.0);
        }

        const Position closest{
            from.x + projection * segmentX,
            from.y + projection * segmentY
        };
        const double distance = std::hypot(closest.x - obs.x, closest.y - obs.y);
        if (distance < config.safetyRadius) {
            return true;
        }
    }
    return false;
}

PlanResult PlannerEngine::calculateSafeTrajectory() const {
    Position current = startPos;
    PlanResult result{{current}, TerminationReason::IterationLimitReached};

    for (std::size_t iteration = 0; iteration < config.maxIterations; ++iteration) {
        const double deltaX = targetPos.x - current.x;
        const double deltaY = targetPos.y - current.y;
        const double distanceToTarget = std::hypot(deltaX, deltaY);

        if (distanceToTarget <= config.stepSize) {
            result.trajectory.push_back(targetPos);
            result.terminationReason = TerminationReason::TargetReached;
            return result;
        }

        Position next{
            current.x + (deltaX / distanceToTarget) * config.stepSize,
            current.y + (deltaY / distanceToTarget) * config.stepSize
        };

        if (isCollisionRisk(current, next)) {
            next = {
                current.x - (deltaY / distanceToTarget) * config.stepSize,
                current.y + (deltaX / distanceToTarget) * config.stepSize
            };
        }

        current = next;
        result.trajectory.push_back(current);
    }

    return result;
}
