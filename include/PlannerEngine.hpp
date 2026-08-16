#ifndef PLANNER_ENGINE_HPP
#define PLANNER_ENGINE_HPP

#include <cstddef>
#include <vector>

struct Position {
    double x;
    double y;
};

struct PlannerConfig {
    double stepSize{1.0};
    double safetyRadius{1.5};
    std::size_t maxIterations{100};
};

enum class TerminationReason {
    TargetReached,
    IterationLimitReached
};

struct PlanResult {
    std::vector<Position> trajectory;
    TerminationReason terminationReason;

    [[nodiscard]] bool reachedTarget() const noexcept {
        return terminationReason == TerminationReason::TargetReached;
    }
};

class PlannerEngine {
public:
    PlannerEngine(Position start, Position target, PlannerConfig config = {});
    
    // Ingests lidar/sonar sensor feeds (Perception Stage)
    void ingestObstacleMap(const std::vector<Position>& detectedObstacles);
    
    // Processes dynamic trajectories avoiding danger grids (Planning Stage)
    [[nodiscard]] PlanResult calculateSafeTrajectory() const;

private:
    Position startPos;
    Position targetPos;
    PlannerConfig config;
    std::vector<Position> obstacles;

    [[nodiscard]] bool isCollisionRisk(Position candidate) const;
};

#endif
