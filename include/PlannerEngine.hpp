#ifndef PLANNER_ENGINE_HPP
#define PLANNER_ENGINE_HPP

#include <vector>
#include <string>

struct Position {
    double x;
    double y;
};

class PlannerEngine {
public:
    PlannerEngine(double startX, double startY, double targetX, double targetY);
    
    // Ingests lidar/sonar sensor feeds (Perception Stage)
    void ingestObstacleMap(const std::vector<Position>& detectedObstacles);
    
    // Processes dynamic trajectories avoiding danger grids (Planning Stage)
    std::vector<Position> calculateSafeTrajectory();

private:
    Position currentPos;
    Position targetPos;
    std::vector<Position> obstacles;
    
    bool isCollisionRisk(double checkX, double checkY, double safetyRadius);
};

#endif
