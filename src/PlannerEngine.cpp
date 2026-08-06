#include "../include/PlannerEngine.hpp"
#include <iostream>
#include <cmath>

PlannerEngine::PlannerEngine(double startX, double startY, double targetX, double targetY) {
    currentPos = {startX, startY};
    targetPos = {targetX, targetY};
}

void PlannerEngine::ingestObstacleMap(const std::vector<Position>& detectedObstacles) {
    obstacles = detectedObstacles;
    std::cout << "[PERCEPTION] Successfully mapped " << obstacles.size() << " dynamic radar obstacle coordinates.\n";
}

bool PlannerEngine::isCollisionRisk(double checkX, double checkY, double safetyRadius) {
    for (const auto& obs : obstacles) {
        double distance = std::sqrt(std::pow(checkX - obs.x, 2) + std::pow(checkY - obs.y, 2));
        if (distance < safetyRadius) return true;
    }
    return false;
}

std::vector<Position> PlannerEngine::calculateSafeTrajectory() {
    std::vector<Position> trajectory;
    trajectory.push_back(currentPos);
    
    double stepSize = 1.0;
    double safetyZone = 1.5; // Meters around an obstacle
    int iterations = 0;

    std::cout << "[PLANNING] Computing safe vector trajectory coordinates...\n";

    while (iterations < 20) {
        double deltaX = targetPos.x - currentPos.x;
        double deltaY = targetPos.y - currentPos.y;
        double distanceToTarget = std::sqrt(deltaX*deltaX + deltaY*deltaY);

        if (distanceToTarget < stepSize) {
            currentPos = targetPos;
            trajectory.push_back(currentPos);
            break;
        }

        // Calculate standard direct step trajectory
        double nextX = currentPos.x + (deltaX / distanceToTarget) * stepSize;
        double nextY = currentPos.y + (deltaY / distanceToTarget) * stepSize;

        // Autonomy Guard Check: If direct path is blocked, execute defensive steering modification
        if (isCollisionRisk(nextX, nextY, safetyZone)) {
            std::cout << "[PLANNING] Collision hazard detected at (" << nextX << ", " << nextY << "). Initiating evasive vector path translation!\n";
            // Steer 90-degrees perpendicular to dodge the obstacle bounding zone
            nextX = currentPos.x - (deltaY / distanceToTarget) * stepSize;
            nextY = currentPos.y + (deltaX / distanceToTarget) * stepSize;
        }

        currentPos = {nextX, nextY};
        trajectory.push_back(currentPos);
        iterations++;
    }

    return trajectory;
}
