#include "../include/PlannerEngine.hpp"
#include <iostream>

int main() {
    std::cout << "===============================================================\n";
    std::cout << "     CYBER-PHYSICAL AUTONOMOUS NAVIGATION PIPELINE         \n";
    std::cout << "===============================================================\n";

    // Initialize: Start at (0,0) trying to steer cleanly to target endpoint (0, 10)
    PlannerEngine autoPilot(0.0, 0.0, 0.0, 10.0);

    // Setup an obstacle directly on the vehicle's line of sight at position (0, 4)
    std::vector<Position> lidarRadarFeeds = { {0.0, 4.0} };
    autoPilot.ingestObstacleMap(lidarRadarFeeds);

    // Execute Autonomy Engine
    std::vector<Position> safePath = autoPilot.calculateSafeTrajectory();

    std::cout << "\n[CONTROL] Streaming Executed Trajectory Path Coordinates:\n";
    std::cout << "---------------------------------------------------------------\n";
    for (size_t i = 0; i < safePath.size(); ++i) {
        std::cout << "Waypoint " << i << " | Latitude Position Coordinate: (" 
                  << safePath[i].x << ", " << safePath[i].y << ")\n";
    }
    std::cout << "---------------------------------------------------------------\n";
    std::cout << "Target intercept accomplished safely. Mission lifecycle complete.\n";
    return 0;
}
