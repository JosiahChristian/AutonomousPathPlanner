cmake_minimum_required(VERSION 3.22)
project(AutonomousPathPlanner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23) # Compiling on modern C++ modules standard
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(run_planner src/main.cpp src/PlannerEngine.cpp)
