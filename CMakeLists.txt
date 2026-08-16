cmake_minimum_required(VERSION 3.22)
project(AutonomousPathPlanner VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(planner_engine src/PlannerEngine.cpp)
target_include_directories(planner_engine PUBLIC include)
target_compile_options(planner_engine PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic;-Wconversion>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(run_planner src/main.cpp)
target_link_libraries(run_planner PRIVATE planner_engine)

include(CTest)
if(BUILD_TESTING)
    add_executable(planner_tests tests/PlannerEngineTests.cpp)
    target_link_libraries(planner_tests PRIVATE planner_engine)
    add_test(NAME planner_engine_tests COMMAND planner_tests)
    add_test(NAME planner_json_output COMMAND run_planner --json)
    set_tests_properties(planner_json_output PROPERTIES
        PASS_REGULAR_EXPRESSION "\"status\": \"target_reached\""
    )
endif()
