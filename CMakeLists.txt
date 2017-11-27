project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/astar.h
        src/track.cpp src/track.h
        src/trajectory_planner.cpp src/trajectory_planner.h
        src/trajectory_state.cpp src/trajectory_state.h
        src/discrete_prediction.cpp src/discrete_prediction.h
        src/discrete_trajectory_planner.cpp src/discrete_trajectory_planner.h
        src/pid_controller.cpp src/pid_controller.h
        src/spline.h src/json.hpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(path_planning src/main.cpp ${sources})

target_link_libraries(path_planning z ssl uv uWS)

# Automated testing based on Google Test with CMakeLists changes loosely based on
# https://github.com/google/googletest/tree/master/googletest#incorporating-into-an-existing-cmake-project

# Download and unpack googletest at configure time
configure_file(CMakeLists-download.txt googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        RESULT_VARIABLE result
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
if(result)
    message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
        ${CMAKE_BINARY_DIR}/googletest-build)

# Define testing target
set(test_sources test/astar_test.cpp test/discrete_trajectory_planner_test.cpp)
add_executable(test_path_planning ${test_sources} ${sources})
target_link_libraries(test_path_planning gtest_main)
