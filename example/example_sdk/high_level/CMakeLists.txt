cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(high_follow_sin follow_sin.cpp)
target_link_libraries(high_follow_sin unitree_sdk2)

add_executable(sportmode_test sportmode_test.cpp)
target_link_libraries(sportmode_test unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)