cmake_minimum_required(VERSION 3.10)

# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(state_machine_example main.cpp)
target_link_libraries(state_machine_example unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)