# 设置项目名称和最低CMake版本
cmake_minimum_required(VERSION 3.10)

# # 设置C++标准为C++17
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含所需的头文件目录
# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(low_level low_level.cpp)
target_link_libraries(low_level unitree_sdk2)

add_executable(stand_example_go2 stand_example_go2.cpp)
target_link_libraries(stand_example_go2 unitree_sdk2)

add_executable(stand_example_b2  stand_example_b2.cpp)
target_link_libraries(stand_example_b2 unitree_sdk2)