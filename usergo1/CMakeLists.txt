cmake_minimum_required(VERSION 3.16)
project(usercontroller)

# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
# link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

add_executable(main main.cpp)

target_link_libraries(main  unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)