cmake_minimum_required(VERSION 3.16)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")

find_package(unitree_sdk2 REQUIRED)

add_executable(test_publisher 
    publisher.cpp 
    HelloWorldData.cpp
)

target_link_libraries(test_publisher unitree_sdk2)

add_executable(test_subscriber 
    subscriber.cpp 
    HelloWorldData.cpp
)
target_link_libraries(test_subscriber unitree_sdk2)

SET(CMAKE_BUILD_TYPE Release)