list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
# 包含路径
include_directories(${unitree_sdk2_INCLUDE_DIRS})
add_executable(go2w_sport_client go2w_sport_client.cpp)
target_link_libraries(go2w_sport_client unitree_sdk2)

add_executable(go2w_stand_example go2w_stand_example.cpp)
target_link_libraries(go2w_stand_example unitree_sdk2)
