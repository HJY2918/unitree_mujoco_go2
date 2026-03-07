cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

enable_language(C)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(mujoco REQUIRED)
find_package(unitree_sdk2 REQUIRED)


FILE (GLOB SIM_SRC
        src/joystick/joystick.cc
        src/mujoco/*.cc
        src/unitree_sdk2_bridge/*.cc)

set(SIM_DEPENDENCIES 
    pthread
    mujoco::mujoco
    glfw
    yaml-cpp
    unitree_sdk2)


add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)
target_link_libraries(unitree_mujoco  ${SIM_DEPENDENCIES})

# ================= 新增：main_push 的可执行文件 =================
# 假设 main_push.cc 放在 src/ 目录下。
# 这里将编译出的程序命名为 unitree_mujoco_push，你可以根据需要修改这个名字。
add_executable(unitree_mujoco_push ${SIM_SRC} src/main_push.cc)
target_link_libraries(unitree_mujoco_push ${SIM_DEPENDENCIES})
# ================================================================

add_executable(test test/test_unitree_sdk2.cpp)
target_link_libraries(test unitree_sdk2)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)

set(CMAKE_BUILD_TYPE Release)
