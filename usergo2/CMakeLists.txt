cmake_minimum_required(VERSION 3.10)

project(mainProj)
# 设置C++标准为C++17
set(CMAKE_CXX_STANDARD 17)

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# 包含路径
include_directories(${unitree_sdk2_INCLUDE_DIRS})

# # 添加库
# add_library(UserBalanceCtrl ./control/UserBalanceCtrl.cpp)
# add_library(conversion ./interface/conversion.cpp)

# 定义源文件列表
set(SOURCE_FILES
    main.cpp
    ./control/UserBalanceCtrl.cpp
    ./interface/conversion.cpp
)

add_executable(main ${SOURCE_FILES})

# add_executable(main main.cpp)

target_link_libraries(main unitree_sdk2)


# 输出调试信息
message(STATUS "Source files: ${SOURCE_FILES}")


set(CMAKE_BUILD_TYPE Debug)