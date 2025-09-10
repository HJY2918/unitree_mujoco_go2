# 设置项目名称和最低CMake版本
cmake_minimum_required(VERSION 3.10)
project(unitree_keyboard_publisher)

# # 设置C++标准为C++17
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# 包含所需的头文件目录
# include_directories(/usr/local/include/ddscxx /usr/local/include/iceoryx/v2.0.2)
list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)

# 使用pkg-config来查找libevdev
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBEVDEV REQUIRED libevdev)

# 包含libevdev的头文件目录
include_directories(${LIBEVDEV_INCLUDE_DIRS})

# 设置库路径
link_directories(${LIBEVDEV_LIBRARY_DIRS})

# # 设置编译选项（如编译标志）
# add_definitions(${LIBEVDEV_CFLAGS_OTHER})

# 编译keyboard_events_publisher可执行文件
add_executable(keyboard_events_publisher src/keyboard_events_publisher.cpp)
target_link_libraries(keyboard_events_publisher unitree_sdk2 ${LIBEVDEV_LIBRARIES})

SET(CMAKE_BUILD_TYPE Release)