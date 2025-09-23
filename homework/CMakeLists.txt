cmake_minimum_required(VERSION 3.10)
project(Homework)

set(CMAKE_CXX_STANDARD 14)

# 查找必要的包
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

# 包含目录
include_directories(${OpenCV_INCLUDE_DIRS})

# Part1: fmt测试程序
add_executable(fmt_test fmt_test.cpp)
target_link_libraries(fmt_test fmt::fmt)

# Part2: OpenCV主程序
add_executable(main main.cpp tools.cpp)
target_link_libraries(main ${OpenCV_LIBS} fmt::fmt)