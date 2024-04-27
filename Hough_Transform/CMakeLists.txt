cmake_minimum_required(VERSION 3.8)
project("hough_transform")

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g -ggdb -Wall")
set(CMAKE_C_COMPILER "D:/program/MinGW/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "D:/program/MinGW/bin/g++.exe")
set(CMAKE_MAKE_PROGRAM "D:/program/MinGW/bin/make.exe")


# 设置默认构建类型为Debug，如果没有指定
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# 查找OpenCV
find_package(OpenCV REQUIRED)

# 检查是否成功找到OpenCV
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "Cannot find OpenCV. Please set OpenCV_DIR.")
endif()

message(STATUS "OpenCV library found at ${OpenCV_INCLUDE_DIRS}")

# 添加可执行文件
add_executable(hough_transform hough_transform.cpp)

# 链接OpenCV库
target_link_libraries(hough_transform PRIVATE ${OpenCV_LIBS})

# 包含OpenCV头文件
target_include_directories(hough_transform PRIVATE ${OpenCV_INCLUDE_DIRS})
