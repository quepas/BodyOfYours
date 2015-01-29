cmake_minimum_required(VERSION 2.8.11)

project(BodyOfYours)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)
include_directories(${Qt5Widgets_INCLUDE_DIRS})

add_executable(BodyOfYoursApp WIN32 src/main.cpp)

target_link_libraries(BodyOfYoursApp Qt5::Widgets)
