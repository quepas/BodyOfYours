cmake_minimum_required(VERSION 2.8.11)

project(BodyOfYours)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt5Widgets)

add_executable(BodyOfYoursApp WIN32 main.cpp)

target_link_libraries(helloworld Qt5::Widgets)
