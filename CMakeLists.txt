cmake_minimum_required(VERSION 2.8.11)

project(BodyOfYours)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt4 4.8.0 REQUIRED QtGui QtCore)

file (GLOB RAW_sources src/*.cpp)
file (GLOB RAW_headers src/*.h)
#file (GLOB RAW_gui gui/*.ui)

add_executable(BodyOfYoursApp ${RAW_sources} ${RAW_headers})
target_link_libraries(BodyOfYoursApp Qt4::QtCore Qt4::QtGui)
