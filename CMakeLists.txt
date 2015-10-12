cmake_minimum_required(VERSION 2.8.11)

project(BodyOfYours)

# setup Qt
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# libraries path
set (RECFUSION_INC "D:/Programy/RecFusion SDK/include")
set (RECFUSION_LIB "D:/Programy/RecFusion SDK/lib")
set (QOGLVIEWER_INC "D:/Programowanie/Biblioteki/libQGLViewer-2.6.3")
set (QOGLVIEWER_LIB "D:/Programowanie/Biblioteki/libQGLViewer-2.6.3/QGLViewer")
set (CMAKE_PREFIX_PATH "D:/Programy/Qt5.5A/5.5/msvc2013_64")

# set libraries
include_directories(${RECFUSION_INC} ${QOGLVIEWER_INC})
link_directories(${RECFUSION_LIB} ${QOGLVIEWER_LIB})

find_package(Qt5Widgets)
find_package(Qt5Gui)
find_package(Qt5OpenGL)
find_package(Qt5Xml)

file (GLOB RAW_sources src/*.cpp)
file (GLOB RAW_headers src/*.h)
#file (GLOB RAW_gui gui/*.ui)

add_executable(BodyOfYoursApp ${RAW_sources} ${RAW_headers})
target_link_libraries(BodyOfYoursApp Qt5::Widgets Qt5::Gui Qt5::OpenGL Qt5::Xml RecFusion QGLViewer2 opengl32 glu32)
