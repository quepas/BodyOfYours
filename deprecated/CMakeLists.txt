cmake_minimum_required(VERSION 2.8.11)

project(BodyOfYours)

# libraries path
set(PCL_INC "D:/Programowanie/Biblioteki/Tidy/pcl-1.7/include/pcl-1.7/")
set(PCL_LIB "D:/Programowanie/Biblioteki/Tidy/pcl-1.7/lib")
set(BOOST_INC "D:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/include")
set(BOOST_LIB "D:/Programowanie/Biblioteki/boost_1.50_msvc2010_x86/lib")
set(EIGEN_INC "D:/Programowanie/Biblioteki/Tidy/Eigen/include")
set(REME_INC "D:/Programy/RecounstructMe SDK/inc")
set(REME_LIB "D:/Programy/RecounstructMe SDK/lib")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

# set libraries
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
link_directories(${VTK_LIBRARY_DIRS})

include_directories(${BOOST_INC} ${PCL_INC} ${EIGEN_INC} ${REME_INC})
link_directories(${BOOST_LIB} ${PCL_LIB} ${REME_LIB})

find_package(Qt4 4.8.0 REQUIRED QtGui QtCore)

file (GLOB RAW_sources src/*.cpp src/data/*.cpp)
file (GLOB RAW_headers src/*.h src/data/*.h)
#file (GLOB RAW_gui gui/*.ui)

add_executable(BodyOfYoursApp ${RAW_sources} ${RAW_headers})
target_link_libraries(BodyOfYoursApp Qt4::QtCore Qt4::QtGui QVTK pcl_visualization_debug pcl_common_debug pcl_io_debug LibReconstructMeSDK)
