cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories("/usr/include/flycapture")
find_library(FLYCAPTURE2 flycapture)

#find_package(FlyCapture)
#include_directories(${FlyCapture_INCLUDE_DIR})

find_package( Boost 1.40 COMPONENTS program_options date_time system thread REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(flycam src/StereoCapture.cpp)

target_link_libraries(flycam
${OpenCV_LIBS} ${FLYCAPTURE2} ${Boost_LIBRARIES}
 )
