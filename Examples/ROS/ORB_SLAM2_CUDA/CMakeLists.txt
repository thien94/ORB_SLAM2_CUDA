cmake_minimum_required(VERSION 3.0.2)
project(ORB_SLAM2_CUDA)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslaunch
  octomap_ros
  octomap_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  image_transport
)

#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
roslaunch_add_file_check(launch)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nxtbot
#  CATKIN_DEPENDS roscpp rospy std_msgs roslaunch
#  DEPENDS OpenCV
)
find_package(cv_bridge)

IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE DEBUG)
ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

find_package(Python 3.8.5 QUIET)



# Check C++11 or C++0x support

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++14.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

find_package(OpenCV 4.4.0 REQUIRED)
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(CUDA REQUIRED)

find_package( PCL 1.3 REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

include_directories(
include
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/lib
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${CUDA_INCLUDE_DIRS}
${catkin_INCLUDE_DIRS} 
${PCL_LIBRARIES}
)

set(LIBS 
cv_bridge
${catkin_LIBRARIES}
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PCL_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2_CUDA.so
)

# Node for monocular camera
add_executable(Mono
lib/SlamData.cc
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

#[[
# Node for stereo camera
add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

#]]
