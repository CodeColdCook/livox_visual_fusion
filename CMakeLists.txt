cmake_minimum_required(VERSION 3.0.2)
project(livox_visual_fusion)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 -fext-numeric-literals")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  image_transport
  cv_bridge
  tf
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
# set(OPENCV_DIR "/home/nvidia/opencv3.4/opencv")
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
    DEPENDS EIGEN3 PCL
    INCLUDE_DIRS include
)



add_executable(livox_visual_fusion_node
               test/livox_visual_fusion_node.cpp
               src/livox_visual_fusion.cpp)
target_link_libraries(livox_visual_fusion_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

