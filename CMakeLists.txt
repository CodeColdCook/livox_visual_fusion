cmake_minimum_required(VERSION 3.0)
project(livox_visual_fusion)


set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
  pcl_ros
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


catkin_package(
    CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros sensor_msgs
    DEPENDS EIGEN3
    INCLUDE_DIRS include
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(livox_visual_fusion_node
               test/livox_visual_fusion_node.cpp
               src/livox_visual_fusion.cpp)
target_link_libraries(livox_visual_fusion_node  ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS} )

