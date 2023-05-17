cmake_minimum_required(VERSION 3.0)
project(livox_visual_fusion)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  pcl_ros
  camera_models
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  sensor_msgs 
  image_transport
  cv_bridge
  pcl_ros 
  camera_models
)

add_library(${PROJECT_NAME} SHARED
    src/livox_visual_fusion.cpp
)
target_link_libraries(${PROJECT_NAME} 
  camera_models
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
)
add_dependencies(${PROJECT_NAME} 
${${PROJECT_NAME}_EXPORTED_TARGETS}
${catkin_EXPORTED_TARGETS}
)

add_executable(livox_visual_fusion_node
  test/livox_visual_fusion_node.cpp
)
target_link_libraries(livox_visual_fusion_node  
  ${PROJECT_NAME} 
)

add_executable(fusion_with_fisheye
  test/fusion_with_fisheye.cpp
)
target_link_libraries(fusion_with_fisheye  
  ${PROJECT_NAME} 
)