cmake_minimum_required(VERSION 3.0.2)
project(wall_seg)

find_package(PCL REQUIRED COMPONENTS common io visualization features filters segmentation kdtree)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_conversions message_generation )


add_message_files(
  FILES
  WallInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wall_seg
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Boost_SYSTEM_INCLUDE_DIRS}
)

add_executable(wall_seg_node src/wall_seg_node.cpp)
target_link_libraries(wall_seg_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${Boost_SYSTEM_LIBRARIES}
)