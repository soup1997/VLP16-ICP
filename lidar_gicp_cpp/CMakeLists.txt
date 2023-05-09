cmake_minimum_required(VERSION 3.0.2)
project(lidar_gicp_cpp)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  tf2

  PCL REQUIRED
)

catkin_package(
  INCLUDE_DIRS include

  CATKIN_DEPENDS roscpp sensor_msgs std_msgs geometry_msgs tf2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/lidar_gicp_cpp_node.cpp src/gicp.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)
