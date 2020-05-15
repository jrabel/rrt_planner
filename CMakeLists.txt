cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_core
  costmap_2d
  base_local_planner
  pluginlib
)

catkin_package(
# INCLUDE_DIRS include
  LIBRARIES rrt_planner
  CATKIN_DEPENDS 
    roscpp nav_core costmap_2d base_local_planner pluginlib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_library(rrt_planner src/RRTPlanner.cpp src/RRTree.cpp src/Pose2D.cpp)
# add_dependencies(rrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rrt_planner ${catkin_LIBRARIES})

add_executable(base_controller_node src/base_controller_node.cpp src/BaseController.cpp src/Pose2D.cpp)
target_link_libraries(base_controller_node ${catkin_LIBRARIES})