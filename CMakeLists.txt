cmake_minimum_required(VERSION 3.0.2)
project(moveit_collision_checker)

add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  rviz_visual_tools
)

find_package(graph_core REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_collision_checker
  CATKIN_DEPENDS actionlib moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_visual_tools roscpp rviz_visual_tools
  DEPENDS graph_core
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/collision_checkers/parallel_moveit_collision_checker.cpp

   src/${PROJECT_NAME}/plugins/collision_checkers/moveit_collision_checker_plugin.cpp
   src/${PROJECT_NAME}/plugins/collision_checkers/parallel_moveit_collision_checker_plugin.cpp
   )

target_link_libraries(${PROJECT_NAME}
  graph_core::graph_core
  ${catkin_LIBRARIES}
)
