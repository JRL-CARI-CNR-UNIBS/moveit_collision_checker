cmake_minimum_required(VERSION 3.0.2)
project(graph_ros1)

add_compile_options(-std=c++17 -funroll-loops -Wall -Ofast)
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
  LIBRARIES graph_ros1
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
  ${graph_core_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/collision_checkers/parallel_moveit_collision_checker.cpp

   src/${PROJECT_NAME}/plugins/collision_checkers/moveit_collision_checker_plugin.cpp
   src/${PROJECT_NAME}/plugins/collision_checkers/parallel_moveit_collision_checker_plugin.cpp

   src/${PROJECT_NAME}/plugins/metrics/euclidean_metrics_plugin.cpp

   src/${PROJECT_NAME}/plugins/samplers/informed_sampler_plugin.cpp

   src/${PROJECT_NAME}/plugins/solvers/rrt_plugin.cpp
   src/${PROJECT_NAME}/plugins/solvers/birrt_plugin.cpp
   src/${PROJECT_NAME}/plugins/solvers/anytime_rrt_plugin.cpp
   src/${PROJECT_NAME}/plugins/solvers/rrt_star_plugin.cpp
   )

target_link_libraries(${PROJECT_NAME}
  ${graph_core_LIBRARIES}
  ${catkin_LIBRARIES}
)
