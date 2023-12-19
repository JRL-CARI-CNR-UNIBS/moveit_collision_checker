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

#message("CNR LOGGER FOUND?")
#message(${cnr_logger_FOUND})
#message("BOOST FOUND?")
#message(${Boost_FOUND})
#message("YAML CPP FOUND?")
#message(${YAML_CPP_LIBRARIES})
#message("Ygraph core FOUND?")
#message(${graph_core_FOUND})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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
   src/${PROJECT_NAME}/parallel_moveit_collision_checker.cpp
 )
target_link_libraries(${PROJECT_NAME}
  ${graph_core_LIBRARIES}
)

