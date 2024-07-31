cmake_minimum_required(VERSION 3.0.2)
project(moveit_collision_checker)

add_compile_options(-std=c++2a -funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

set(DEPS
    graph_core::graph_core
)

set(ROS_DEPS
    moveit_core
    roscpp
)

find_package(graph_core REQUIRED)
find_package(catkin REQUIRED COMPONENTS
${ROS_DEPS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_collision_checker
  CATKIN_DEPENDS ${ROS_DEPS}
  DEPENDS
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
   src/${PROJECT_NAME}/collision_checkers/thread_pool_moveit_collision_checker.cpp

   src/${PROJECT_NAME}/plugins/collision_checkers/moveit_collision_checker_plugin.cpp
   src/${PROJECT_NAME}/plugins/collision_checkers/parallel_moveit_collision_checker_plugin.cpp
   )

target_link_libraries(${PROJECT_NAME} PUBLIC
  ${DEPS}
  ${catkin_LIBRARIES}
)

#Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)
