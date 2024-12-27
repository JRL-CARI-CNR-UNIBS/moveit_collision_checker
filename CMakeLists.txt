cmake_minimum_required(VERSION 3.16)
project(moveit_collision_checker)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(graph_core REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES moveit_collision_checker
  CATKIN_DEPENDS roscpp moveit_core 
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

target_link_libraries(${PROJECT_NAME} PUBLIC
  graph_core::graph_core
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
