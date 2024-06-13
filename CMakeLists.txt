cmake_minimum_required(VERSION 3.8)
project(moveit_collision_checker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-funroll-loops -Wall -Ofast)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(graph_core REQUIRED)

set(DEPS_LIB
    graph_core
    )
set(DEPS_TARGET
    graph_core::graph_core
    )
set(ROS_DEPS
    rclcpp
    moveit_core
    )

# Declare a C++ library
add_library(${PROJECT_NAME} SHARED
            src/${PROJECT_NAME}/collision_checkers/parallel_moveit_collision_checker.cpp

            src/${PROJECT_NAME}/plugins/collision_checkers/moveit_collision_checker_plugin.cpp
            src/${PROJECT_NAME}/plugins/collision_checkers/parallel_moveit_collision_checker_plugin.cpp
 )
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPS_TARGET})
 target_include_directories(${PROJECT_NAME} PUBLIC
   $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
 ament_target_dependencies(${PROJECT_NAME} PUBLIC ${ROS_DEPS})
 
 ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
 ament_export_dependencies(${DEPS_LIB} ${ROS_DEPS})
 
 install(TARGETS ${PROJECT_NAME}
 EXPORT export_${PROJECT_NAME}
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin
 INCLUDES DESTINATION include
)

install(
 DIRECTORY include/
 DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 # the following line skips the linter which checks for copyrights
 # comment the line when a copyright and license is added to all source files
 set(ament_cmake_copyright_FOUND TRUE)
 # the following line skips cpplint (only works in a git repo)
 # comment the line when this package is in a git repo and when
 # a copyright and license is added to all source files
 set(ament_cmake_cpplint_FOUND TRUE)
 ament_lint_auto_find_test_dependencies()
endif()

ament_package()
