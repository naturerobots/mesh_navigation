cmake_minimum_required(VERSION 2.8.3)
project(mbf_mesh_nav)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  mbf_mesh_core
  mbf_abstract_nav
  mesh_map
  dynamic_reconfigure
  pluginlib
)

find_package(LVR2 2 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/MoveBaseFlex.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mbf_mesh_server
  CATKIN_DEPENDS mbf_mesh_core mesh_map dynamic_reconfigure mbf_abstract_nav pluginlib
  DEPENDS LVR2
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
)

add_library(mbf_mesh_server
  src/mesh_controller_execution.cpp
  src/mesh_navigation_server.cpp
  src/mesh_planner_execution.cpp
  src/mesh_recovery_execution.cpp
)

add_dependencies(mbf_mesh_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(mbf_mesh_server
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/mbf_mesh_nav.cpp)

add_dependencies(${PROJECT_NAME} mbf_mesh_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  mbf_mesh_server
)

install(TARGETS ${PROJECT_NAME} mbf_mesh_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
