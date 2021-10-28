cmake_minimum_required(VERSION 3.0.2)
project(mbf_mesh_core)

find_package(catkin REQUIRED COMPONENTS
  mbf_abstract_core
  mesh_map
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS mbf_abstract_core mesh_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
