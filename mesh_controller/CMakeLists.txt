cmake_minimum_required(VERSION 2.8.3)
project(mesh_controller)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  mbf_mesh_core
  mbf_utility
  mbf_msgs
  mesh_map
  tf2_geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/MeshController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_controller
  CATKIN_DEPENDS mbf_mesh_core mbf_msgs mesh_map
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mesh_controller.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES mesh_controller.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
