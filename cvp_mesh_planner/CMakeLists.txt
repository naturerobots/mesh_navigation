cmake_minimum_required(VERSION 2.8.3)
project(cvp_mesh_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mbf_mesh_core
  mbf_utility
  mbf_msgs
  mesh_map
  dynamic_reconfigure
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

generate_dynamic_reconfigure_options(
  cfg/CVPMeshPlanner.cfg
)

catkin_package(
  LIBRARIES mesh_planner
  CATKIN_DEPENDS roscpp mbf_mesh_core mbf_utility mbf_msgs mesh_map dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/cvp_mesh_planner.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES cvp_mesh_planner.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

