cmake_minimum_required(VERSION 2.8.3)
project(mesh_map)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  lvr_ros
  mesh_client
  pluginlib
  visualization_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(LVR2 2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)


generate_dynamic_reconfigure_options(
  cfg/MeshMap.cfg
)

catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES mesh_map
  CATKIN_DEPENDS geometry_msgs lvr_ros visualization_msgs dynamic_reconfigure pluginlib mesh_client
  DEPENDS LVR2 Boost JSONCPP
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LVR2_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mesh_map.cpp
  src/util.cpp
)

add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencfg
)

link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${JSONCPP_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

