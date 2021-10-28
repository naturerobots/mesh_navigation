cmake_minimum_required(VERSION 3.0.2)
project(mesh_layers)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  mesh_map
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(
  cfg/HeightDiffLayer.cfg
  cfg/InflationLayer.cfg
  cfg/RoughnessLayer.cfg
  cfg/SteepnessLayer.cfg
  cfg/RidgeLayer.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_layers
  CATKIN_DEPENDS mesh_map dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/roughness_layer.cpp
  src/height_diff_layer.cpp
  src/inflation_layer.cpp
  src/steepness_layer.cpp
  src/ridge_layer.cpp
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

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(FILES
  mesh_layers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

