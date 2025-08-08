cmake_minimum_required(VERSION 3.5)
project(mesh_map)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake_ros REQUIRED)
# ROS Deps
set(dependencies
  geometry_msgs
  mesh_msgs_conversions
  pluginlib
  rclcpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  std_srvs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

find_package(LVR2 REQUIRED)
find_package(assimp REQUIRED)
find_package(MPI)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
  ${LVR2_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/abstract_layer.cpp
  src/mesh_map.cpp
  src/util.cpp
  src/layer_manager.cpp
  src/timer.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ASSIMP_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} 
    ${LVR2_LIBRARIES}
    ${ASSIMP_LIBRARIES}
)
target_compile_definitions(${PROJECT_NAME}
  PUBLIC
  ${LVR2_DEFINITIONS}
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # Add test lib with layer plugin(s)
  # -> Why a test plugin? Concrete plugin are available in the mesh_layers package.
  #    However, mesh_map cannot test_depend on it without introducing a circular dependency (on colcon pkg level, the build order would be unclear).
  add_library(${PROJECT_NAME}_test_layer_plugin
    test/layer_plugin.cpp
  )
  ament_target_dependencies(${PROJECT_NAME}_test_layer_plugin pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_layer_plugin mesh_map)
  pluginlib_export_plugin_description_file(mesh_map test/layer_plugin.xml)
  install(TARGETS 
    ${PROJECT_NAME}_test_layer_plugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # add gtest tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_mesh_map_test test/mesh_map_test.cpp)
  target_include_directories(${PROJECT_NAME}_mesh_map_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_mesh_map_test ${PROJECT_NAME})
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
