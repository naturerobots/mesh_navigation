cmake_minimum_required(VERSION 3.5)
project(mbf_mesh_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_ros REQUIRED)
set(dependencies
  geometry_msgs
  LVR2
  mbf_abstract_nav
  mbf_mesh_core
  mbf_msgs
  mesh_map
  nav_msgs
  pluginlib
  rclcpp
  std_srvs
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
)

add_library(mbf_mesh_server
  src/mesh_controller_execution.cpp
  src/mesh_navigation_server.cpp
  src/mesh_planner_execution.cpp
  src/mesh_recovery_execution.cpp
)
ament_target_dependencies(mbf_mesh_server ${dependencies})
target_link_libraries(mbf_mesh_server ${JSONCPP_LIBRARIES})

add_executable(${PROJECT_NAME} src/mbf_mesh_nav.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} mbf_mesh_server)
target_link_libraries(${PROJECT_NAME}
  ${JSONCPP_LIBRARIES}
  mbf_mesh_server
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME} mbf_mesh_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()