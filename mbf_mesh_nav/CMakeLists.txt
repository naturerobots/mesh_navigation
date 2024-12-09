cmake_minimum_required(VERSION 3.5)
project(mbf_mesh_nav)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake_ros REQUIRED)
# ROS deps
set(dependencies
  geometry_msgs
  mbf_abstract_nav
  mbf_simple_core
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

find_package(LVR2 REQUIRED)
find_package(MPI)
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
target_include_directories(mbf_mesh_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LVR2_INCLUDE_DIRS})
ament_target_dependencies(mbf_mesh_server ${dependencies})
target_link_libraries(mbf_mesh_server 
  ${JSONCPP_LIBRARIES}
  ${LVR2_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
)

add_executable(${PROJECT_NAME} src/mbf_mesh_nav.cpp)
ament_target_dependencies(${PROJECT_NAME} ${dependencies} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LVR2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}
  ${LVR2_LIBRARIES}
  ${MPI_CXX_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  mbf_mesh_server
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS mbf_mesh_server
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()