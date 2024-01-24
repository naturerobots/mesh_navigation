cmake_minimum_required(VERSION 3.8)
project(dijkstra_mesh_planner)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(mesh_map REQUIRED)
find_package(rclcpp REQUIRED)

pluginlib_export_plugin_description_file(mbf_mesh_core dijkstra_mesh_planner.xml)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(JSONCPP jsoncpp)

add_library(${PROJECT_NAME}
  src/dijkstra_mesh_planner.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_definitions(${PROJECT_NAME} PRIVATE "FTC_CONTROLLER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp)

install(DIRECTORY include/
 DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp)
ament_package()