cmake_minimum_required(VERSION 3.8)
project(mesh_controller)

find_package(ament_cmake_ros REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(mesh_map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

pluginlib_export_plugin_description_file(mbf_mesh_core mesh_controller.xml)

add_library(${PROJECT_NAME} src/mesh_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(${PROJECT_NAME} PRIVATE "MESH_CONTROLLER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp tf2_geometry_msgs)

install(DIRECTORY include DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp tf2_geometry_msgs)
ament_package()