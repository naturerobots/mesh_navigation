cmake_minimum_required(VERSION 3.8)
project(mesh_controller)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(mbf_mesh_core REQUIRED)
find_package(mbf_msgs REQUIRED)
find_package(mbf_utility REQUIRED)
find_package(mesh_map REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(LVR2 REQUIRED)
find_package(MPI)

pluginlib_export_plugin_description_file(mbf_mesh_core mesh_controller.xml)

add_library(${PROJECT_NAME} src/mesh_controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LVR2_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PRIVATE "MESH_CONTROLLER_BUILDING_LIBRARY")
ament_target_dependencies(${PROJECT_NAME} example_interfaces mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} 
    ${LVR2_LIBRARIES}
    ${MPI_CXX_LIBRARIES}
)

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
ament_export_dependencies(example_interfaces mbf_mesh_core mbf_msgs mbf_utility mesh_map rclcpp tf2_geometry_msgs)
ament_package()