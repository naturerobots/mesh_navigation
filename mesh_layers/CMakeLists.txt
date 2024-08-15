cmake_minimum_required(VERSION 3.8)
project(mesh_layers)

find_package(ament_cmake_ros COMPONENTS)
find_package(mesh_map)
find_package(LVR2)

pluginlib_export_plugin_description_file(mesh_map mesh_layers.xml)

add_library(${PROJECT_NAME}
  src/border_layer.cpp
  src/roughness_layer.cpp
  src/height_diff_layer.cpp
  src/inflation_layer.cpp
  src/steepness_layer.cpp
  src/ridge_layer.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} mesh_map LVR2)
target_compile_definitions(${PROJECT_NAME} PRIVATE "MESH_LAYERS_BUILDING_LIBRARY")

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(mesh_map)
ament_package()