cmake_minimum_required(VERSION 3.5)
project(mbf_mesh_core)


find_package(ament_cmake_ros REQUIRED)
set(dependencies
  mbf_abstract_core
  mesh_map
)

foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

include_directories(
  include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()