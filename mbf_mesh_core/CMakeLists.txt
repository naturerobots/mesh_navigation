cmake_minimum_required(VERSION 3.5)
project(mbf_mesh_core)

# DEFAULT RELEASE
if (NOT EXISTS ${CMAKE_BINARY_DIR}/CMakeCache.txt)
  if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
  endif()
endif()

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