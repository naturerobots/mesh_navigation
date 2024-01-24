cmake_minimum_required(VERSION 3.5)
project(mesh_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_ros REQUIRED)
set(dependencies
  rclcpp
  LVR2
  CURL
  Eigen3
)
foreach(dep IN LISTS dependencies)
  find_package(${dep} REQUIRED)
endforeach()

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/mesh_client.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} jsoncpp)

install(DIRECTORY include/
  DESTINATION include
)
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()