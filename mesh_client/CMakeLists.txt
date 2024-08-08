cmake_minimum_required(VERSION 3.5)
project(mesh_client)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(LVR2 REQUIRED)
find_package(MPI)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(CURL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
)

add_library(${PROJECT_NAME}
  src/mesh_client.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
  ${LVR2_INCLUDE_DIRS})
ament_target_dependencies(${PROJECT_NAME} 
    rclcpp)
target_link_libraries(${PROJECT_NAME} 
    jsoncpp 
    Eigen3::Eigen
    ${LVR2_LIBRARIES}
    ${MPI_CXX_LIBRARIES}
)

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