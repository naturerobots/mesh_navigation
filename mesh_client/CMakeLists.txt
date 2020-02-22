cmake_minimum_required(VERSION 2.8.3)
project(mesh_client)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(LVR2 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(CURL REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)

include_directories(
  include
  ${LVR_INCLUDE_DIRS}
  ${JSONCPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/mesh_client.cpp
)

link_libraries(${PROJECT_NAME}
  ${LVR_LIBRARIES}
  ${JSONCPP_LIBRARIES}
  ${CURL_LIBRARIES}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mesh_client
  DEPENDS LVR2 Eigen3 ${JSONCPP_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

