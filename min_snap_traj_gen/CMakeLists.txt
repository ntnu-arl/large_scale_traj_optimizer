cmake_minimum_required(VERSION 2.8.3)

project(min_snap_gen)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES image_receiver
 CATKIN_DEPENDS roscpp geometry_msgs nav_msgs
#  DEPENDS system_lib
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ../include
    include/thirdparty/alglib-cpp/src
)

add_executable(rectangle_node src/rectangle_node.cpp)
target_link_libraries(rectangle_node
  ${catkin_LIBRARIES}
)

file(GLOB ALGLIB_SRC ${CMAKE_CURRENT_SOURCE_DIR}/include/thirdparty/alglib-cpp/src/*.cpp)
add_executable(rounded_rectangle_node
  ${ALGLIB_SRC}
  src/rounded_rectangle_node.cpp)
target_link_libraries(rounded_rectangle_node
  ${catkin_LIBRARIES}
)
