cmake_minimum_required(VERSION 2.8.3)
project(ky_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE LIB_SRCS "*.cpp")
file(GLOB_RECURSE LIB_HEADERS "*.h")
list(REMOVE_ITEM LIB_SRCS "src/ky_control.cpp")
add_library(${PROJECT_NAME} ${LIB_SRCS} ${LIB_HEADERS})

add_executable(ky_test
  src/ky_control.cpp
)

add_dependencies(ky_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ky_test
  ${catkin_LIBRARIES}
)

