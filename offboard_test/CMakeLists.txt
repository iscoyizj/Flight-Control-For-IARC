cmake_minimum_required(VERSION 2.8.3)
project(offb_node)

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
list(REMOVE_ITEM LIB_SRCS "src/offb_node.cpp")
add_library(${PROJECT_NAME} ${LIB_SRCS} ${LIB_HEADERS})

add_executable(offb_test
  src/offb_node.cpp
)

add_dependencies(offb_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(offb_test
  ${catkin_LIBRARIES}
)

