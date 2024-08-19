cmake_minimum_required(VERSION 3.0.2)
project(diagnostic_service_client)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs std_srvs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add an executable node
add_executable(diagnostic_service_client_node src/diagnostic_service_client_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(diagnostic_service_client_node
  ${catkin_LIBRARIES}
)

