cmake_minimum_required(VERSION 2.8.3)
project(node_base)
set (CMAKE_CXX_STANDARD 11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  state_machine_msgs
  tf
)

# System dependencies
find_package(Boost REQUIRED)


# catkin specific configuration
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ROSNode
  CATKIN_DEPENDS roscpp std_msgs state_machine_msgs
)

# Build

# additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

#Declare a C++ library
add_library(ROSNode
   src/ROSNode.cpp
   src/life_cycle.cpp
)

target_link_libraries(ROSNode ${catkin_LIBRARIES})
add_dependencies(ROSNode state_machine_msgs_generate_messages_cpp)
