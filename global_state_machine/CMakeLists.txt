cmake_minimum_required(VERSION 2.8.3)
project(global_state_machine)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED
  roscpp
  std_msgs
  message_generation
)

find_package(Boost REQUIRED)

add_service_files(
   FILES
   SetGlobalState.srv
   GetGlobalState.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES SMInterface
    CATKIN_DEPENDS roscpp std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

add_library(SMInterface
   src/GlobalStateMachine.cpp
   src/SMInterface.cpp
)
add_dependencies(SMInterface global_state_machine_generate_messages_cpp)

add_executable(overseer
    src/Overseer.cpp
    src/GlobalStateMachine.cpp
    include/tinyxml2/tinyxml2.cpp
)

add_dependencies(overseer global_state_machine_generate_messages_cpp)
target_link_libraries(overseer rt ${catkin_LIBRARIES})
