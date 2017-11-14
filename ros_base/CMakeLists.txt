cmake_minimum_required(VERSION 2.8.3)
project(ros_base)
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  state_machine_msgs
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS interprocess)
find_package(Boost REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ROSNode SMInterface
  CATKIN_DEPENDS roscpp std_msgs state_machine_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

#Declare a C++ library
add_library(ROSNode
   src/ROSNode.cpp
   include/StateMachine/Fault.cpp
   include/StateMachine/StateMachine.cpp
)

target_link_libraries(ROSNode ${catkin_LIBRARIES})
add_dependencies(ROSNode state_machine_msgs_generate_messages_cpp)

add_library(SMInterface
   src/GlobalStateMachine.cpp
   src/SMInterface.cpp
)

target_link_libraries(SMInterface ${catkin_LIBRARIES})
add_dependencies(SMInterface state_machine_msgs_generate_messages_cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(aadl_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(node_reference
    src/node_reference.cpp
    include/StateMachine/Fault.cpp
    include/StateMachine/StateMachine.cpp
)

add_executable(overseer
    src/Overseer.cpp
    src/GlobalStateMachine.cpp
    include/tinyxml2/tinyxml2.cpp
)
add_dependencies(overseer state_machine_msgs_generate_messages_cpp)

target_link_libraries(node_reference ${catkin_LIBRARIES})
target_link_libraries(overseer rt ${catkin_LIBRARIES})

#add_executable(ROSNode
#    src/ROSNode.cpp
#    include/StateMachine/Fault.cpp
#    include/StateMachine/StateMachine.cpp
#)

#target_link_libraries(ROSNode ${catkin_LIBRARIES})

## Declare a C++ executable
# add_executable(aadl_ros_node src/aadl_ros_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(aadl_ros_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(aadl_ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS aadl_ros aadl_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_aadl_ros.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
