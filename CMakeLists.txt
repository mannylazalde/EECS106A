cmake_minimum_required(VERSION 2.8.3)
project(mw_udp_communicator)

# Use C++ 11
set(CMAKE_CXX_STANDARD 11)

# Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  message_generation
  nodelet
  rosconsole
  roscpp
  roscpp_serialization
  rostime
  sensor_msgs
  tf2
  tf2_ros
  
)

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    include
    # ${EIGEN3_INCLUDE_DIR}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
  DEPENDS
    #Eigen3
  CFG_EXTRAS
    # ${PROJECT_NAME}-extras.cmake
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${EIGEN3_INCLUDE_DIR}
)

# Declare CPP libraries
add_library(${PROJECT_NAME}
  src/cassie_out_t.cpp
  src/cassie_user_in_t.cpp
  src/udp_proto.cpp
)

# Declare CPP executables
add_executable(
  udp_communicator
  src/udp_communicator.cpp
)

# Specify libraries to link libraries or executables target against
target_link_libraries(
  udp_communicator
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(
  TARGETS ${PROJECT_NAME} udp_communicator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark CPP header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

# Mark other files for installation
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

# if(CATKIN_ENABLE_TESTING)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

# Add gtest based CPP test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test
#   test/test_grid_map_core.cpp
#   test/GridMapMathTest.cpp
#   test/GridMapTest.cpp
#   test/GridMapIteratorTest.cpp
#   test/LineIteratorTest.cpp
#   test/EllipseIteratorTest.cpp
#   test/SubmapIteratorTest.cpp
#   test/PolygonIteratorTest.cpp
#   test/PolygonTest.cpp
#   test/EigenPluginsTest.cpp
#   test/SpiralIteratorTest.cpp
#   test/SlidingWindowIteratorTest.cpp
#   )
# endif()

# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
