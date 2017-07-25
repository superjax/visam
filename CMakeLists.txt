cmake_minimum_required(VERSION 2.8.3)
project(visam)

SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++11 ${OpenMP_CXX_FLAGS} -O3")


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  rospy
  roscpp
)
find_package(OpenCV REQUIRED)
find_package(GTSAM)
find_package(pybind11)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES visam
  CATKIN_DEPENDS roscpp rospy nav_msgs std_msgs sensor_msgs
)

###########
## Build ##
###########
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

## Declare a pybind module this is a fancy wrapper for add_library
pybind11_add_module(visam
    src/visam.cpp
    include/visam/visam.h
)

# Don't prepend wrapper library name with lib and add to Python libs.
set_target_properties(visam PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

# Put an __init__.py file in the package location so it can be imported
file(WRITE ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}/__init__.py "")
