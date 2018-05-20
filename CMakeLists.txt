cmake_minimum_required (VERSION 2.8.3)
project (psm_coop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED COMPONENTS
	      roscpp
              rospy
              )

find_package (Eigen3 REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package (
  INCLUDE_DIRS include
  LIBRARIES psm_coop
   CATKIN_DEPENDS rospy
   #DEPENDS eigen3/Eigen
)

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

include_directories(
include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(psm_coop src/Interpolate.cpp include/Interpolate.h)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(force_control src/force_control.cpp)
target_link_libraries(force_control ${catkin_LIBRARIES})

add_executable(test_interpolate src/test_interpolate.cpp)
target_link_libraries(test_interpolate ${catkin_LIBRARIES})

catkin_python_setup ()
