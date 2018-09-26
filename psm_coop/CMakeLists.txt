cmake_minimum_required (VERSION 2.8.3)
project (psm_coop)

add_compile_options(-std=c++11)
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
   CATKIN_DEPENDS rospy roscpp
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

set(SRC_FILES
		src/Interpolate.cpp
		src/force_control.cpp
		src/Psm.cpp
		src/Cooperative.cpp
		src/force_main.cpp
        src/path_planner.cpp
		src/psm_sub.cpp)
set(HDR_FILES
		include/psm_coop/Interpolate.h
		include/psm_coop/force_control.h
		include/psm_coop/Psm.h
		include/psm_coop/Cooperative.h
		include/psm_coop/DataType.h
		include/psm_coop/path_planner.h
		include/psm_coop/csv_writer.h
		include/psm_coop/psm_sub.h)

add_library(psm_coop  ${SRC_FILES} ${HDR_FILES})
target_link_libraries(psm_coop ${catkin_LIBRARIES})

add_executable(force_main src/force_main.cpp)
target_link_libraries(force_main psm_coop)

add_executable(Run src/Run.cpp)
target_link_libraries(Run psm_coop)

add_executable(path_node src/path_node.cpp)
target_link_libraries(path_node psm_coop)

#add_executable(talker src/talker.cpp)
#target_link_libraries(talker ${catkin_LIBRARIES})

#add_executable(force_control src/force_control.cpp)
#target_link_libraries(force_control ${catkin_LIBRARIES})

add_executable(test_interpolate src/test_interpolate.cpp)
target_link_libraries(test_interpolate psm_coop)

catkin_python_setup ()
