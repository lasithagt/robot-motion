cmake_minimum_required(VERSION 2.8.3)
project(robot_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ecl_geometry
  ecl_exceptions
  geometry_msgs
  iiwa_msgs
  robot_interface
  robot_models
  #robot_motion
)

find_package(Boost REQUIRED
  system 
  thread 
  program_options
)

find_package(orocos_kdl REQUIRED)


# FIND_LIBRARY(NLOPT libnlopt.so /usr/local/lib)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   # LIBRARIES kuka_motion
   CATKIN_DEPENDS roscpp rospy std_msgs iiwa_msgs ecl_geometry robot_interface robot_motion
   # DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  #${netft_utils_INCLUDE_DIRS}
  /usr/include/eigen3
  /usr/local/include
  ${Boost_INCLUDE_DIRS} 
  src/alglib/src
)


## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(kuka_motion iiwa_msgs_generate_messages_cpp)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
MESSAGE( STATUS "   " ${orocos_kdl_INCLUDE_DIRS})

file(GLOB alglib_SRC
    "src/alglib/src/*.cpp"
)
## Declare a C++ library
add_library(kuka_motion
   src/kuka_lib.cpp
)

target_link_libraries(kuka_motion
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${netft_utils_LIBRARIES}
)

add_executable(kuka_path_test scripts/path_test.cpp src/utils.cpp src/robot_kinematics.cpp src/robot_motion_library.cpp  ${alglib_SRC})

add_executable(position_wrench scripts/position_wrench.cpp src/utils.cpp src/robot_kinematics.cpp src/robot_motion_library.cpp  ${alglib_SRC})


target_link_libraries(position_wrench
   ${catkin_LIBRARIES}
   ${robot_interface_LIBRARIES}
   kuka_motion
   ${orocos_kdl_LIBRARIES}
)

target_link_libraries(kuka_path_test
  ${catkin_LIBRARIES}
  ${robot_interface_LIBRARIES}
  kuka_motion
  ${orocos_kdl_LIBRARIES} 
)

#target_link_libraries(kuka_path_test -lnlopt)
target_link_libraries(position_wrench -lnlopt)

# Compile python bindings for python 2.7
# add_library(pybindings SHARED src/pybindings.cpp src/robot_class.cpp)
# Set target specific libraries and includes
# find_package(PythonLibs 2.7 REQUIRED)
# target_compile_options(pybindings PRIVATE -I/usr/local/include/python2.7)
# target_include_directories(pybindings PRIVATE "${PYTHON_INCLUDE_DIRS}" include ${catkin_INCLUDE_DIRS})
# Link libraries
# target_link_libraries(pybindings -Wl,--start-group ${catkin_LIBRARIES} -Wl,--end-group)
# Disable 'lib' prefix in front of shared library name
# SET_TARGET_PROPERTIES(pybindings PROPERTIES PREFIX "")

# c++ -O3 -Wall -shared -std=c++11 -fPIC `python2 -m pybind11 --includes` pybindings.cpp -o pybindings`python2-config --extension-suffix`

# catkin_install_python(PROGRAMS scripts/optical_sphere.py DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
# catkin_python_setup()


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
##install(TARGETS kuka_motion 
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##)

# install(DIRECTORY include/
#      DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kuka_motion_lib.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
