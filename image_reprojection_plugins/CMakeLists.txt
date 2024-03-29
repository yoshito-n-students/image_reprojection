cmake_minimum_required(VERSION 2.8.3)
project(image_reprojection_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_reprojection
  message_generation
  nav_msgs
  nodelet
  pluginlib
  roscpp
  shape_msgs
  std_msgs
  tf
)

## System dependencies are found with CMake's conventions
find_package(
  OpenCV REQUIRED COMPONENTS
  calib3d
  core
  imgproc
)
find_package(
  Boost REQUIRED COMPONENTS
  thread
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MeshStamped.msg
  PlaneStamped.msg
  SphereStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  shape_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
  LIBRARIES 
  image_reprojection_camera_model_plugins
  image_reprojection_surface_model_plugins
  image_reprojection_nodelet_plugins
  CATKIN_DEPENDS 
  camera_info_manager
  cv_bridge 
  geometry_msgs
  image_reprojection 
  message_runtime
  nav_msgs 
  nodelet
  pluginlib 
  roscpp 
  shape_msgs
  std_msgs
  tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(
  image_reprojection_camera_model_plugins
  src/camera_model_plugins.cpp
)
add_library(
  image_reprojection_surface_model_plugins
  src/surface_model_plugins.cpp
)
add_library(
  image_reprojection_nodelet_plugins
  src/nodelet_plugins.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(
  image_reprojection_surface_model_plugins
  ${image_reprojection_plugins_EXPORTED_TARGETS} 
)

## Declare a C++ executable
add_executable(
  static_mesh_publisher 
  src/static_mesh_publisher.cpp
)
add_executable(
  static_plane_publisher 
  src/static_plane_publisher.cpp
)
add_executable(
  static_sphere_publisher 
  src/static_sphere_publisher.cpp
)
add_executable(
  tf_tracking_gimbal
  src/tf_tracking_gimbal.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
  static_mesh_publisher 
  ${image_reprojection_plugins_EXPORTED_TARGETS} 
)
add_dependencies(
  static_plane_publisher 
  ${image_reprojection_plugins_EXPORTED_TARGETS} 
)
add_dependencies(
  static_sphere_publisher 
  ${image_reprojection_plugins_EXPORTED_TARGETS} 
)

## Specify libraries to link a library or executable target against
target_compile_options(
  image_reprojection_camera_model_plugins
  PRIVATE -O2
)
target_link_libraries(
  image_reprojection_camera_model_plugins
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(
  image_reprojection_surface_model_plugins
  PRIVATE -O2
)
target_link_libraries(
  image_reprojection_surface_model_plugins
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(
  image_reprojection_nodelet_plugins
  PRIVATE -O2
)
target_link_libraries(
  image_reprojection_nodelet_plugins
  ${catkin_LIBRARIES}
)
target_link_libraries(
  static_mesh_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(
  static_plane_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(
  static_sphere_publisher
  ${catkin_LIBRARIES}
)
target_link_libraries(
  tf_tracking_gimbal
  ${catkin_LIBRARIES}
)

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
# install(TARGETS image_reprojection_plugins image_reprojection_plugins_node
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
catkin_add_gtest(
  image_reprojection_plugins_unit_tests
  test/unit_tests_camera_models.cpp
  test/unit_tests_dem_surface_model.cpp
  test/unit_tests_main.cpp
  test/unit_tests_mesh_surface_model.cpp
  test/unit_tests_plane_surface_model.cpp
  test/unit_tests_sphere_surface_model.cpp
)
if(TARGET image_reprojection_plugins_unit_tests)
  target_compile_options(
    image_reprojection_plugins_unit_tests
    PRIVATE -O2
  )
  target_link_libraries(
    image_reprojection_plugins_unit_tests
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
