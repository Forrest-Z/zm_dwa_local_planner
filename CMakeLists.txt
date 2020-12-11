cmake_minimum_required(VERSION 3.0.2)
project(zm_dwa_local_planner)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

generate_dynamic_reconfigure_options(
  cfg/zmDWALocalPlanner.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES zm_dwa_local_planner
  CATKIN_DEPENDS
   base_local_planner
   dynamic_reconfigure
   nav_msgs 
   pluginlib 
   roscpp 
   sensor_msgs 
   tf2 
   tf2_ros
)

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(zm_dwa_local_planner src/zm_dwa_local_planner.cpp src/zm_dwa_local_planner_ros.cpp)
add_dependencies(zm_dwa_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(zm_dwa_local_planner ${catkin_LIBRARIES})

install(TARGETS zm_dwa_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(FILES zm_dwa_local_planner_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)