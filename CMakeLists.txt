cmake_minimum_required(VERSION 2.8.12)
project(slamdunk_obstacle_avoidance_example)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  nodelet
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  cmake_modules
)

find_package(Eigen REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp)

include_directories(${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})

add_executable(obstacle_avoidance_node ros/src/obstacle_avoidance_node.cpp)
target_compile_options(obstacle_avoidance_node PUBLIC -std=c++11)
target_link_libraries(obstacle_avoidance_node
  ${catkin_LIBRARIES}
)
add_dependencies(obstacle_avoidance_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_library(obstacle_avoidance_nodelet MODULE
  ros/src/obstacle_avoidance_nodelet.cpp
  src/Avoid.cpp)

target_include_directories(obstacle_avoidance_nodelet
  PRIVATE
  include/)

target_compile_options(obstacle_avoidance_nodelet PUBLIC -std=c++11)
target_link_libraries(obstacle_avoidance_nodelet
  ${catkin_LIBRARIES}
)
add_dependencies(obstacle_avoidance_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

install(TARGETS
  obstacle_avoidance_node
  obstacle_avoidance_nodelet
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY
  ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
