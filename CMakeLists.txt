cmake_minimum_required(VERSION 2.8.3)
project(rrt_planner)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  costmap_2d
  base_local_planner
  tf
  geometry_msgs
  visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pluginlib
    nav_core
    costmap_2d
    base_local_planner
)

add_library(${PROJECT_NAME}
  src/rrt_planner.cpp
)

add_dependencies(rrt_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES rrt_planner_plugin.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}) 