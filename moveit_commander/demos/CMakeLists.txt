find_package(catkin REQUIRED
  COMPONENTS
    moveit_core
    moveit_ros_planning_interface
    tf2_eigen
    tf2_geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(pick_tutorial pick_tutorial.cpp)
target_link_libraries(pick_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS pick_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
