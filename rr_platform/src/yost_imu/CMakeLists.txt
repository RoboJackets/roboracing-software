add_executable(yost_imu YostLabDriver.cpp yostlab_driver_node.cpp)
target_link_libraries(yost_imu ${catkin_LIBRARIES})
add_dependencies(yost_imu ${catkin_EXPORTED_TARGETS})
