add_library(odometry_to_pose odometry_to_pose.cpp)
target_link_libraries(odometry_to_pose ${catkin_LIBRARIES})
add_dependencies(odometry_to_pose ${catkin_EXPORTED_TARGETS})
