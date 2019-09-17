add_executable(pointcloud_combiner pointcloud_combiner.cpp)
target_link_libraries(pointcloud_combiner ${catkin_LIBRARIES} ${PCL_LIBRARIES})
