add_executable(image_pcl_converter image_pcl_converter.cpp)
target_link_libraries(image_pcl_converter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
