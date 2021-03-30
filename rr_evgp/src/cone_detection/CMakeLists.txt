add_library(lidar_cone_parsing lidar_cone_parsing.cpp)
target_link_libraries(lidar_cone_parsing
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})
