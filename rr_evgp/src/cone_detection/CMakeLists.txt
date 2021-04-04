add_library(cone_parsing lidar_cone_parsing.cpp)
add_library(cone_connection cone_connection.cpp)
target_link_libraries(cone_parsing
        cone_connection
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})
