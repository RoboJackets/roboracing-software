add_executable(cone_connection_node cone_connection_node.cpp)
target_link_libraries(cone_connection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_cone_connection test_cone_connection.cpp)
target_link_libraries(test_cone_connection ${catkin_LIBRARIES})

add_executable(cone_parsing lidar_cone_parsing.cpp)
target_link_libraries(cone_parsing ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
