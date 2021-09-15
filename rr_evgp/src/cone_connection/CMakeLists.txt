add_executable(cone_connection_node cone_connection_node.cpp)
target_link_libraries(cone_connection_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cone_connect cone_connect.cpp)
target_link_libraries(cone_connect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
