add_library(opponent_detection opponent_detection.cpp)
target_link_libraries(opponent_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(opponent_detection ${catkin_EXPORTED_TARGETS})
