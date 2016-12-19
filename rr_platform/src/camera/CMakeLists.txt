add_executable(transform_tester transform_tester.cpp)
target_link_libraries(transform_tester ${catkin_LIBRARIES})

add_executable(calibration_tester calibration_tester.cpp)
target_link_libraries(calibration_tester ${catkin_LIBRARIES})

add_executable(image_transform image_transform.cpp)
target_link_libraries(image_transform ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(image_transform avc_generate_messages_cpp)

add_executable(trigger_calibration trigger_calibration.cpp)
target_link_libraries(trigger_calibration ${catkin_LIBRARIES})
add_dependencies(trigger_calibration avc_generate_messages_cpp)
