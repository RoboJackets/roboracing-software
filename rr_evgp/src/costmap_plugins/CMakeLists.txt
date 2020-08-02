add_library(rr_evgp_plugins
        binary_bayes_filter_obstacle_layer.cpp
        track_closing_layer.cpp ../april_tag_detector/april_tag_detector.cpp ../april_tag_detector/april_tag_detector.h)
target_link_libraries(rr_evgp_plugins
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})
