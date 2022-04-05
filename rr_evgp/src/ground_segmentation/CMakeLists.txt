add_executable(ground_segmentation
    ground_segmentation.cpp
    ground_segmentation.hpp
    ground_segmenter/ground_segmenter.cpp
    ground_segmenter/segment.cpp
    ground_segmenter/bin.cpp
)

target_link_libraries(ground_segmentation ${catkin_LIBRARIES})