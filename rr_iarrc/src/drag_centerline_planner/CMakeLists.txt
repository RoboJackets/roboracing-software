find_package( OpenCV REQUIRED ) # locate OpenCV in system
include_directories( ${OpenCV_INCLUDE_DIRS} ) # provide library headers

add_library(PID PID.h PID.cpp)

add_executable(drag_centerline_planner drag_centerline_planner.cpp)
target_link_libraries(drag_centerline_planner PID ${catkin_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries( drag_centerline_planner  )

add_executable(drag_centerline_planner_contours drag_centerline_planner_contours.cpp)
target_link_libraries(drag_centerline_planner_contours PID ${catkin_LIBRARIES} ${OpenCV_LIBS})
