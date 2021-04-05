add_library(UniformCostSearch UniformCostSearch.cpp)
target_link_libraries(UniformCostSearch ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(UniformCostSearch ${catkin_EXPORTED_TARGETS})
