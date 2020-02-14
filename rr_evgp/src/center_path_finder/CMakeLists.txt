#add_library(UniformCostSearch UniformCostSearch.h UniformCostSearch.cpp)
#target_link_libraries(UniformCostSearch ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(UniformCostSearch ${catkin_EXPORTED_TARGETS})

add_executable(center_path_finder center_path_finder.cpp)
target_link_libraries(center_path_finder UniformCostSearch ${catkin_LIBRARIES} ${OpenCV_LIBS})
