add_library(distance_checker distance_checker.cpp)

add_library(kinematic_bicycle_model bicycle_model.cpp)

add_library(random_sample_planner random_sample_planner.cpp)
target_link_libraries(random_sample_planner kinematic_bicycle_model distance_checker ${catkin_LIBRARIES})

add_library(annealing_planner annealing_planner.cpp)
target_link_libraries(annealing_planner kinematic_bicycle_model distance_checker ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner random_sample_planner annealing_planner ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})
