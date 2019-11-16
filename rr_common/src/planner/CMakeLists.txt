add_library(nearest_point_cache nearest_point_cache.cpp)
add_library(bicycle_model bicycle_model.cpp)

set(planning_libs nearest_point_cache bicycle_model)

add_library(random_sample_planner random_sample_planner.cpp)
target_link_libraries(random_sample_planner ${planning_libs} ${catkin_LIBRARIES})

add_library(annealing_planner annealing_planner.cpp)
target_link_libraries(annealing_planner ${planning_libs} ${catkin_LIBRARIES})

add_library(hill_climb_planner hill_climb_planner.cpp)
target_link_libraries(hill_climb_planner ${planning_libs} ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner
        random_sample_planner
        annealing_planner
        hill_climb_planner
        ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})
