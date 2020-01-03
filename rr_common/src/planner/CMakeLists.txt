add_library(nearest_point_cache nearest_point_cache.cpp)
target_link_libraries(nearest_point_cache ${catkin_LIBRARIES})

add_library(inflation_cost inflation_cost.cpp)
target_link_libraries(inflation_cost ${catkin_LIBRARIES})

add_library(bicycle_model bicycle_model.cpp)
target_link_libraries(nearest_point_cache ${catkin_LIBRARIES})

add_library(effector_tracker effector_tracker.cpp)
target_link_libraries(effector_tracker ${catkin_LIBRARIES})

set(planning_libs nearest_point_cache inflation_cost bicycle_model effector_tracker)

add_library(annealing_optimizer annealing_optimizer.cpp)
target_link_libraries(annealing_optimizer ${catkin_LIBRARIES})

add_library(hill_climb_optimizer hill_climb_optimizer.cpp)
target_link_libraries(hill_climb_optimizer ${catkin_LIBRARIES})

add_executable(planner planner_node.cpp)
target_link_libraries(planner
        bicycle_model
        nearest_point_cache
        inflation_cost
        annealing_optimizer
        effector_tracker
        hill_climb_optimizer
        ${catkin_LIBRARIES})
add_dependencies(planner ${catkin_EXPORTED_TARGETS})
