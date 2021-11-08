add_library(nearest_point_cache nearest_point_cache.cpp)

add_library(inflation_map inflation_map.cpp)
add_library(distance_map distance_map.cpp)
add_library(bicycle_model bicycle_model.cpp)

add_library(effector_tracker effector_tracker.cpp)

set(planning_libs nearest_point_cache inflation_map distance_map bicycle_model effector_tracker)

add_library(annealing_optimizer annealing_optimizer.cpp)

add_library(hill_climb_optimizer hill_climb_optimizer.cpp)

add_library(global_path global_path.cpp)

add_executable(planner planner_node.cpp)
target_link_libraries(planner
        bicycle_model
        nearest_point_cache
        inflation_map
        distance_map
        annealing_optimizer
        effector_tracker
        hill_climb_optimizer
        global_path
        )
