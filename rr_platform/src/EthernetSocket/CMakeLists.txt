add_library(EthernetSocket EthernetSocket.cpp)
target_link_libraries(EthernetSocket ${catkin_LIBRARIES})
add_dependencies(EthernetSocket ${catkin_EXPORTED_TARGETS})
