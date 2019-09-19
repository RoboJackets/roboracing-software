add_executable(${PROJECT_NAME}_joystick_driver joystick_driver.cpp)
target_link_libraries(${PROJECT_NAME}_joystick_driver ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_joystick_driver ${catkin_EXPORTED_TARGETS})
set_target_properties(${PROJECT_NAME}_joystick_driver PROPERTIES OUTPUT_NAME "joystick_driver")

catkin_install_python(
        PROGRAMS keyboard_driver.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
