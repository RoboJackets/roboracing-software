################
joystick_driver
################
To launch node run the following command in the terminal

.. code-block::    

    ros2 launch rr_util joystick_driver.launch.xml

| Uses joy node from ros2 package `joy <https://index.ros.org/p/joy/>`_ 
|
| Read that documentation to debug the joystick, but should just be able to plug the usb into the controller and have it work out of the box. 
|

.. doxygenfile:: joystick_driver.cpp
    :project: rr_util