============
Installation
============

Welcome!

This will eventually replace the existing RoboRacing Software Installation Instructions at https://wiki.robojackets.org/RoboRacing_Software_Installation_Instructions.
We are currently in the process of migrating our repository to ROS2. 

Version Information
===================
We have decided on using ROS2 Galactic due to its drastic stability increase over Foxy.
One primary example is rosbag2 having only a 30% message recording rate in Foxy, but 100% in Galactic.
Samsung published an article detailing some of the differences here_.

.. _here: https://research.samsung.com/blog/Newest-ROS2-Distribution-Galactic-Geochelone-Released.


Install ROS2 Galactic
---------------------
This guide will assume that you are running Ubuntu 20.04. If you do not have Ubuntu 20.04, we recommend dual-booting your computer. You can find a guide to dual boot by using your search engine of choice.

Step 1 - ROS2 Galactic Installation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This guide will follow https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html.

.. code-block::

    sudo apt update && sudo apt install locales sudo locale-gen en_US en_US.UTF-8 sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8
    sudo apt install software-properties-common sudo add-apt-repository universe
    sudo apt update && sudo apt install curl gnupg lsb-release sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt install ros-galactic-desktop
    


Step 2 - Get Terminator
-----------------------
We primarily use terminator instead of the default terminal since it allows for both horizontal and vertical splitting (which obviously increases overall productivity). Start up the default terminal and type:

``sudo apt install terminator``

Step 3 - Source Environment Variables
-------------------------------------
For the time being, we do not recommend placing these commands into ~/.bashrc (or equivalent) because we will be switching between ROS and ROS2. Instead, you will need to source your environment with each shell instance.

``source /opt/ros/galactic/setup.bash``



Setup Workspace
===============

Step 4 - Make File Structure
----------------------------
.. code-block::

    mkdir -p ~/roboracing_ws/src
    cd ~/roboracing_ws/src
    git clone https://github.com/RoboJackets/roboracing-software.git -b ros2/main --recursive

Step 5 - Install Dependencies
-----------------------------

.. code-block::

    cd ~/roboracing_ws 
    sudo apt install -y python3-colcon-common-extensions python3-rosdep 
    rosdep install --from-path src --ignore-src -y

Step 6 - Building the Code
--------------------------

.. code-block::

    colcon build 
    source ~/roboracing_ws/install/setup.bash

Similar to step 3, you should run ``source ~/roboracing_ws/install/setup.bash`` with every new shell instance.

Documentation
=============

Step 1 - Install Sphinx
-----------------------

.. code-block::

    sudo apt update
    sudo apt install python3-sphinx
    pip install furo

Step 3 - Install PlantUML
-------------------------

.. code-block::

    sudo apt install default-jre
    sudo apt install graphviz

Download most recent version of plantuml from: https://github.com/plantuml/plantuml/releases/latest.
Download the one that has no suffix, meaning NOT -javadoc or -sources.

Download the file into ``~/java`` and rename the file to plantuml.jar.

To enable plantuml from anywhere in your file structure add the following line to your bashrc. :doc:`linux_info/bashrc`

.. code-block::

    echo alias plantuml="'java -jar /home/<username>/java/plantuml.jar'" >> ~/.bashrc

Where you replace ``<username>`` with your username. For example I would write ``/home/charlie/java/plantuml.jar``.

To ensure that it is installed correctly, run ``plantuml -testdot`` and it should output something similar to:

.. code-block::

    Dot version: dot - graphviz version 2.43.0 (0)
    Installation seems OK. File generation OK


Step 2 - Build Documentation
----------------------------

.. code-block::

    cd docs
    make html

Step 3 - View Documentation
----------------------------

You can now open up index.html at ``docs/_build/html/index.html``
