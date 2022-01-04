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

Prerequisites
---------------------
This guide will assume that you are running Ubuntu 20.04. If you do not have Ubuntu 20.04, we recommend dual-booting your computer. You can find a guide to dual boot by using your search engine of choice.

Step 1 - ROS2 Galactic Installation
-----------------------------------

This guide will follow https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html.

.. code-block::

    sudo apt update && sudo apt install locales 
    sudo locale-gen en_US en_US.UTF-8 
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 export LANG=en_US.UTF-8
    sudo apt install software-properties-common 
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl gnupg lsb-release
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
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

.. note::

    You should notice that this clone is different from a normal clone. We append ``-b ros2/main`` which tells git to only download one branch,
    the ``ros2/main`` branch. This is important because while we are in this transition period between ROS1 and ROS2 the default branch is ``master``
    while the branch you will most likely be working on is actually ``ros2/main``.
    Then we have ``--recursive``. This is an interesting feature of git. There are certain pieces of code that we rely on that are not published
    on some platform such as rosdep (the default package manager for ROS) or pip (the python package manager). When that occurs, we opt
    to grab the code directly from the source, another git repository. In order to add a git repository inside of another git repository, we use
    something called ``submodules``. This flag iterates through the repository and clones any submodules for you. If you do not add this ``--recursive`` flag, 
    you will need to run this command instead:

    ``git submodule update --init``

    But we don't need to run this since we added ``--recursive``. We place all of these submodules inside of the ``third_party`` directory so you
    can explore there if you are interested. If you ever see an error that includes code you don't think you changed, try running ``git submodule update``
    because maybe somebody added a new submodule.

Step 5 - Install Dependencies
-----------------------------

.. code-block::

    cd ~/roboracing_ws 
    sudo apt install -y python3-colcon-common-extensions python3-rosdep 
    rosdep install --from-path src --ignore-src -y
    rosdep init
    rosdep update

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
    sudo apt install python3 python3-pip
    pip install -U sphinx
    pip install -U furo

Step 2 - Install PlantUML
-------------------------

.. code-block::

    sudo apt install default-jre
    sudo apt install graphviz
    pip install sphinxcontrib-plantuml

Download most recent version of plantuml from: https://github.com/plantuml/plantuml/releases/latest.
Download the one that has no suffix, meaning NOT -javadoc or -sources. It is probably the third one on the list.

Download the file into ``/home/<username>/java/`` and rename the file to plantuml.jar. The download path must be: ``/home/<username>/java/plantuml.jar``.
To create a new directory in the command line you can run ``mkdir ~/java``. To move the file you can run ``mv original_file_name ~/java/plantuml.jar``.
You can find more information about ``mkdir`` and ``mv`` by reading the man-pages (short for manual). For example, run: ``man mkdir``. To learn about man you
can even run ``man man``!

.. note::
    ``/home/<username>/`` has a nice alias on Linux which is ``~``. So instead of writing ``/home/<username>/java``
    you can write ``~/java``. This is referred to as your user home directory. The /home/ directory is configured
    to support multiple users on a single machine. If you type ``ls /home`` it will list everything inside of the
    ``/home`` directory, and you will have one entry for each user on your Linux machine. The ``~`` symbol knows
    the location of your home directory using the ``HOME`` environment variable. You can run ``echo $HOME`` to see 
    the value of the ``HOME`` environment variable.

    For some fun with this you can run ``HOME=/opt`` and then you can run ``cd ~`` and it will take you to the ``/opt``
    directory! To verify run ``pwd`` and it will show you your current path. This will be reset to ``/home/<username>`` 
    when you launch a new shell instance (eg when you open a new terminal). To learn more about what a shell is, 
    read this other article: :doc:`linux_info/bashrc`.

    You can also play with this feature on MacOS and on Powershell. Environment variables are different on Powershell so
    if you are interested in that you can check out the `Powershell Environment Variables Docs <https://docs.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_environment_variables?view=powershell-7.2>`_.


Optional:

To enable plantuml from anywhere in your file structure add the following line to your bashrc. :doc:`linux_info/bashrc`

.. code-block::

    echo alias plantuml="'java -jar /home/<username>/java/plantuml.jar'" >> /home/<username>/.bashrc

Where you replace ``<username>`` with your username. For example I would write ``/home/charlie/java/plantuml.jar``.

To ensure that it is installed correctly, run ``plantuml -testdot`` and it should output something similar to:

.. code-block::

    Dot version: dot - graphviz version 2.43.0 (0)
    Installation seems OK. File generation OK

.. note::

    If you got an error it may be because you did not re-source your ``.bashrc`` file. Anytime you make a change to the ``.bashrc``, the changes
    are not automatically applied, afterall it is just a text file and only get automatically called when you create a new shell instance. To manually
    apply changes, you need to execute the file. In order to do this, you can use the ``source`` command. So you should run:

    ``source ~/.bashrc``

Step 3 - Install Doxygen and Breathe
------------------------------------

.. code-block::
    
    sudo apt-get install doxygen
    pip install breathe

Step 4 - Build Documentation
----------------------------

.. code-block::

    cd ~/roboracing_ws/src/roboracing-software
    doxygen
    make html

Step 5 - View Documentation
----------------------------

You can now open up index.html at ``~/roboracing_ws/src/roboracing-software/docs/_build/html/index.html``
and view the documentation by running ``gio open ~/roboracing_ws/src/roboracing-software/docs/_build/html/index.html``!
``gio open`` is a general purpose tool to launch a file in the registered application, eg an ``html`` file will be opened
in you firefox, a ``txt`` file will open in ``gedit`` the default Ubuntu text editor, and so on. To view all or change all
of the defaults you can run ``gedit /usr/share/applications/defaults.list``. At least was where mine was located, I had to
hunt for it so if it is not here then you can look at `this post <https://askubuntu.com/questions/957608/where-i-find-mimeapps-list>`_.
