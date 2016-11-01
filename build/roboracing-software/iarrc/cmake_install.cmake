# Install script for directory: /home/brian/catkin_ws/src/roboracing-software/iarrc

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/brian/catkin_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/brian/catkin_ws/build/roboracing-software/iarrc/catkin_generated/installspace/iarrc.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iarrc/cmake" TYPE FILE FILES
    "/home/brian/catkin_ws/build/roboracing-software/iarrc/catkin_generated/installspace/iarrcConfig.cmake"
    "/home/brian/catkin_ws/build/roboracing-software/iarrc/catkin_generated/installspace/iarrcConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/iarrc" TYPE FILE FILES "/home/brian/catkin_ws/src/roboracing-software/iarrc/package.xml")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/stoplight_watcher/cmake_install.cmake")
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/color_detector_dragrace/cmake_install.cmake")
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/color_detector_circuit/cmake_install.cmake")
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/steerer_dragrace/cmake_install.cmake")
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/steerer_circuit/cmake_install.cmake")
  include("/home/brian/catkin_ws/build/roboracing-software/iarrc/src/speed_controller/cmake_install.cmake")

endif()

