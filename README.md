# RoboRacing Software [![CircleCI](https://circleci.com/gh/RoboJackets/roboracing-software.svg?style=svg)](https://circleci.com/gh/RoboJackets/roboracing-software)

<img src="https://lh3.googleusercontent.com/0cJDyJD-12-9vKtZUMvslvm1m2w5WWF__mj_edrEzq3ndi-Gl8Hww96OJjMh_DlIoNeSdyoOzV2q7LsFSR5WRR3l05wzrToRQw29pP5Vi-76HZsuAynt7bi-0vgckzXJ3Nu7A_FZP7C0-efs-7gbflydUGS4QwtrE1An0iwFiFRGiRVu9plkYb-mX2UrFYK2Oc4RBsCvJ7Vkj7B0OrdJWBY5HTf9Me0i2oE79ZOf28UfoWw1hUXjizUjpYyr_jHlfZ2l0KH6l5jQjljhhIfqajAJfxJFUY5ihd57LuYwWSPoRCuNjwwRJ4yrrD-K24Uk8JIHkTqpGYbYyNtnHJ1RJ9uPBiiUuYZA8eYtE8kh1yuK7eRlboeG6a6JPCeaVY4eFkUekpQZKJwyu5D3X7a1mlejqYAmuXgpqHyXcDtioSdkOB3iXn57CMpuQKHSb_9b4bIqPoQa-IO3gW6mAFBkevecNNovnRoO9aqxvt0mI1P7ccayUUVgVCqLsOWCulRMq9zSj_Saqcy-108jRlXjgITjRd9oxaEWZpkTZOa11OyIEjxM9RRR8E5H7gPxvyizM_o_RCdY6vUfbh8RaG_YLhXOU8Pi5FAL7VldACEAady1BWbmu7_FzMYI95ooJvseDX7LA83y0hg5zjaTKpOS3rncqnQ6c1s4aw=w1310-h983-no" style="max-height=400px;">

This repository contains [ROS](http://ros.org) packages for the [RoboJackets](http://robojackets.org) RoboRacing team.

[![Software Lead](https://img.shields.io/badge/Software%20Lead-Evan%20Bretl-blue.svg)](https://github.com/ebretl)

[![Project Manager](https://img.shields.io/badge/Project%20Manager-Varun%20Madabushi-blue.svg)](https://github.com/varunm99)

[![Maintainer](https://img.shields.io/badge/Maintainer-Matthew%20Barulic-blue.svg)](https://github.com/barulicm)

## Organization

Most folders in this repository are ROS packages.

**avc**: This package contains mission code for the [Sparkfun Autonomous Vehicle Challenge](http://avc.sparkfun.com).

**iarrc**: This package contains mission code for the [International Autonomous Robot Racing Challenge](http://robotracing.wordpress.com).

**rr_common**: This package contains general-purpose or mission-agnostic intelligence code.

**rr_description**: This package contains URDFs and meshes that describe the platform's layout.

**rr_gazebo**: This package contains resources for running the car in the [Gazebo](http://gazebosim.org) simulator.

**rr_platform**: This package contains code for interfacing with the various cars built and maintained by the team.

**sandbox**: This package contains utilities and non-ROS code. This includes tools for working with log files and the Arduino code for the car.

The following files and folders enable our continuous integration system.

* external
* util
* circle.yml
* config.docif

## Installation

1. Make sure you have the appropriate ROS version installed for your version of Ubuntu.

2. Clone this repo into the _src_ directory of your catkin workspace.

3. Install the remaining dependencies with the following command in your catkin workspace folder:

   ```sh
   rosdep install --from-path src --ignore-src -y
   ```

4. You should now be able to build the project with `catkin_make`.

## Simulation

You can get started with the RoboRacing code base right away by launching our simulator!

The following command will load our platform in the Sparkfun AVC track:
```
roslaunch rr_gazebo macaroni_avc.launch
```
Then, the following command will start our race AI and drive the car around the track:
```
roslaunch avc avc.launch
```
Alternatively, you can control the car manually with a USB gamepad with this command:
```
roslaunch rr_platform joystick_driver.launch
```
