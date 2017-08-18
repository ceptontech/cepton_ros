# Cepton ROS

## Overview

This package provides ROS support for the Cepton LIDAR sensor.

### Driver nodelet

The driver nodelet is a thin wrapper around the Cepton driver, and publishes PointCloud2 topics for each sensor. The point type definition can be found in `include/cepton_pcl/point.hpp`. It has the following fields:

  - timestamp (uint64_t)
  - x, y, z (float)
  - intensity (float)

## Installation

If you have not done so already, install ROS, and create a catkin workspace: <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>.

Change to the catkin workspace directory.

Clone the Cepton code

    $ git clone git@github.com:ceptontech/cepton_ros.git src/cepton

Build the code

    $ catkin_make

## Getting started

Connect the sensor's ethernet cable to the computer (we recommend using a USB to Ethernet adapter). Assign a static IP address to the Ethernet interface of the form 192.168.0.\*/16 (e.g. 192.168.0.2/16). On Ubuntu, this can be done through the Network Manager GUI. The sensor IP address is 192.168.\*.\*, and it sends UDP broadcast packets on port 8808. The sensor will start sending packets as soon as the power is connected.

The best place to start, is to launch the demo (`roscore` must be running already)

    $ roslaunch cepton_ros demo_single.launch
