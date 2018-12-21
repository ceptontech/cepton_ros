# Cepton ROS

## Overview

This package provides ROS support for the Cepton SDK.

Before using this ROS driver, we recommend that you download CeptonViewer to test your sensor setup.

## Compatibility

Currently, this driver only works on Ubuntu (>=16.04).

## Installation

If you have not done so already, install ROS, and [create a catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Change to the catkin workspace directory.

Clone the repository.

```sh
git clone --recursive https://github.com/ceptontech/cepton_ros.git src/cepton_ros
```

Run catkin make.

```sh
catkin_make
```

Source the catkin setup script.

```sh
source devel/setup.bash
```

## Getting started

Connect the sensor's ethernet cable to the host computer (we recommend using a USB -> Ethernet adapter). The sensor IP address is of the form `192.168.*.*`, and it sends UDP broadcast packets on port 8808. The sensor will start sending packets as soon as the power is connected.

On Ubuntu, it is necessary to assign a static IP address to the host computer's Ethernet interface, e.g. IP=`192.168.0.1`, Netmask=`255.255.0.0`. This can be done through the Network Manager GUI.

First, try viewing the sensor in CeptonViewer, to ensure that it is connected properly. Then, launch the ROS demo (`roscore` must be running already).

```sh
roslaunch cepton_ros demo.launch
```

A rviz window should popup showing a sample point cloud.

To launch the driver standalone, you need to first launch the nodelet manager

```sh
roslaunch cepton_ros manager.launch
```

Then, you can launch the driver

```sh

roslaunch cepton_ros driver.launch
```

You can print a help menu for the driver launcher

```sh
roslaunch --ros-args cepton_ros driver.launch
```

### Using multiple sensors

If the `transforms_path` parameter is passed, the driver will output each sensor point cloud with a unique tf frame id.

```sh
roslaunch cepton_ros driver.launch transforms_path:=<path_to_cepton_transforms.json>
```

A sample transforms file can be found at `launch/settings/cepton_transforms.json`. The rotation is in Quaternion format `<x, y, z, w>`. The coordinate system is as follows: `+x` = right, `+y` = forward, `+z` = up.

## Capture Replay

Refer to the launch files in `tests` for examples on how to replay data from PCAP capture files.

## Troubleshooting

First, try viewing the sensor in CeptonViewer to determine if the issue is ROS or the sensor/network.

The most common issue is the host computer blocking the sensor packets. Using Wireshark, or another networking tool, check that you are receiving packets on port 8808. If you not, check your networking/firewall settings.

## Reference

### Driver nodelet

The driver nodelet is a thin wrapper around the Cepton SDK. The point type definitions can be found in `include/cepton_ros/point.hpp`.
