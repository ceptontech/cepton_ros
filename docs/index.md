# Introduction

Before using the Cepton ROS driver, we recommend that you download the [CeptonViewer](https://github.com/ceptontech/cepton_sdk_redist/tree/master/bin).

## Installation

If you have not done so already, install ROS, and [create a catkin workspace](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Change to the catkin workspace directory.

Clone the code

    $ git clone git@github.com:ceptontech/cepton_ros.git src/cepton

Build the code

    $ catkin_make

## Getting started

Connect the sensor's ethernet cable to the host computer (we recommend using a USB -> Ethernet adapter). The sensor IP address is of the form `192.168.*.*`, and it sends UDP broadcast packets on port 8808. The sensor will start sending packets as soon as the power is connected.

On Ubuntu, it is necessary to assign a static IP address to the host computer's Ethernet interface, e.g. IP=`19.168.0.1`, Netmask=`255.255.0.0`. This can be done through the Network Manager GUI. 

First, try viewing the sensor in CeptonViewer, to ensure that it is connected properly. Then, launch the ROS demo (`roscore` must be running already)

    $ roslaunch cepton_ros demo_single.launch

### Using multiple sensors

Multiple sensors can be viewed using the `driver_multi.launch` file. The driver will publish separate topics and transforms for each sensor.

    $ roslaunch cepton_ros driver_multi.launch transforms_path:=<transforms_file>

A sample transforms file can be found at `samples/cepton_transforms.json`. The rotation is in quaternion format `<x, y, z, w>`. The coordinate system is as follows: `+x` = right, `+y` = forward, `+z` = up.

## Reference

### Driver nodelet

The driver nodelet is a thin wrapper around the Cepton SDK. It publishes sensor messages and PointCloud2 topics for each sensor. The point type definitions can be found in `include/cepton_ros/point.hpp`.

**`cepton_ros::CeptonImagePoint`**

- timestamp (uint64_t)
- image_x, image_z (float)
- distance (float)
- intensity (float)

**`cepton_ros::CeptonPoint`**

- timestamp (uint64_t)
- x, y, z (float)
- intensity (float)

## Troubleshooting

First, try viewing the sensor in CeptonViewer to determine if the issue is ROS or the sensor/network.

The most common issue is the host computer blocking the sensor packets. Using Wireshark, or another networking tool, check that you are receiving packets on port 8808. If you not, check your networking/firewall settings.
