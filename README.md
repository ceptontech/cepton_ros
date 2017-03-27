# Cepton ROS

## Overview

This package provides ROS support for the Cepton sensor.

- Each sensor publishes PointCloud2 messages on its own topic.
- The cepton point type definition can be found in `include/cepton_pcl/point.hpp`. It has the following fields:
  - timestamp (uint64_t)
  - x, y, z (float)
  - intensity (float)
- The sensor topics are named `cepton_point_cloud_<sensor_id>` (e.g. sensor 1 uses topic `cepton_point_cloud_1`)
- The sensor transform frames are named `cepton_frame_<sensor_id>` (e.g. sensor 1 uses frame `cepton_frame_1`).

## Beginner's guide

If you have not done so already, install ROS, and create a catkin workspace: <http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment>

Change to the catkin workspace directory.

Clone the cepton code

    $ git clone git@github.com:ceptontech/cepton_ros.git src/cepton

Build the code

    $ catkin_make

### Run roscore

In a separate terminal, run roscore

    $ roscore

### Run the cepton node

In a separate terminal, run the cepton node

    $ rosrun cepton cepton-node

If a sensor is connected, you should see the following message: "sensor connected: <sensor_id>".

### Publish a transform frame

In a separate terminal, run a static transform publisher (replace `<sensor_id>` with the sensor id)

    $ rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cepton_frame_<sensor_id> 10

### Run rviz

In a separate terminal, run rviz

    $ rosrun rviz rviz

Add the `cepton_point_cloud_<sensor_id>` topic to the viewer.
