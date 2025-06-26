# pgm_map_creator

A Gazebo Classic plugin and ROS utility for generating occupancy `.pgm` maps from a simulated Gazebo world. These maps can be used in ROS localization and navigation pipelines.

## Background

I adapted this repository based on OSRF's [collision_map_creator_plugin](https://github.com/osrf/collision_map_creator_plugin) by Stephen Brawner. It is used in Udacity's Robotics Program with [Udacity's fork](https://github.com/udacity/pgm_map_creator). 

The package seems to have helped numerous roboticists in their work over the years. With ROS 1 going EOL I refreshed it with LLMs' help to ensure it works with Noetic.

If you are looking for ROS 2 option, @arshadlab built [gazebo_map_creator](https://github.com/arshadlab/gazebo_map_creator) with great improvements and documentation for his robotics projects.

## Environment
Tested on Ubuntu 20.04, ROS Noetic, Gazebo 11.

Easiest way to set up is using OSRF's Noetic docker image: `osrf/ros:noetic-desktop-full`. Need X11 shenanigans to bring up GUI if you need it.

```
docker run -it --rm --net=host --user $(id -u):$(id -g) --device /dev/dri --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"  --volume="/home/$(id -u):/home/$(id -u):rw"  --name ros_noetic_gazebo osrf/ros:noetic-desktop-full bash
```

## Setup

### 1. Clone into your Catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/hyfan1116/pgm_map_creator.git
cd ..
catkin_make
source devel/setup.bash
```

### 2. Add the plugin to your Gazebo world

Copy your .world file to the world/ folder if needed. Then, add the plugin line after the `<world>` tag:
```
<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
```

Example:
```
<world name="default">
  <plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>
  ...
</world>
```

## Usage

### 1. Start the Gazebo server

```
gzserver src/pgm_map_creator/world/<YOUR MAP>
```

### 2. Launch the map creation request node

```
roslaunch pgm_map_creator request_publisher.launch
```

This will send a request to the plugin to rasterize the environment and write the .pgm file to the map/ folder. A .pgm file will be generated representing the occupancy map of the specified region. It can be used directly with ROS's map_server by pairing it with a .yaml map file.

## Configuration

Edit the arguments inside the request_publisher.launch file to define:

- corners: (x,y) corner pairs in clockwise order starting from upper-left
- height: height from which to raycast downward
- resolution: map resolution in meters/pixel
- filename: output file path
- threshold: pixel threshold for marking occupied space

Example (inside request_publisher.launch):

```
<arg name="corners" default="(1,1)(5,1)(5,5)(1,5)"/>
<arg name="height" default="2.0"/>
<arg name="resolution" default="0.05"/>
<arg name="filename" default="$(find pgm_map_creator)/map/my_map"/>
<arg name="threshold" default="100"/>
```

## Acknowledgements
This package is adapted from the [collision_map_creator_plugin](https://github.com/osrf/collision_map_creator_plugin) by Stephen Brawner, developed as part of the [Gazebo Custom Messages Tutorial](https://classic.gazebosim.org/tutorials?tut=custom_messages).

The [Gazebo Perfect Map Generator](https://github.com/koenlek/ros_lemtomap/tree/154c782cf8feb9112bc928e33a59728ca2192489/st_gazebo_perfect_map_generator) also packaged original plugin for easier use within a ROS-based simulation workflow for Hydro and Indigo.
