# pgm_map_creator
Create pgm map from Gazebo world file for ROS localization

## Environment
Tested on Ubuntu 16.04, ROS Kinetic, Boost 1.58

## Usage

### Add the package to your workspace
0. Create a catkin workspace
1. Clone the package to the src folder
2. `catkin_make` and source `devel/setup.bash`

### Add the map and insert the plugin
1. Add your world file to world folder
2. Add this line at the end of the world file, before `</world>` tag:
`<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>`

### Create the pgm map file
1. Open a terminal, run gzerver with the map file
`gzserver src/pgm_map_creator/world/<map file>`
2. Open another terminal, launch the request_publisher node
`roslaunch pgm_map_creator request_publisher.launch`
3. Wait for the plugin to generate map. It will be located in the map folder

## Map Properties
Currently, please update the argument value in launch/request_publisher.launch file.

## Build using Docker without dealing with dependencies.

Tested on docker-ce 20.10.12

+ First install docker-ce from [here](https://docs.docker.com/engine/install/ubuntu/)

+ Clone and change directory to this folder
    ```bash
    cd pgm_map_creator
    ```

+ Build image with desired tag
    ```bash
    docker build -t <myimage> .
    ```
+ Create your container
    ```bash
    docker run -t -d --name <mycontainer> <myimage:latest>
    ```
+ Start bash shell in your container
    ```bash
    docker exec -it <mycontainer> /bin/bash
    ```
+ Source your ros distro and workspace
    ```bash
    source /home/username/.bashrc
    ```
+ Build
    ```bash
    cd /home/username/catkin_ws/ && catkin_make
    ```
+ Find your container IP address
  ```bash
  sudo docker inspect -f "{{ .NetworkSettings.IPAddress }}" <mycontainer>
  ```
+ Open up a new terminal for transfering your **.world** file to container with nautilus file manager. But make sure you have [SAMBA](https://ubuntu.com/tutorials/install-and-configure-samba#1-overview) installed on your computer.
    ```bash
    nautilus sftp://username@<container-ip-address>
    ```

## Acknowledgements
[Gazebo Custom Messages](http://gazebosim.org/wiki/Tutorials/1.9/custom_messages)
[Gazebo Perfect Map Generator](https://github.com/koenlek/ros_lemtomap/tree/154c782cf8feb9112bc928e33a59728ca2192489/st_gazebo_perfect_map_generator)

