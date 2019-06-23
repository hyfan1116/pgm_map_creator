# pgm_map_creator
Create pgm map from Gazebo world file for ROS localization

## Environment
Tested on Ubuntu 16.04, ROS Kinetic, Boost 1.58, [Protocol Buffers v2.6.1](https://github.com/protocolbuffers/protobuf/releases/tag/v2.6.1)

Install Protocol Buffers v2.6.1
- Download the zip file 
- Remove the previous protocol buffer version
- `$ cd /usr/local/include/google`
- `$ sudo rm -rf protobuf`
- Download the zip file of [Protocol Buffers v2.6.1](https://github.com/protocolbuffers/protobuf/releases/tag/v2.6.1)
- Follow the installations given [here](https://github.com/protocolbuffers/protobuf/tree/master/src):
  1. `$ ./autogen.sh`

  2. `$ ./configure`

  3. `$ make`

  4. `$ make check`

  5. `$ sudo make install`

  6. `$ sudo ldconfig`
  
## Usage

### Add the package to your workspace
1. Create a catkin workspace / Open the catkin workspace
2. Clone the package to the src folder
3. Comment the following lines in CMakeLists.txt in the msgs folder of pgm_map_creator and save it.(Edit it using gedit)
``` 
    #${PROTOBUF_IMPORT_DIRS}/vector2d.proto
    #${PROTOBUF_IMPORT_DIRS}/header.proto
    #${PROTOBUF_IMPORT_DIRS}/time.proto
```
4. `catkin_make` and `source devel/setup.bash`

### Add the map and insert the plugin
1. Add your world file to world folder in the pgm_map_creator folder
2. Add this line at the end of the world file, before `</world>` tag:
`<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>`

### Create the pgm map file
1. Run `gazebo /home/jit/catkin_ws/src/pgm_map_creator/world/<map file>` with full path to the map and check if it opens correctly. If yes, the close it and follow the next steps.
2. Run `source devel/setup.bash` in the following terminals before running the next commands to setup the environment variables 
3. Open a terminal, run gzerver with the map file (Enter the entire path to the map file)
`gzserver /home/jit/catkin_ws/src/pgm_map_creator/world/<map file>`
4. Open another terminal, launch the request_publisher node
`roslaunch pgm_map_creator request_publisher.launch`
5. Wait for the plugin to generate map. Track the progress in the gzserver terminal. It will be located in the map folder

## Map Properties
Currently, please update the argument value in launch/request_publisher.launch file.

## Acknowledgements
[Gazebo Custom Messages](http://gazebosim.org/wiki/Tutorials/1.9/custom_messages)
[Gazebo Perfect Map Generator](https://github.com/koenlek/ros_lemtomap/tree/154c782cf8feb9112bc928e33a59728ca2192489/st_gazebo_perfect_map_generator)

