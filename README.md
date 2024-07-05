# pgm_map_creator
Create pgm map from Gazebo world file for ROS localization

## Environment
Tested on Ubuntu 22.04, ROS2 Humble, Boost 1.58

## Usage

### Add the package to your workspace
0. `mkdir -p colcon_ws/src && cd colcon_ws/src`
0. `git clone {package}`
0. `cd pgm_map_creator && mkdir build && cd build && cmake .. && make`
0. `cd build && my_path=$(pwd)`
0. `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${my_path}`

> 尝试使用colcon build后，生成的共享库无法被正常加载至gzserver中，改用make方式成功加载

### Add the map and insert the plugin
1. 将 world mode file 放到 world/下
2. 打开world模型文件，在\</world\>前添加一行
  `<plugin filename="libcollision_map_creator.so" name="collision_map_creator"/>`

```xml
   <!-- example -->
	<gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>34.735 -1.92201 108.948 2e-05 1.5698 1.58037</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
    <plugin filename="libcollision_map_creator.so" name="collision_map_creator" />
  </world>
</sdf>
```

### Create the pgm map file
1. Open a terminal, run gzerver with the map file
  `gzserver src/pgm_map_creator/world/<map file> --verbose`

> 可以使用gzclient查看模型文件记载结果，将启动gazebo显示
>
> 这一步如果显示 no such file or library错误，就是export没有正确加载或者使用colcon build编译结果不正确

2. Open another terminal, launch the request_publisher node
   `roslaunch pgm_map_creator request_publisher.launch`

> 注意scan_height的参数配置，需要高于模型文件的高度才可以

3. Wait for the plugin to generate map. It will be located in the map folder

## Map Properties
Currently, please update the argument value in launch/request_publisher.launch file.

## Acknowledgements
[Gazebo Custom Messages](http://gazebosim.org/wiki/Tutorials/1.9/custom_messages)
[Gazebo Perfect Map Generator](https://github.com/koenlek/ros_lemtomap/tree/154c782cf8feb9112bc928e33a59728ca2192489/st_gazebo_perfect_map_generator)
