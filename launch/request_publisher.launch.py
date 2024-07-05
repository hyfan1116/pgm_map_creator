import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():  
    # 定义参数  
    save_path = os.path.join(get_package_share_directory('pgm_map_creator'), 'maps')
    print(save_path)
    map_name = 'greatroom'  
    save_folder = os.path.join(save_path, map_name)
    print(save_folder)  
    xmin = '-30'  
    xmax = '80'
    ymin = '-30'
    ymax = '30'
    scan_height = '10'
    resolution = '0.01'
  
    # 构建参数列表，用于传递给节点
    polygon_str = f"({xmin},{ymax})({xmax},{ymax})({xmax},{ymin})({xmin},{ymin})"  
  
    # 创建并配置节点  
    request_publisher = Node(  
        package='pgm_map_creator',  
        executable='request_publisher',  
        name='request_publisher',  
        output='screen',  
        arguments=[polygon_str, scan_height, resolution, save_folder]  
    )  
  
    # 返回包含所有组件的 LaunchDescription  
    return LaunchDescription([  
        request_publisher  
    ])