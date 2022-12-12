import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():  
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package='amr_sensors').find('amr_sensors')
    params = os.path.join(pkg_share,'params.yaml')

    start_rgbd_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='rgbd_camera_node',
        parameters=[params]
        )

    start_lidar_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='lidar_node',
        parameters=[params]
        )

    ld.add_action(start_rgbd_node)
    ld.add_action(start_lidar_node)

    return ld