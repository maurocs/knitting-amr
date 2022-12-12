#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    start_base = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        FindPackageShare('amr_hw').find('amr_hw')),
        '/amr_base.launch.py'])
    )

    start_description = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        FindPackageShare('amr_desc').find('amr_desc')),
        '/amr_description.launch.py'])
    )

    start_sensors = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        FindPackageShare('amr_sensors').find('amr_sensors')),
        '/amr_sensors.launch.py'])
    )

    ld = LaunchDescription()

    ld.add_action(start_base)
    ld.add_action(start_description)
    ld.add_action(start_sensors)
    #ld.add_action(start_apriltag)

    return ld