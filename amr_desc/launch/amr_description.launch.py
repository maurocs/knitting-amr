#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro 

def generate_launch_description():
  pkg_share = FindPackageShare(package='amr_desc').find('amr_desc')
  default_model_path = os.path.join(pkg_share, 'amr_base.xacro')
  default_rviz_config_path = os.path.join(pkg_share, 'urdf_config.rviz')
  
  ld = LaunchDescription()

  rviz_config_file = LaunchConfiguration('rviz_config_file')
  robot_description_raw = xacro.process_file(default_model_path).toxml()

  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  start_joint_state_publisher_cmd = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher')


  start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    output='screen',
    parameters=[{'robot_description': robot_description_raw}]
  )

  start_joint_state_publisher_gui = Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui'
  )

  start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])

  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)

  #ld.add_action(start_rviz_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_joint_state_publisher_cmd)
  #ld.add_action(start_joint_state_publisher_gui)

  return ld
