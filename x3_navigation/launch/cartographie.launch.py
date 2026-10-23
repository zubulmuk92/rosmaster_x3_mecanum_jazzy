# SPDX-License-Identifier: Apache-2.0
"""Cartographie du X3 (le robot ou la simulation doit déjà tourner : robot.launch.py ou simulation.launch.py).

  ros2 launch x3_navigation cartographie.launch.py [rviz:=true]
  Sauver : ros2 run nav2_map_server map_saver_cli -f <dossier>/<nom>

slam_toolbox, mode asynchrone (config/slam_toolbox.yaml : réglages par défaut de Jazzy, portée 12 m).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('x3_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox', output='screen',
             parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml')]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
