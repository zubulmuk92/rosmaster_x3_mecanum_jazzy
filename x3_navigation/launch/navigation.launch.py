"""Navigation Nav2 du X3 sur une carte (le robot ou la simulation doit déjà tourner).

  ros2 launch x3_navigation navigation.launch.py [carte:=turtlebot3_world|/chemin/carte.yaml] [vitesse:=prudent|normal|rapide]
                                                 [pose_initiale:=true x:=0 y:=0 cap:=0] [rviz:=true]

Nav2 de Jazzy (nav2_bringup bringup_launch.py, comme yahboomcar_nav navigation_dwa_launch.py) avec les réglages X3
(config/nav2_<vitesse>.yaml : MPPI, STVL sur la profondeur, collision_monitor). use_composition:=False (constaté le
22/10 : sinon échecs de démarrage). La carte doit être absente des réglages (map_server.yaml_filename) : c'est
l'argument carte qui la donne. pose_initiale : publie la pose de départ dès qu'AMCL est actif.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def nav2(context):
    pkg = get_package_share_directory('x3_navigation')
    carte = LaunchConfiguration('carte').perform(context)
    if not carte.endswith('.yaml'):
        carte = os.path.join(pkg, 'cartes', carte + '.yaml')
    reglages = os.path.join(pkg, 'config', 'nav2_%s.yaml' % LaunchConfiguration('vitesse').perform(context))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments={'map': carte, 'params_file': reglages, 'use_sim_time': 'false',
                          'use_composition': 'False', 'autostart': 'true'}.items())]


def generate_launch_description():
    pkg = get_package_share_directory('x3_navigation')
    return LaunchDescription([
        DeclareLaunchArgument('carte', default_value='turtlebot3_world'),
        DeclareLaunchArgument('vitesse', default_value='prudent', choices=['prudent', 'normal', 'rapide']),
        DeclareLaunchArgument('pose_initiale', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('cap', default_value='0.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=nav2),
        Node(package='x3_navigation', executable='pose_initiale.py', output='screen',
             parameters=[{'x': LaunchConfiguration('x'), 'y': LaunchConfiguration('y'), 'cap': LaunchConfiguration('cap')}],
             condition=IfCondition(LaunchConfiguration('pose_initiale'))),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(pkg, 'rviz', 'navigation.rviz')], condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
