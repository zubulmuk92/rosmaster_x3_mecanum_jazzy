"""Simulation Gazebo du X3 : seul le matériel est simulé, tout le reste est le robot.launch.py du vrai robot.

  ros2 launch x3_gazebo simulation.launch.py [gui:=false] [camera:=true] [monde:=turtlebot3_world] [x:=-2.0 y:=-0.5]

Gazebo (monde turtlebot3_gazebo) + robot créé depuis le modèle unique x3_description (simulation:=true) + ponts
ROS <-> Gazebo (config/ponts*.yaml) + lidar_sim / camera_sim (remplacent les pilotes) + x3_bringup robot.launch.py
materiel:=simulation (x3_base avec la carte simulée, Madgwick, EKF). Horodatage à l'heure système, comme le robot.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('x3_gazebo')
    tb3 = get_package_share_directory('turtlebot3_gazebo')
    gz_sim = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    monde = PathJoinSubstitution([tb3, 'worlds', PythonExpression(["'", LaunchConfiguration('monde'), "' + '.world'"])])
    camera = LaunchConfiguration('camera')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Client graphique Gazebo'),
        DeclareLaunchArgument('camera', default_value='false', description='Caméra simulée (rendu coûteux)'),
        DeclareLaunchArgument('monde', default_value='turtlebot3_world'),
        DeclareLaunchArgument('x', default_value='-2.0'),
        DeclareLaunchArgument('y', default_value='-0.5'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(tb3, 'models')),
        # package://x3_description/meshes/... : Gazebo cherche x3_description dans ce dossier
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(get_package_share_directory('x3_description'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim), launch_arguments={
            'gz_args': ['-r -s -v2 ', monde], 'on_exit_shutdown': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_sim), launch_arguments={
            'gz_args': '-g -v2 ', 'on_exit_shutdown': 'false'}.items(), condition=IfCondition(LaunchConfiguration('gui'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('x3_bringup'), 'launch', 'robot.launch.py')),
            launch_arguments={'materiel': 'simulation', 'camera': camera}.items()),
        Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-topic', '/robot_description', '-name', 'rosmaster_x3',
                        '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'), '-z', '0.01']),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='x3_ponts',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'ponts.yaml')}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='x3_ponts_camera',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'ponts_camera.yaml')}], condition=IfCondition(camera)),
        Node(package='x3_gazebo', executable='lidar_sim.py'),
        Node(package='x3_gazebo', executable='camera_sim.py', condition=IfCondition(camera)),
    ])
