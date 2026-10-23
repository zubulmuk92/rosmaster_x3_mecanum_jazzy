# SPDX-License-Identifier: Apache-2.0
"""Modèle du X3 publié par robot_state_publisher (/robot_description, repères fixes sur /tf_static, roues sur /tf).

  ros2 launch x3_description description.launch.py [roues_immobiles:=true] [simulation:=true [camera:=true]]

roues_immobiles:=true ajoute joint_state_publisher (roues à angle nul) quand rien ne publie /joint_states (PC sans
robot, ou robot sans carte moteur) : sinon les roues manquent dans l'arbre TF.
simulation:=true : modèle avec la physique, les capteurs et les moteurs de x3_gazebo (camera:=true : caméra simulée).
Les pièces 3D du fabricant sont utilisées automatiquement si deploy/recuperer_yahboom.sh les a installées.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('x3_description')
    cao = os.path.exists(os.path.join(share, 'meshes', 'yahboom', 'base_link_X3_sans_ecran.STL'))
    modele = ParameterValue(Command([
        'xacro ', os.path.join(share, 'urdf', 'x3.urdf.xacro'),
        ' simulation:=', LaunchConfiguration('simulation'), ' camera:=', LaunchConfiguration('camera'),
        ' pieces_yahboom:=', LaunchConfiguration('pieces_yahboom')]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('roues_immobiles', default_value='false'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('camera', default_value='false'),
        DeclareLaunchArgument('pieces_yahboom', default_value=str(cao).lower()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': modele}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             condition=IfCondition(LaunchConfiguration('roues_immobiles'))),
    ])
