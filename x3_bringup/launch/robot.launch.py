# SPDX-License-Identifier: Apache-2.0
"""Démarrage du X3 : modèle, pilote de la carte STM32, localisation (Madgwick + EKF) et, sur le vrai robot, lidar et caméra.

  ros2 launch x3_bringup robot.launch.py                        vrai robot (Raspberry Pi), démarré par systemd
  ros2 launch x3_bringup robot.launch.py materiel:=simulation   lancé par x3_gazebo simulation.launch.py

Identique dans les deux cas sauf le matériel : carte STM32 (x3_base carte:=serie) ou carte simulée ; pilotes du lidar
et de la caméra (vrai robot) ou lidar_sim / camera_sim (lancés par x3_gazebo).
Lidar : pilote officiel Slamtec, lanceur A1 (115200 bauds, mode Sensitivity), /dev/rplidar.
Caméra : pilote officiel Orbbec, Astra Pro Plus, infrarouge coupé, repères donnés par le modèle (publish_tf:=false).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def lanceur(paquet, nom):
    # Chemin résolu seulement si le lancement inclus est utilisé : la simulation ne demande pas les pilotes du robot
    return PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare(paquet), 'launch', nom]))


def generate_launch_description():
    config = os.path.join(get_package_share_directory('x3_bringup'), 'config')
    materiel = LaunchConfiguration('materiel')
    simulation = PythonExpression(["'true' if '", materiel, "' == 'simulation' else 'false'"])
    reel = PythonExpression(["'", materiel, "' == 'reel'"])
    return LaunchDescription([
        DeclareLaunchArgument('materiel', default_value='reel', choices=['reel', 'simulation']),
        DeclareLaunchArgument('camera', default_value='true', description='Caméra (pilote Orbbec ou caméra simulée)'),
        IncludeLaunchDescription(lanceur('x3_description', 'description.launch.py'),
                                 launch_arguments={'simulation': simulation, 'camera': LaunchConfiguration('camera')}.items()),
        Node(package='x3_base', executable='base_node',
             parameters=[{'carte': PythonExpression(["'simulation' if '", materiel, "' == 'simulation' else 'serie'"])}]),
        Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node',
             parameters=[os.path.join(config, 'imu_filter.yaml')]),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(config, 'ekf.yaml')], remappings=[('/odometry/filtered', '/odom')]),
        IncludeLaunchDescription(lanceur('sllidar_ros2', 'sllidar_a1_launch.py'),
                                 launch_arguments={'serial_port': '/dev/rplidar'}.items(), condition=IfCondition(reel)),
        IncludeLaunchDescription(
            lanceur('orbbec_camera', 'astra_pro_plus.launch.py'),
            launch_arguments={'enable_ir': 'false', 'publish_tf': 'false'}.items(),
            condition=IfCondition(PythonExpression([reel, " and '", LaunchConfiguration('camera'), "' == 'true'"]))),
    ])
