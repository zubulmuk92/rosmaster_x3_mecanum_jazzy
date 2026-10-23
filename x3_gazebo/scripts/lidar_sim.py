#!/usr/bin/env python3
"""Lidar simulé : remplace le pilote sllidar_node (lidar RPLIDAR A1M8) quand le robot est la simulation Gazebo.

Comme sur le vrai robot (pilote Slamtec sllidar_ros2 + TF de yahboomcar_nav/laser_bringup_launch.py, mesurés le 23/10) :
  /scan dans le repère « laser », 1080 points sur 360° (réglés dans x3_gazebo/urdf/simulation.xacro), QoS fiable, +inf = pas de
  mesure ; portée minimale 0,12 m et non 0,05 (voir x3_gazebo/urdf/simulation.xacro : la pièce 3D du lidar est dans son plan),
  horodaté à l'heure système à la réception (la pile Yahboom n'utilise pas le temps simulé) ;
  repère « laser » (lacet π sous laser_link : le A1 est monté à l'envers) publié par le modèle x3_description.
Gazebo publie son lidar sur /x3_sim/scan dans laser_link (non tourné) : le tableau est décalé d'un demi-tour
pour être exprimé dans « laser » (l'angle a de « laser » = l'angle a + pi de laser_link).
"""
import math

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main():
    rclpy.init()
    node = rclpy.create_node('lidar_sim')
    pub = node.create_publisher(LaserScan, '/scan', 10)  # fiable, comme sllidar_node


    def recu(m):
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id = 'laser'
        # Comme sllidar_node (mesuré) : angle_max annoncé = pi même avec 1080 points de 2 pi / 1080 ; durée d'un tour
        m.angle_max = 3.14159265
        m.scan_time = 1.0 / 7.5
        m.time_increment = m.scan_time / len(m.ranges)
        d = len(m.ranges) // 2  # tour complet, nombre pair de points : demi-tour = moitié du tableau
        # Gazebo rend -inf sous la portée minimale ; le pilote Slamtec met +inf pour « pas de mesure »
        m.ranges = [math.inf if r == -math.inf else r for r in m.ranges[d:] + m.ranges[:d]]
        if len(m.intensities) == 2 * d:
            m.intensities = m.intensities[d:] + m.intensities[:d]
        pub.publish(m)

    node.create_subscription(LaserScan, '/x3_sim/scan', recu, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
