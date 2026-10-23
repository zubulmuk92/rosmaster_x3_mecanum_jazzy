#!/usr/bin/env python3
"""Publie la pose de départ (/initialpose, repère map) dès qu'AMCL est actif, puis s'arrête.

Paramètres x, y (m), cap (rad) ; covariance 0,25 m² en x et y, 0,07 rad² en cap.
"""
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.node import Node


def main():
    rclpy.init()
    n = Node('pose_initiale')
    x = float(n.declare_parameter('x', 0.0).value)
    y = float(n.declare_parameter('y', 0.0).value)
    cap = float(n.declare_parameter('cap', 0.0).value)
    etat = n.create_client(GetState, '/amcl/get_state')
    pub = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
    while rclpy.ok():                                   # attendre AMCL « active »
        if etat.wait_for_service(timeout_sec=1.0):
            f = etat.call_async(GetState.Request())
            rclpy.spin_until_future_complete(n, f, timeout_sec=2.0)
            if f.result() is not None and f.result().current_state.label == 'active':
                break
        rclpy.spin_once(n, timeout_sec=1.0)
    m = PoseWithCovarianceStamped()
    m.header.frame_id = 'map'
    m.pose.pose.position.x, m.pose.pose.position.y = x, y
    m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(cap / 2), math.cos(cap / 2)
    m.pose.covariance[0] = m.pose.covariance[7] = 0.25
    m.pose.covariance[35] = 0.07
    for _ in range(3):                                  # trois fois, au cas où AMCL manquerait la première
        m.header.stamp = n.get_clock().now().to_msg()
        pub.publish(m)
        rclpy.spin_once(n, timeout_sec=1.0)
    n.get_logger().info('pose initiale publiée : x %.2f y %.2f cap %.2f rad' % (x, y, cap))


if __name__ == '__main__':
    main()
