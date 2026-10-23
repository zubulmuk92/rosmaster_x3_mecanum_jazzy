# SPDX-License-Identifier: Apache-2.0
"""Pilote de la carte d'extension STM32 du X3 (moteurs, odométrie des roues, IMU, batterie, LED, buzzer).

  ros2 run x3_base base_node --ros-args -p carte:=serie        vrai robot (Rosmaster_Lib de Yahboom, /dev/myserial)
  ros2 run x3_base base_node --ros-args -p carte:=simulation   simulation (x3_base.carte_simulee, topics /x3_sim/*)

La vitesse du robot entier est envoyée à la carte (set_car_motion), dont le microprogramme calcule et régule les
quatre roues ; la carte renvoie la vitesse mesurée. Même comportement, mêmes topics et même cadence (10 Hz) que
Mcnamu_driver_X3 + base_node_X3 de Yahboom, réunis en un seul nœud.

  /cmd_vel (Twist)                  -> set_car_motion(vx, vy, wz)
  /RGBLight (Int32), /Buzzer (Bool) -> LED et buzzer
  10 Hz : /vel_raw (Twist, vitesse mesurée), /odom_raw (Odometry intégrée, odom -> base_footprint, sans TF : c'est
          l'EKF qui la publie), /imu/data_raw (Imu), /imu/mag (MagneticField), /voltage (Float32), /edition (Float32,
          version du microprogramme), /joint_states (quatre roues, angle intégré de la vitesse mesurée)

Rosmaster_Lib appartient à Yahboom et ne fait pas partie du dépôt : voir deploy/recuperer_yahboom.sh.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState, MagneticField
from std_msgs.msg import Bool, Float32, Int32

# Géométrie de x3_description : demi-empattement, demi-voie, rayon des roues (m)
LX, LY, RAYON = 0.08, 0.0845, 0.0323
ROUES = ['front_left_wheel_joint', 'front_right_wheel_joint', 'back_left_wheel_joint', 'back_right_wheel_joint']


def vitesses_roues(vx, vy, wz):
    """Vitesse de rotation des quatre roues mécanum (rad/s, positive = vers l'avant), dans l'ordre de ROUES."""
    k = LX + LY
    return [(vx - vy - k * wz) / RAYON, (vx + vy + k * wz) / RAYON,
            (vx + vy - k * wz) / RAYON, (vx - vy + k * wz) / RAYON]


def integrer(x, y, cap, vx, vy, wz, dt):
    """Pas d'odométrie : vitesses dans le repère du robot, cap pris avant le pas (comme base_node de Yahboom)."""
    return (x + (vx * math.cos(cap) - vy * math.sin(cap)) * dt,
            y + (vx * math.sin(cap) + vy * math.cos(cap)) * dt,
            cap + wz * dt)


class BaseNode(Node):

    def __init__(self):
        super().__init__('base_node')
        carte = self.declare_parameter('carte', 'serie').value
        self.repere_imu = self.declare_parameter('repere_imu', 'imu_link').value
        self.repere_odom = self.declare_parameter('repere_odom', 'odom').value
        self.repere_base = self.declare_parameter('repere_base', 'base_footprint').value
        if carte == 'simulation':
            from x3_base.carte_simulee import Rosmaster
            self.car = Rosmaster()
        else:
            from Rosmaster_Lib import Rosmaster  # Yahboom, installée à part
            self.car = Rosmaster(com=self.declare_parameter('port', '/dev/myserial').value)
        self.car.set_car_type(1)  # 1 = X3
        self.get_logger().info('carte : %s' % carte)

        self.create_subscription(Twist, 'cmd_vel', self.recu_cmd_vel, 1)
        self.create_subscription(Int32, 'RGBLight', self.recu_led, 100)
        self.create_subscription(Bool, 'Buzzer', self.recu_buzzer, 100)
        self.pub_vel = self.create_publisher(Twist, 'vel_raw', 50)
        self.pub_odom = self.create_publisher(Odometry, 'odom_raw', 50)
        self.pub_imu = self.create_publisher(Imu, 'imu/data_raw', 100)
        self.pub_mag = self.create_publisher(MagneticField, 'imu/mag', 100)
        self.pub_tension = self.create_publisher(Float32, 'voltage', 100)
        self.pub_version = self.create_publisher(Float32, 'edition', 100)
        self.pub_roues = self.create_publisher(JointState, 'joint_states', 100)

        self.x = self.y = self.cap = 0.0
        self.angles = [0.0] * 4
        self.dernier = None
        self.car.create_receive_threading()
        self.create_timer(0.1, self.publier)  # 10 Hz

    def recu_cmd_vel(self, m):
        self.car.set_car_motion(m.linear.x, m.linear.y, m.angular.z)

    def recu_led(self, m):
        for _ in range(3):  # répété comme chez Yahboom : la carte peut manquer une trame
            self.car.set_colorful_effect(m.data, 6, parm=1)

    def recu_buzzer(self, m):
        for _ in range(3):
            self.car.set_beep(1 if m.data else 0)

    def publier(self):
        maintenant = self.get_clock().now()
        stamp = maintenant.to_msg()
        vx, vy, wz = (float(v) for v in self.car.get_motion_data())
        dt = 0.0 if self.dernier is None else (maintenant - self.dernier).nanoseconds * 1e-9  # premier pas ignoré
        self.dernier = maintenant

        vel = Twist()
        vel.linear.x, vel.linear.y, vel.angular.z = vx, vy, wz
        self.pub_vel.publish(vel)

        self.x, self.y, self.cap = integrer(self.x, self.y, self.cap, vx, vy, wz, dt)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, self.repere_odom, self.repere_base
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(self.cap / 2), math.cos(self.cap / 2)
        odom.pose.covariance[0] = odom.pose.covariance[7] = odom.pose.covariance[35] = 0.001
        odom.twist.twist = vel
        odom.twist.covariance[0] = odom.twist.covariance[7] = odom.twist.covariance[35] = 0.0001
        self.pub_odom.publish(odom)

        imu = Imu()
        imu.header.stamp, imu.header.frame_id = stamp, self.repere_imu
        acc, gyro = self.car.get_accelerometer_data(), self.car.get_gyroscope_data()
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = (float(v) for v in acc)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (float(v) for v in gyro)
        self.pub_imu.publish(imu)
        mag = MagneticField()
        mag.header = imu.header
        mag.magnetic_field.x, mag.magnetic_field.y, mag.magnetic_field.z = (
            float(v) for v in self.car.get_magnetometer_data())
        self.pub_mag.publish(mag)
        self.pub_tension.publish(Float32(data=float(self.car.get_battery_voltage())))
        self.pub_version.publish(Float32(data=float(self.car.get_version())))

        for i, w in enumerate(vitesses_roues(vx, vy, wz)):
            self.angles[i] += w * dt
        roues = JointState()
        roues.header.stamp, roues.name, roues.position = stamp, ROUES, list(self.angles)
        self.pub_roues.publish(roues)


def main():
    rclpy.init()
    rclpy.spin(BaseNode())
