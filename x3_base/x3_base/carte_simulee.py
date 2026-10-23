# SPDX-License-Identifier: Apache-2.0
"""Carte STM32 simulée : remplace la Rosmaster_Lib de Yahboom quand le robot est la simulation Gazebo.

Mêmes noms de fonctions et mêmes unités que la vraie bibliothèque (seules celles qu'appelle base_node existent) :
base_node exécute le même code avec l'une ou l'autre carte (paramètre carte : serie / simulation).

  vraie carte                          ici
  ----------------------------------   -----------------------------------------------
  liaison série /dev/myserial          topics Gazebo /x3_sim/... (pont ros_gz)
  vitesse mesurée (encodeurs)          /x3_sim/odom (vitesse des roues simulées)
  gyroscope, accéléromètre             /x3_sim/imu
  set_car_motion -> moteurs            /x3_sim/cmd_vel

Facteur temps réel : Gazebo ralentit quand le PC est chargé (temps simulé < temps système). base_node et le filtre
Madgwick intègrent les vitesses sur des intervalles d'horloge SYSTÈME, comme sur le vrai robot ; des vitesses en m/s
de temps simulé leur feraient compter 1/facteur de trop (mesuré : facteur 0,7 -> +43 %, AMCL perdu de 1,7 m).
Vitesses des roues et gyroscope sont donc ramenées en m/s de temps système (multipliées par le facteur mesuré sur
l'horodatage simulé de /x3_sim/imu). Accéléromètre inchangé (seule la gravité y sert). Pas d'abonnement à /clock :
publié à 1 kHz en mode fiable, un lecteur Python trop lent bloque toute la simulation.
"""
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class Rosmaster:

    def __init__(self, car_type=1, com='/dev/myserial', delay=.002, debug=False):
        # À zéro tant que la simulation n'a rien envoyé, comme la vraie bibliothèque au démarrage
        self.__ax, self.__ay, self.__az = 0.0, 0.0, 0.0
        self.__gx, self.__gy, self.__gz = 0.0, 0.0, 0.0
        self.__vx, self.__vy, self.__vz = 0.0, 0.0, 0.0
        # rclpy est déjà initialisé par base_node
        self.__node = rclpy.create_node('carte_simulee')
        self.__cmd = self.__node.create_publisher(Twist, '/x3_sim/cmd_vel', 10)
        self.__node.create_subscription(Imu, '/x3_sim/imu', self.__imu, qos_profile_sensor_data)
        self.__node.create_subscription(Odometry, '/x3_sim/odom', self.__odom, qos_profile_sensor_data)
        # Facteur temps réel (temps simulé / temps système) sur la dernière demi-seconde ; 1 avant la première mesure
        self.__horloge = deque()
        self.__facteur = 1.0

    def __imu(self, m):
        c = self.__horloge
        c.append((time.monotonic(), m.header.stamp.sec + m.header.stamp.nanosec * 1e-9))
        while c[-1][0] - c[0][0] > 0.5:
            c.popleft()
        if c[-1][0] - c[0][0] > 0.2:
            self.__facteur = (c[-1][1] - c[0][1]) / (c[-1][0] - c[0][0])
        self.__ax, self.__ay, self.__az = m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z
        self.__gx, self.__gy, self.__gz = m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z

    def __odom(self, m):
        t = m.twist.twist
        # La carte transmet les vitesses en entiers de mm/s (ou mrad/s) : même résolution
        self.__vx, self.__vy, self.__vz = (round(v, 3) for v in (t.linear.x, t.linear.y, t.angular.z))

    # Vraie bibliothèque : fil qui lit la liaison série. Ici : fil qui reçoit les topics de la simulation.
    def create_receive_threading(self):
        executeur = SingleThreadedExecutor()
        executeur.add_node(self.__node)
        threading.Thread(target=executeur.spin, daemon=True).start()

    def set_car_type(self, car_type):
        pass  # un seul robot simulé, le X3

    def set_car_motion(self, v_x, v_y, v_z):
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(v_x), float(v_y), float(v_z)
        self.__cmd.publish(t)

    def get_accelerometer_data(self):
        return self.__ax, self.__ay, self.__az

    def get_gyroscope_data(self):
        f = self.__facteur
        return self.__gx * f, self.__gy * f, self.__gz * f

    def get_magnetometer_data(self):
        return 0, 0, 0  # pas de magnétomètre simulé (inutilisé : use_mag false)

    def get_motion_data(self):
        f = self.__facteur
        return tuple(round(v * f, 3) for v in (self.__vx, self.__vy, self.__vz))

    def get_battery_voltage(self):
        return 12.0  # batterie 3S chargée

    def get_version(self):
        return 0.0  # pas de carte : 0 signale la simulation

    def set_colorful_effect(self, effect, speed=255, parm=255):
        pass  # pas de LED simulée

    def set_beep(self, on_time):
        pass  # pas de buzzer simulé
