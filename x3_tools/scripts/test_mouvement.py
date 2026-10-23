#!/usr/bin/env python3
"""Consignes /cmd_vel (avant, crabe, rotation : 0,2 m/s ou 0,5 rad/s pendant 3 s) puis comparaison de /odom (EKF) et
/odom_raw (x3_base) avec la vraie pose Gazebo (gz model -p). Simulation lancée, même ROS_DOMAIN_ID et GZ_PARTITION."""
import math, re, subprocess, time, rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
rclpy.init(); n = rclpy.create_node('test_mouvement'); pub = n.create_publisher(Twist, '/cmd_vel', 10); od = [None]; raw = [None]
n.create_subscription(Odometry, '/odom', lambda m: od.__setitem__(0, m), 10)
n.create_subscription(Odometry, '/odom_raw', lambda m: raw.__setitem__(0, m), 10)
def gz():
    t = subprocess.run(['gz', 'model', '-m', 'rosmaster_x3', '-p'], capture_output=True, text=True).stdout
    v = [float(x) for x in re.findall(r'-?\d+\.\d+(?:e-?\d+)?', t.split('Pose')[-1])][:6]
    return v[0], v[1], v[5]
def attendre(s):
    f = time.time() + s
    while time.time() < f: rclpy.spin_once(n, timeout_sec=0.05)
def pose_odom(src=od):
    p = src[0].pose.pose; return p.position.x, p.position.y, 2 * math.atan2(p.orientation.z, p.orientation.w)
def essai(nom, vx, vy, wz, duree):
    attendre(1.0); g0, o0, r0 = gz(), pose_odom(), pose_odom(raw)
    t = Twist(); t.linear.x, t.linear.y, t.angular.z = float(vx), float(vy), float(wz)
    f = time.time() + duree
    while time.time() < f: pub.publish(t); attendre(0.1)
    pub.publish(Twist()); attendre(1.5); g1, o1, r1 = gz(), pose_odom(), pose_odom(raw)
    dr = [r1[i] - r0[i] for i in range(3)]
    dg = [g1[i] - g0[i] for i in range(3)]; do = [o1[i] - o0[i] for i in range(3)]
    print(f"{nom:10s} Gazebo : dx {dg[0]:+.3f} dy {dg[1]:+.3f} m, cap {math.degrees(dg[2]):+6.1f}°   "
          f"| /odom : dx {do[0]:+.3f} dy {do[1]:+.3f}, cap {math.degrees(do[2]):+6.1f}° | /odom_raw : dx {dr[0]:+.3f} dy {dr[1]:+.3f}, cap {math.degrees(dr[2]):+6.1f}°")
attendre(3)
essai('avant', 0.2, 0, 0, 3.0)
essai('crabe', 0, 0.2, 0, 3.0)
essai('rotation', 0, 0, 0.5, 3.0)
