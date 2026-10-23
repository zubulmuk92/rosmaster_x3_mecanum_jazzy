#!/usr/bin/env python3
"""Caméra Astra simulée : remplace le pilote de la caméra Orbbec (Astra Pro Plus) quand le robot est la simulation Gazebo.

Gazebo publie sa caméra couleur et sa caméra de profondeur sous /x3_sim/camera/... (pont ros_gz,
x3_gazebo/config/ponts_camera.yaml, avec camera:=true). Ce programme les republie comme le pilote :
  /camera/color/image_raw + camera_info      rgb8, repère camera_color_optical_frame
  /camera/depth/image_raw + camera_info      16UC1 en millimètres (0 = pas de mesure), camera_depth_optical_frame
  /camera/depth/points                       nuage XYZ calculé depuis la profondeur, camera_depth_optical_frame
camera_link vient de l'URDF Yahboom (pile). Tout est horodaté à l'heure système à la réception, comme /scan
(lidar_sim.py) : la pile Yahboom n'utilise pas le temps simulé. Image et camera_info d'une même vue portent la même date.
QoS par défaut (fiable), comme le pilote (color_qos, depth_qos, point_cloud_qos = default).
Calibrations (camera_info) et décalage de la couleur (2,5 cm) = valeurs du pilote sur le vrai robot (23/10), réglées
dans x3_gazebo/urdf/simulation.xacro ; profondeur utile 0,6 à 8 m.
Fréquences comme le pilote réel (23/10) : couleur et profondeur 30 i/s, nuage ~10 Hz. Pas de flux infrarouge
(coupé aussi sur le robot, enable_ir:=false).
"""
import struct

import numpy as np
import rclpy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

QOS = 10


def main():
    rclpy.init()
    node = rclpy.create_node('camera', namespace='/camera')  # même nom que le pilote : /camera/camera
    pub = {n: node.create_publisher(t, '/camera/' + n, QOS) for n, t in (
        ('color/image_raw', Image), ('color/camera_info', CameraInfo),
        ('depth/image_raw', Image), ('depth/camera_info', CameraInfo), ('depth/points', PointCloud2))}
    infos = {}

    # Repères de la caméra : publiés par le modèle x3_description (comme sur le vrai robot), plus ici

    def info(cle, stamp):
        i = infos.get(cle)
        if i:
            i.header.stamp = stamp
            pub[cle + '/camera_info'].publish(i)

    def couleur(brut):
        # Image couleur republiée sans la décoder : seuls les 8 octets de la date changent (CDR petit-boutiste :
        # 4 octets d'en-tête, puis header.stamp.sec int32 et nanosec uint32)
        stamp = node.get_clock().now().to_msg()
        b = bytearray(brut)
        struct.pack_into('<iI', b, 4, stamp.sec, stamp.nanosec)
        pub['color/image_raw'].publish(bytes(b))
        info('color', stamp)

    rayons = {}  # (largeur, hauteur, K) -> directions des pixels, calculées une fois
    dernier_nuage = [0]  # date (ns) du dernier nuage publié

    def profondeur(m):
        stamp = node.get_clock().now().to_msg()
        z = np.frombuffer(m.data, dtype=np.float32).reshape(m.height, m.width)
        valide = np.isfinite(z) & (z > 0)
        mm = np.where(valide, np.rint(z * 1000.0), 0).astype(np.uint16)
        im = Image(header=m.header, height=m.height, width=m.width, encoding='16UC1', is_bigendian=0,
                   step=m.width * 2, data=mm.tobytes())
        im.header.stamp = stamp
        pub['depth/image_raw'].publish(im)
        info('depth', stamp)

        # Nuage de points calculé seulement s'il est lu, et au plus 10 fois par seconde : le vrai pilote, limité par
        # son calcul, le publie à ~10,5 Hz (mesuré sur le robot le 23/10) alors que la profondeur est à 30 i/s
        i = infos.get('depth')
        maintenant = node.get_clock().now().nanoseconds
        if i is None or pub['depth/points'].get_subscription_count() == 0 or maintenant - dernier_nuage[0] < 95_000_000:
            return
        dernier_nuage[0] = maintenant
        cle = (m.width, m.height, tuple(i.k))
        if cle not in rayons:
            fx, cx, fy, cy = i.k[0], i.k[2], i.k[4], i.k[5]
            u, v = np.meshgrid(np.arange(m.width, dtype=np.float32), np.arange(m.height, dtype=np.float32))
            rayons.clear()
            rayons[cle] = ((u - cx) / fx, (v - cy) / fy)
        rx, ry = rayons[cle]
        zn = np.where(valide, z, np.nan).astype(np.float32)
        # x, y, z + 4 octets de remplissage (point_step 16), comme le pilote (mesuré sur le robot le 23/10)
        xyz = np.stack((rx * zn, ry * zn, zn, np.zeros_like(zn)), axis=-1)
        nuage = PointCloud2(header=im.header, height=m.height, width=m.width, is_bigendian=False,
                            point_step=16, row_step=16 * m.width, is_dense=False, data=xyz.tobytes(),
                            fields=[PointField(name=n, offset=4 * k, datatype=PointField.FLOAT32, count=1)
                                    for k, n in enumerate('xyz')])
        pub['depth/points'].publish(nuage)

    def garder(cle):
        def cb(i):
            infos[cle] = i
        return cb

    node.create_subscription(CameraInfo, '/x3_sim/camera/color/camera_info', garder('color'), QOS)
    node.create_subscription(CameraInfo, '/x3_sim/camera/depth/camera_info', garder('depth'), QOS)
    node.create_subscription(Image, '/x3_sim/camera/color/image_raw', couleur, QOS, raw=True)
    node.create_subscription(Image, '/x3_sim/camera/depth/image_raw', profondeur, QOS)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
