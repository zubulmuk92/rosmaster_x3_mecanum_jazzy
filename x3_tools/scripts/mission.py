#!/usr/bin/env python3
"""Mission de navigation répétable : enchaîne des objectifs fixes avec Nav2 (nav2_simple_commander) et mesure chaque
trajet contre la vraie pose du robot dans Gazebo. Sert de démonstration et de banc de comparaison des réglages.

À lancer pendant la simulation + la navigation sur turtlebot3_world (robot à son point de départ) :
  python3 x3_tools/scripts/mission.py <étiquette> [mission]    mission : tb3 (défaut) ou couloir
Écrit tests/resultats/mission_<étiquette>.json et affiche un bilan.

Mesures par objectif (vraie pose Gazebo, repère carte = point de départ du robot lors de la cartographie) :
  résultat, durée, distance parcourue, écart d'arrivée (position, cap), manœuvres de secours,
  écart de localisation à l'arrivée (pose estimée par AMCL / vraie pose),
  part en crabe = déplacement latéral du robot / déplacement total (0 % = toujours vers l'avant).
"""
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

ICI = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ETIQUETTE = sys.argv[1]
MISSION = sys.argv[2] if len(sys.argv) > 2 else 'tb3'
# Objectifs (x, y, cap en degrés) dans le repère de la carte, et position Gazebo du départ (= origine de la carte)
MISSIONS = {
    'tb3': {'depart_gazebo': (-2.0, -0.5), 'monde': 'default', 'objectifs': [
        ('entre les piliers', 1.45, -0.05, 0),
        ('1,1 m à gauche (crabe utile)', 1.45, 1.05, 0),
        ('case du haut, tourné à 90°', 2.55, 2.15, 90),
        ('côté droit, demi-tour', 3.6, 0.5, 180),
        ('retour au départ', 0.0, 0.0, 0),
    ]},
    # Couloir dégagé au-dessus des piliers (y = 2,3 m, piliers à y = 1,6 m, mur à y = 3,1 m) : 2,4 m en ligne droite,
    # pour mesurer la vitesse de croisière, que les trajets courts de « tb3 » ne laissent pas atteindre.
    'couloir': {'depart_gazebo': (-2.0, -0.5), 'monde': 'default', 'objectifs': [
        ('entrée du couloir haut', 0.9, 2.3, 0),
        ('couloir, aller tout droit 2,4 m', 3.3, 2.3, 0),
        ('couloir, demi-tour et retour', 0.9, 2.3, 180),
        ('retour au départ', 0.0, 0.0, 0),
    ]},
}
m = MISSIONS[MISSION]
x0, y0 = m['depart_gazebo']

vraie = {}


def gz_pose(msg):
    for p in msg.pose:
        if p.name == 'rosmaster_x3':
            q = p.orientation
            vraie['p'] = (p.position.x - x0, p.position.y - y0,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


gz = GzNode()
gz.subscribe(Pose_V, '/world/%s/pose/info' % m['monde'], gz_pose)

rclpy.init()
nav = BasicNavigator()
nav.waitUntilNav2Active(localizer='amcl')
time.sleep(1.0)


def objectif(x, y, cap):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(math.radians(cap) / 2), math.cos(math.radians(cap) / 2)
    return p


resultats = []
print('%-30s | %-9s | %6s | %7s | %8s | %7s | %7s | %7s | %s'
      % ('objectif', 'résultat', 'durée', 'trajet', 'écart', 'cap', 'crabe', 'loc.', 'secours'))
for nom, x, y, cap in m['objectifs']:
    debut, trace, secours, loc = time.time(), [vraie['p']], 0, 0.0
    nav.goToPose(objectif(x, y, cap))
    while not nav.isTaskComplete():
        f = nav.getFeedback()
        if f:
            secours = max(secours, f.number_of_recoveries)
            e = f.current_pose.pose.position  # pose estimée (AMCL) au dernier retour, robot quasi arrêté à la fin
            loc = math.hypot(e.x - vraie['p'][0], e.y - vraie['p'][1])
        if vraie['p'] != trace[-1]:
            trace.append(vraie['p'])
        time.sleep(0.05)
    res = nav.getResult()
    time.sleep(0.5)
    trace.append(vraie['p'])
    duree = time.time() - debut
    long = lat = 0.0
    for (ax, ay, ac), (bx, by, _) in zip(trace, trace[1:]):
        dx, dy = bx - ax, by - ay
        long += math.hypot(dx, dy)
        lat += abs(-dx * math.sin(ac) + dy * math.cos(ac))  # composante latérale dans le repère du robot
    fx, fy, fc = trace[-1]
    ecart = math.hypot(fx - x, fy - y)
    ecart_cap = abs(math.degrees(math.atan2(math.sin(fc - math.radians(cap)), math.cos(fc - math.radians(cap)))))
    r = {'objectif': nom, 'resultat': {TaskResult.SUCCEEDED: 'atteint', TaskResult.CANCELED: 'annulé',
                                       TaskResult.FAILED: 'ÉCHEC'}.get(res, str(res)),
         'duree_s': round(duree, 1), 'trajet_m': round(long, 2), 'ecart_m': round(ecart, 3),
         'ecart_cap_deg': round(ecart_cap, 1), 'part_crabe': round(lat / long, 3) if long > 0.05 else 0.0,
         'secours': secours, 'ecart_loc_m': round(loc, 3)}
    resultats.append(r)
    print('%-30s | %-9s | %5.1fs | %6.2fm | %7.3fm | %6.1f° | %6.0f%% | %6.3fm | %d'
          % (nom, r['resultat'], r['duree_s'], r['trajet_m'], r['ecart_m'], r['ecart_cap_deg'],
             100 * r['part_crabe'], loc, secours), flush=True)

ok = [r for r in resultats if r['resultat'] == 'atteint']
print('bilan : %d/%d atteints, durée totale %.0f s, écart moyen %.3f m, cap moyen %.1f°'
      % (len(ok), len(resultats), sum(r['duree_s'] for r in resultats),
         sum(r['ecart_m'] for r in ok) / max(len(ok), 1), sum(r['ecart_cap_deg'] for r in ok) / max(len(ok), 1)))
chemin = os.path.join(ICI, "resultats", "mission_%s.json" % ETIQUETTE)
with open(chemin, 'w') as f:
    json.dump(resultats, f, indent=1, ensure_ascii=False)
print('mesures :', chemin, flush=True)  # vidage avant os._exit, qui ne vide pas les tampons
os._exit(0)
