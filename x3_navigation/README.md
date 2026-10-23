# x3_navigation

Cartographie et navigation Nav2 du X3 (le robot ou la simulation doit tourner) :
`ros2 launch x3_navigation cartographie.launch.py`,
`ros2 launch x3_navigation navigation.launch.py [carte:=…] [vitesse:=prudent|normal|rapide] [pose_initiale:=true]`.
Réglages `config/nav2_<vitesse>.yaml` : MPPI, STVL sur la profondeur, collision_monitor, contour réel du robot ;
issus des réglages Nav2 de Jazzy et de la configuration du constructeur, adaptés au X3 (valeurs de vitesse
recopiées dans `x3_hmi`). `config/slam_toolbox.yaml` : défauts Jazzy, portée
12 m. Cartes : `cartes/`. Vue RViz : `rviz/navigation.rviz`. `pose_initiale.py` : publie la pose de départ dès qu'AMCL est actif.

Vérifié le 23/10 (simulation, `x3_tools/scripts/mission.py`) : 5/5, 67 s, écart moyen 0,054 m, cap 2,0°
(référence avant migration : 75 s, 0,059 m, 2,7°).
