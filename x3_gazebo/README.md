# x3_gazebo

Simulation Gazebo du X3 : **seul le matériel est simulé**. `ros2 launch x3_gazebo simulation.launch.py [gui:=false]
[camera:=true] [monde:=…] [x:=… y:=…]` lance Gazebo, crée le robot depuis le modèle unique (`x3_description`,
`simulation:=true`), les ponts (`config/ponts*.yaml`), `lidar_sim.py` / `camera_sim.py` (remplacent les pilotes) et
`x3_bringup robot.launch.py materiel:=simulation` (même code que le vrai robot, carte simulée de `x3_base`).
`urdf/simulation.xacro` : capteurs et plugin mécanum (inclus par le modèle). `lidar_sim` et `camera_sim` ne publient
pas de TF : le modèle les donne.

Vérifié le 23/10 (domaine isolé) : empreinte des capteurs = vrai robot (formats, repères, calibrations ; écarts voulus :
portée min lidar 0,12 m) ; mouvement (`x3_tools/scripts/test_mouvement.py`, 3 passages) : écarts /odom - Gazebo ±2 cm
et 0,1-2° en rotation, comme l'ancienne pile (±2 cm, 0,1-0,3° sur 2 passages) : pas de régression nette mesurée.
