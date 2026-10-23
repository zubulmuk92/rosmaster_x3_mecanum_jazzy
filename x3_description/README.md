# x3_description

Modèle unique du ROSMASTER X3 (`urdf/x3.urdf.xacro`), partagé par le vrai robot, la simulation (`x3_gazebo`
l'inclut avec `simulation:=true`) et RViz.

```bash
ros2 launch x3_description description.launch.py [roues_immobiles:=true]
```

- Repères (REP-105) : `base_footprint` → `base_link` (0,0712 m) → `laser_link` → `laser` (lidar tourné de 180°),
  `camera_link` → `camera_{color,depth}_frame` → `*_optical_frame`, `imu_link`, quatre roues (axe y).
- Formes simples aux dimensions mesurées sur la CAO du fabricant. Les pièces 3D Yahboom ne sont pas
  redistribuables : `deploy/recuperer_yahboom.sh` les copie dans `meshes/yahboom/` (ignoré par git) ; le lanceur
  les utilise alors automatiquement (vrai robot et RViz seulement).
