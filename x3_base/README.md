# x3_base

Pilote de la carte d'extension STM32 du ROSMASTER X3 (`base_node`) : `/cmd_vel` → `set_car_motion(vx, vy, wz)` (le
microprogramme de la carte régule les quatre roues mécanum), vitesse mesurée → `/odom_raw`, IMU → `/imu/data_raw`,
batterie → `/voltage`, angle des roues → `/joint_states`. 10 Hz, mêmes topics que le pilote Yahboom.

```bash
ros2 run x3_base base_node --ros-args -p carte:=serie        # vrai robot : nécessite la Rosmaster_Lib de Yahboom
ros2 run x3_base base_node --ros-args -p carte:=simulation   # simulation : x3_base/carte_simulee.py
```

La `Rosmaster_Lib` (Yahboom) n'est pas redistribuable : l'installer avec `deploy/recuperer_yahboom.sh`.
Tests : `python3 -m pytest test/`.
