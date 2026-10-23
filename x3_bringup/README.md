# x3_bringup

Démarrage du ROSMASTER X3 : `ros2 launch x3_bringup robot.launch.py [materiel:=reel|simulation] [camera:=true|false]`.

Modèle (`x3_description`), `x3_base` (`carte:=serie` ou `simulation`), filtre Madgwick et EKF (`config/`), et sur le
vrai robot : lidar (Slamtec `sllidar_ros2`, `sllidar_a1_launch.py`, `/dev/rplidar`) et caméra (Orbbec
`OrbbecSDK_ROS2`, `astra_pro_plus.launch.py`, `enable_ir:=false publish_tf:=false`).
