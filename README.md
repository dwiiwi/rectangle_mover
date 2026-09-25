# Rectangle Mover - ROS 2

Program ROS 2 untuk menggerakkan robot pada simulasi membentuk lintasan persegi panjang.

## Fitur

- Gerak maju 3 meter
- Rotasi 90 derajat
- Gerak maju 2 meter
- Rotasi 90 derajat
- Mengulangi hingga membentuk persegi panjang
- Menggunakan TF `odom -> base_footprint`
- Robot berhenti setelah menyelesaikan lintasan

## Menjalankan

Build package:

```bash
cd ~/ros2_ws
colcon build --packages-select rectangle_mover
source install/setup.bash

Jalankan Simulasi:

ros2 launch robin_bringup my_robot_gazebo.launch.py

Buka terminal lain, dan jalankan:

source ~/ros2_ws/install/setup.bash
ros2 run rectangle_mover rectangle_node
