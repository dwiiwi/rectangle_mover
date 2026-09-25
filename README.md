# Rectangle Mover - ROS 2

Program ROS 2 untuk menggerakkan robot pada simulasi membentuk lintasan persegi panjang.

## Fitur

- Gerak maju 3 meter
- Rotasi 90 derajat
- Gerak maju 2 meter
- Rotasi 90 derajat
- Mengulangi hingga membentuk persegi panjang
- Menggunakan TF `odom -> base_footprint`
- Kontrol proporsional (slow-down) mendekati target untuk mengurangi overshoot
- Robot berhenti setelah menyelesaikan lintasan

## Menjalankan

Build package:

```bash
cd ~/ros2_ws
colcon build --packages-select rectangle_mover
source install/setup.bash
```

Jalankan node (pastikan simulasi/robot yang menyediakan TF `odom -> base_footprint` dan subscriber `cmd_vel` sudah aktif):

```bash
ros2 run rectangle_mover rectangle_node
```
