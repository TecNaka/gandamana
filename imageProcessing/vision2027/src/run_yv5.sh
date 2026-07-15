#!/bin/bash
echo "==========================================="
echo "[INFO] Menjalankan vision_yv5.py dengan Python dari yolov5n-env"
echo "==========================================="

# Turn on virtualenv
source /home/robotis/yolov5n-env/bin/activate

# Source ROS
source /opt/ros/kinetic/setup.bash
source /home/robotis/catkin_ws/devel/setup.bash

# Pindah ke direktori script
cd "/home/robotis/catkin_ws/src/DEWO-OP3/KRI2025/ImageProcessing/v2_detection/src"

# Jalankan pakai python dari venv
/home/robotis/yolov5n-env/bin/python vision_yv5.py
