#!/bin/bash

# 1. Source lingkungan ROS Anda
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 2. Jalankan op3_manager di latar belakang (tambahkan simbol &)
roslaunch op3_manager op3_manager.launch & #manager
sleep 3 #delay
rosrun kri2024 driver_baru.py & #driver
sleep 3 #delay
rosrun kri2024 process_com2.py & #process
sleep 3 #delay
rosrun kri2024 odometry.py & #odometry
sleep 3 #delay
rosrun vision_cpp vision vision.cpp & #vision
sleep 3 #delay
rosrun communication com_sendernasional.py  & #sender
sleep 3 #delay
rosrun communication com_receivernasional.py & #receiver
sleep 3 #delay
rosrun protocolgc receiver8.py & #receiverGameController
sleep 3 #delay
rosrun kri2024 lapangan.py #lapangan 

# Teks
echo "========================================="
echo "SEMUA PROGRAM NASIONAL BERHASIL DIRUN!"
echo "========================================="


# Menjaga skrip tetap berjalan dan memantau proses background
wait
