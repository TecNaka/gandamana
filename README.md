# Workflow Pengoperasian Robot Humanoid <Gandamana/>

* Manager = roslaunch op3_manager op3_manager.launch
* Gui = rosrun op3_gui_demo op3_gui_demo
* Process = rosrun kri2024 proccess_com.py
* Button start = rostopic pub/robotis/open_cr/button std_msg/String "data: 'start'"
* Motion = roslaunch op3_action_editor op3_action_editor.launch
* Driver = rosrun kri2024 driver_baru.py
* Odometry = rosrun kri2024 odometry.py
* Vision = rosrun vision_cpp vision 
* Cek button = rostopic echo /robotis/open_cr/button
* Lapangan = rosrun kri2024 lapangan.py
* Communication sender = rosrun communication com_sendernasional.py
* Communication receiver = rosrun communication com_receivernasional.py
* Game controler = rosrun protocolgc receiver8.py
* Buka code: code $(rospack find nama_file)/src

# Robot 2
* roslaunch op3_manager op3_manager.launch
* rosrun vision2027 vision_2027.py 
* rosrun communication2027 sendernasional.py
* rosrun communication2027 com_receivernasional.py
* rosrun kri2027 process.py
* rosrun kri2027 driver.py
* rosrun kri2027 lapangan.py
* rosrun protocolgc receiver8.py
* rosrun kri2026 odometry_baru_20.py 
* rostopic pub /robotis/open_cr/button std_msgs/String "data: 'start'" //start //mode //reset