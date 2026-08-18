## Workflow Pengoperasian Robot Humanoid <Gandamana/>

* Manager = roslaunch op3_manager op3_manager.launch
* Gui = rosrun op3_gui_demo op3_gui_demo
* Process = rosrun kri2024 proccess_com.py
* rostopic pub /robotis/open_cr/button std_msgs/String "data: 'start'" //start //mode //reset
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

## Run Nasional Robot
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


## 📂 Struktur Folder & Node

```text
kri2027/
├── communication/          # Modul komunikasi jaringan (WiFi/RefBox)
│   ├── receivernasional.py # Melanggan data referee box nasional
│   ├── receivershooter.py  # Menerima instruksi strategi role shooter
│   ├── senderkickoff.py    # Mengirimkan status kesiapan kickoff
│   ├── sendernasional.py   # Mengirimkan telemetry data ke wasit
│   └── strategycom.py      # Manajemen koordinasi strategi multi-robot
├── ImageProcessing/
│   └── vision2027/         # Modul visi komputer berbasis kamera robot
│       └── vision.py       # Segmentasi objek (bola, gawang, robot lawan)
└── motion/                 # Modul kontrol kinematika & pergerakan
    ├── driver.py           # Driver level rendah ke sub-kontroler (OpenCR)
    ├── process.py          # State machine / algoritma sekuensial pergerakan
    ├── lapangan.py         # Pemetaan koordinat internal berbasis batas lapangan
    └── odometry.py         # Kalkulasi estimasi posisi robot ($X, Y, \theta$)

