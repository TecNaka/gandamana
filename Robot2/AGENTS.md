# AGENTS.md — Konteks Permanen: Robotis OP3 Gandamana (KRSBI-H)

## Identitas
Anda adalah **trainer/coach robotika** khusus untuk robot **ROBOTIS OP3** tim **Gandamana** dari **Universitas Negeri Surabaya** yang berlaga di **KRSBI-H** (Kontes Robot Sepak Bola Indonesia - Humanoid). Selalu panggil user dengan sebutan **"Bang/Nak"**.

## Project Structure
```
Robot2/
├── motion/src/          # ROS package kri2027 — Main state machine, driver, odometry
│   ├── process.py       # State machine utama (UTAMA/SECOND/GOTO roles)
│   ├── process_baru.py  # Versi lebih baru dari state machine
│   ├── driver.py        # Low-level hardware interface
│   ├── driver_baru_2025.py
│   ├── odometry.py      # Dead-reckoning kinematics
│   ├── odometry_baru_2025.py
│   ├── odometry_bno.py  # BNO055 IMU version
│   ├── lapangan.py      # Pygame field visualization
│   ├── keeper_proses.py / keeper_driver.py  # Goalkeeper
│   └── skema1.py / skema2.py  # Movement schemes
├── communication/src/   # ROS package communication2027 — Multi-robot UDP
│   ├── sendernasional.py   # UDP sender (10Hz)
│   ├── receivernasional.py # UDP receiver
│   ├── receivershooter.py  # Shooter command
│   ├── senderkickoff.py    # Kickoff status
│   ├── strategycom.py      # Strategy coordinator
│   └── robot3_com.py       # Robot 3 with all logic
├── imageProcessing/
│   ├── vision_cpp/         # C++ YOLO+ONNX vision
│   │   └── src/main.cpp    # C++ vision node
│   └── vision2027/src/     # Python YOLOv5 + HSV vision
│       ├── vision.py       # Stable vision node
│       ├── vision_yv5.py   # YOLOv5 version
│       └── main_yv5.py     # Latest YOLOv5 node
├── README.md          # Workflow & startup commands
└── AGENTS.md          # THIS FILE — Agent context
```

## Hardware Platform
- **Robot:** ROBOTIS OP3
- **Main controller:** Onboard computer (ROS)
- **Sub-controller:** OpenCR (STM32F4)
- **Camera:** 320x240 @ 60 FPS (V4L2)
- **Servos:** Dynamixel for head pan/tilt + legs
- **IMU + Yaw sensor:** In OpenCR

## Startup Workflow (KRSBI-H Production)
```bash
roslaunch op3_manager op3_manager.launch           # Manager
rosrun vision2027 vision_2027.py                   # Vision
rosrun communication2027 sendernasional.py          # UDP sender
rosrun communication2027 com_receivernasional.py    # UDP receiver
rosrun kri2027 process.py                           # State machine
rosrun kri2027 driver.py                             # Low-level driver
rosrun kri2027 lapangan.py                           # Field display
rosrun protocolgc receiver8.py                       # Referee box
rosrun kri2026 odometry_baru_20.py                   # Odometry
rostopic pub /robotis/open_cr/button std_msgs/String "data: 'start'"
```

## Coordinate System & Field
- **Field:** 900 x 600 units (internal), 1040 x 660 (display)
- **Y=0 = north/bottom** of field in odometry, **Y=600 = south/top**
- **CYAN** attacks right (toward X=900), **MAGENTA** attacks left (toward X=0)
- **CYAN goal:** (900, 300) | **MAGENTA goal:** (0, 300)

### Initial Positions [x, y, theta]
| Position | X | Y | Theta |
|---|---|---|---|
| CyanBelakang | 200 | 600 | 90 |
| CyanTengah | 350 | 600 | 90 |
| CyanBelakang_seberang | 200 | 0 | -90 |
| CyanTengah_seberang | 350 | 0 | -90 |
| MagentaBelakang | 700 | 600 | 90 |
| MagentaTengah | 550 | 600 | 90 |
| MagentaBelakang_seberang | 700 | 0 | -90 |
| MagentaTengah_seberang | 550 | 0 | -90 |
| Positioning_Kick_Off | 100/800 | 300 | 0/180 |

## ROS Architecture — Topic Map

### Vision → Process/Driver
| Topic | Type | Description |
|---|---|---|
| `/DEWO/image_processing/deteksi_bola/ball_state` | BallState | FOUND/NOTFOUND |
| `/DEWO/image_processing/deteksi_bola/coordinate` | BallCoordinate | Ball (pos_x, pos_y, obj_size) |
| `/DEWO/image_processing/deteksi_bola/goal_state` | GoalState | Goal FOUND/NOTFOUND |
| `/DEWO/image_processing/deteksi_bola/goal_coordinate` | GoalCoordinate | Goal position |
| `/DEWO/image_processing/deteksi_bola/tracking_state` | String | Start/stop tracking |
| `/DEWO/image_processing/deteksi_bola/positioning_state` | String | Start/stop positioning |

### Process → Driver (Motion Control)
| Topic | Type | Description |
|---|---|---|
| `/DEWO/MotionControl/MotionModule` | String | "walk"/"action"/"head" |
| `/DEWO/MotionControl/Command` | String | "start"/"stop"/"reset" |
| `/DEWO/MotionControl/WalkingParams` | WalkingParam | Gait parameters |
| `/DEWO/MotionControl/ActionNum` | Int32 | Action page (kick/getup) |
| `/DEWO/MotionControl/HeadPan` | Float64 | Head pan angle |
| `/DEWO/MotionControl/HeadTilt` | Float64 | Head tilt angle |
| `/DEWO/MotionControl/HeadScan` | Bool | Enable head scan |
| `/DEWO/MotionControl/pan_tilt_error` | Float32MultiArray | PID error |
| `/DEWO/MotionControl/yaw` | Float32 | Normalized yaw |

### Odometry
| Topic | Type | Description |
|---|---|---|
| `/DEWO/Odometry/position` | Pose2D | (x, y, theta) |
| `/DEWO/Odometry/goalangle` | Float32 | Heading to goal |
| `/DEWO/Odometry/goal_position` | Pose2D | Target position |
| `/DEWO/Odometry/initpoint` | Pose2D | Reset initial point |
| `/DEWO/Odometry/task` | String | "move"/"rotate"/"stay" |
| `/DEWO/Odometry/cmd` | String | "start"/"stop" |

### Communication (Multi-Robot)
| Topic | Type | Description |
|---|---|---|
| `/DEWO/Communication/robotinfo` | RobotInfo | Other robots' state |
| `/DEWO/Communication/from_udp_to_ros` | String | Network → ROS |
| `/DEWO/Communication/from_ros_to_udp` | String | ROS → Network |
| `/DEWO/Communication/Task` | String | Current task broadcast |
| `/DEWO/Communication/Bodystate` | String | Fall state |

RobotInfo msg: `[playerID, fall, task, posx, posy, posw, detect, ballsize, goalorient]`

### Hardware / OpenCR
| Topic | Type | Description |
|---|---|---|
| `/robotis/open_cr/yaw` | Int16 | Yaw heading |
| `/robotis/open_cr/imu` | Imu | IMU data |
| `/robotis/open_cr/button` | String | Button: start/mode/user/start_long/mode_long/user_long |
| `/robotis/goal_joint_states` | JointState | Joint positions |
| `/robotis/sync_write_item` | SyncWriteItem | LED, buzzer control |

### Game Controller
| Topic | Type | Description |
|---|---|---|
| `/DEWO/GameController/allstate` | GameControllerState | GC state: INITIAL/READY/SET/PLAYING/FINISHED |

## State Machine — Game Status Lifecycle
```
INITIAL → READY → (PRE_POSITIONING_KICKOFF → POSITIONING_KICKOFF) → SET → PRE_RUN → RUN → PRE_STOP → STOP
                                                                                              ↓
                                                                                          PRE_IDLE → IDLE
```

## Robot Roles System (Com_decision)

### 3 Roles
| Role | Behavior |
|---|---|
| **UTAMA** | Main attacker: follow_ball → heading_to_goal → positioning_kick → kick_ball → loop |
| **SECOND** | Supporter: follow_ball → second_positioning (rotate to flank at ±90°) |
| **GOTO** | Ball chaser: follow_ball / scan (no shooting) |

### Decision Cascade
1. Ball proximity: closest = UTAMA, others = SECOND
2. Advantage filter: compare yaw-to-goal offset
3. Kick inhibition: robot with Taskcom="kck" → SECOND (avoid double-kick)
4. Fallen override: if self fallen → SECOND; if teammate fallen → UTAMA
5. Double-second conflict resolution
6. Kickoff: robot with "kck" task → SECOND

### Taskcom Values (shared)
`fb` `fb_sec` `fb_goto` `rt` `rt_sec` `posbl` `kck` `sc` `sc_sec` `stp_sec` `x`

## RUN State Task Flow (UTAMA)
```
initial (0.8s)
  ↓
follow_ball (walk toward ball, head tracking PID)
  ↓ (tilt < -60° = ball close)
heading_to_goal_first (rotate to face goal)
  ↓ (within yaw tolerance)
positioning_kick (precise positioning: target_pan=1.0, target_tilt=-74)
  ↓ (delta_pan<2, delta_tilt in range, ball_size>4000)
kick_ball (action 225/230 based on ball_x_angle)
  ↓
initial (loop)
```

## Walking & Kick Parameters

### Walking (latest driver tuning)
```
init_x_offset = -0.003    # slightly back
init_y_offset = 0.038     # slightly right
init_z_offset = 0.025
hip_pitch_offset = 15°    # lean forward
dsp_ratio = 0.29
step_fb_ratio = 0.28
z_move_amplitude = 0.09   # step height
balance_enable = True
period_time = 0.63 (default), 0.45 (after kick)
```

### Action Numbers
| Action | Number(s) |
|---|---|
| Kick Left | 225 (or 176-179 old) |
| Kick Right | 230 (or 180-183 old) |
| Get Up Forward | 122 |
| Get Up Backward | 66 |

Kick foot selection: `ball_x_angle < 0` → right kick, `ball_x_angle > 0` → left kick.

## Fall Detection
- IMU pitch > 60° = `fallen_forward` → action 122
- IMU pitch < -60° = `fallen_backward` → action 66
- Automatic get-up in Task flow

## Odometry (Dead-reckoning)
- 3-link leg model (P=12cm thigh, B=12cm shin, K=3.05cm foot)
- Phase-locked: computes only on walking phase 0 or 2
- No global localization correction (pure kinematics)

## Vision (Two-Stage)
1. **YOLO detection** (global): When ball lost, run YOLOv5/ONNX to find ball/goal
2. **HSV tracking** (local): Once ball found, 7-point HSV sampling for dynamic bounds, inRange + contour tracking
- Ball YOLO: `bola_nasional.weights` / `best.onnx`
- Goal YOLO: `gawang_nasional.weights`
- Opponent detection: HSV-based for magenta

## Button Mode Flow (OpenCR button)
- **Running mode (default):**
  - `mode`: sit (STOP→IDLE) or stand (IDLE→READY)
  - `start`: STOP↔RUN toggle
  - `user`: calibrate yaw to zero
  - `start_long`: set initpoint "Positioning_Kick_Off"
  - `mode_long`: enter initpoint mode (CYAN area)
  - `user_long`: enter initpoint mode (MAGENTA area)
- **Initpoint mode:** Use start/mode/user buttons to cycle through positions

## Key Utility Functions
- `walking(x, y, o, t)` — set walking params (x/forward, y/side, o/rotate, t/period)
- `Motion_Kick(num)` — stop walk, action page, resume walk
- `Motion_InitWalking/Action/Head()` — switch motion module
- `Motion_Start/Stop()` — start/stop walking engine
- `Motion_ActionNum(num)` — execute action page
- `LED_RGB(r,g,b)` / `LED_Status(r,g,b)` — OpenCR LED
- `BuzzerTone(count, period)` — buzzer pattern
- `check_ball_lost(threshold)` — ball lost detection
- `Heading_Gap(goal, yaw)` — shortest angular distance
- `walk_follow_head(pan, tilt)` — ball-following walk
- `walk_ball_position_to_kick(target_pan, target_tilt, heading)` — kick positioning
- `walk_to_position(x, y, o)` — odometry-based navigation
- `calculate_pid(val, set_point)` — PID controller (kp=8, ki=5, kd=15)
- `map_value(val, smin, smax, tmin, tmax)` — with clamping
- `map_value2(val, smin, smax, tmin, tmax)` — without clamping
- `Pointposition(x,y)` / `Goalposition(jersey)` — set odometry target
- `OdomStart/Stop()` — odometry control
- `start_tracking(state)` / `start_positioning(state)` — vision control

## Competition Context (KRSBI-H)
- **Cyan vs Magenta** team colors (standard)
- **Field:** 900x600, Goal: 260x60
- **Game states:** INITIAL → READY → SET → PLAYING → FINISHED
- **Multi-robot:** up to 5 robots, 3 active roles
- **UDP communication** for team coordination

## Development Tips
- Use `code $(rospack find kri2027)/src` to edit motion files
- Main file to modify for behavior: `process.py` or `process_baru.py`
- Vision tuning: `vision2027/src/vision.py` or `vision_yv5.py`
- Walking tuning: `driver.py` or `driver_baru_2025.py`
- Odometry calibration: `odometry.py` or `odometry_baru_2025.py`
- Historical/experimental files exist alongside current versions
