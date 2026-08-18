# Belajar Motion System — Gandamana Robot OP3 (KRSBI-H)

## Daftar Isi
1. [Arsitektur 4 File Motion](#1-arsitektur-4-file-motion)
2. [process.py — Otak Robot (State Machine)](#2-processpy--otak-robot-state-machine)
3. [driver.py — Tangan Kiri Robot (Hardware Interface)](#3-driverpy--tangan-kiri-robot-hardware-interface)
4. [odometry_baru_2025.py — Indra Posisi (Dead-Reckoning)](#4-odometry_baru_2025py--indra-posisi-dead-reckoning)
5. [lapangan.py — Mata Pelatih (Field Visualization)](#5-lapanganpy--mata-pelatih-field-visualization)
6. [Data Flow & Topic Map](#6-data-flow--topic-map)
7. [Parameter Walking & Cara Tuning](#7-parameter-walking--cara-tuning)
8. [Latihan & Eksperimen](#8-latihan--eksperimen)

---

## 1. Arsitektur 4 File Motion

```
+-------------------+       +-------------------+       +---------------------+
|    process.py     | ----> |     driver.py     | ----> |   Hardware OP3      |
|  (State Machine)  |       | (Low-Level Driver)|       | (OpenCR + Servos)   |
|                   |       |                   |       +---------------------+
|  - Game Status    |       |  - WalkingParam   |
|  - Role Decision  |       |  - Action Module  |
|  - Task Flow      |       |  - Head PID       |
|  - Fall Detection |       |  - Joint Publish  |
+-------------------+       +-------------------+
         |                           |
         |                           v
         |                  +---------------------+
         |                  | odometry_baru_2025  |
         |                  |  (Dead-Reckoning)   |
         |                  |                     |
         |                  |  - Forward Kinematic |
         |                  |  - Side Kinematic    |
         |                  |  - Position Calc     |
         |                  +---------------------+
         |
         v
+-------------------+
|   lapangan.py     |
| (Field Display)   |
|                   |
|  - Pygame GUI     |
|  - Draw Lapangan  |
|  - Robot Position |
+-------------------+
```

**Alur data:**
1. `process.py` baca data sensor (yaw, IMU, ball, goal) → ambil keputusan → kirim perintah gerak
2. `driver.py` terima perintah dari process → kontrol hardware (walking/action/head module)
3. `odometry_baru_2025.py` baca joint kaki dari driver → hitung posisi (x, y, theta) → kirim balik ke process
4. `lapangan.py` baca posisi dari odometry → tampilkan di layar pygame

---

## 2. process.py — Otak Robot (State Machine)

**File:** `motion/src/process.py` (1785 baris)

### 2.1 Game Status Lifecycle

Status robot mengikuti siklus pertandingan KRSBI-H:

```
INITIAL ──► READY ──► SET ──► PRE_RUN ──► RUN ──► PRE_STOP ──► STOP
              │                                                │
              │                                                ▼
              └──► PRE_POSITIONING_KICKOFF ──► POSITIONING   IDLE (duduk)
```

Dipicu oleh Game Controller (`/DEWO/GameController/allstate`):
- `STATE_INITIAL` → Status = `READY`
- `STATE_READY` → Status = `PRE_POSITIONING_KICKOFF`
- `STATE_SET` → Status = `SET` → langsung `PRE_STOP`
- `STATE_PLAYING` → Status = `PRE_RUN` → `RUN`
- `STATE_FINISHED` → Status = `PRE_STOP`

### 2.2 Tiga Role Robot (Com_decision)

Fungsi `Com_decision()` menentukan peran robot tiap siklus:

| Role | Perilaku | Taskcom |
|---|---|---|
| **UTAMA** | Striker utama: follow_ball → positioning_kick → kick | `fb`, `rt`, `posbl`, `kck`, `sc` |
| **SECOND** | Pendukung: follow_ball → second_positioning (rotasi ±90°) | `fb_sec`, `rt_sec`, `sc_sec`, `stp_sec` |
| **GOTO** | Pemburu bola (tanpa nendang) | `fb_goto`, `sc_goto` |

Kriteria keputusan (bertingkat):
1. **Jarak bola:** `ball_size >= ballsize_treshold` (380) = dekat
2. **Proximity:** Yang paling dekat = UTAMA
3. **Advantage filter:** Bandingkan `abs(yaw_fixed2)` dengan kawan
4. **Kick inhibition:** Robot dengan `Taskcom="kck"` dipaksa SECOND
5. **Fallen override:** Self fallen → SECOND; teammate fallen → UTAMA

### 2.3 Task Flow UTAMA (RUN State)

```
initial (0.8 detik)
  │  start_tracking(True), OdomStart(), Motion_Start()
  │  count_task > 2 → follow_ball
  ▼
follow_ball
  │  walk_follow_head(pan, tilt) → jalan mengikuti bola
  │  PID head tracking: pan_head → a_move (belok), tilt_head → x_move (maju)
  │  tilt_degree < -60° (bola di depan kaki) → heading_to_goal_first
  ▼
heading_to_goal_first
  │  Rotasi menghadap gawang: walk_rotate2(goal_orientation)
  │  yaw_fixed2 dalam ±5° goal_orientation → positioning_kick
  ▼
positioning_kick
  │  Posisi presisi: walk_ball_position_to_kick(target_pan=1.0, target_tilt=-74)
  │  Syarat: delta_pan < ±2, delta_tilt < 13, ball_size > 4000
  │  count_goal_found > 2 → kick_ball
  ▼
kick_ball
  │  ball_x_angle < 0 → Right Kick (action 230)
  │  ball_x_angle > 0 → Left Kick (action 225)
  │  Taskcom = "kck", Task = 'initial' (loop)
  ▼
initial (lagi)
```

### 2.4 Fall Detection

```python
if present_pitch * 180/π > 60:
    body_status = 'fallen_forward' → action 122 (bangun depan)
if present_pitch * 180/π < -60:
    body_status = 'fallen_backward' → action 66 (bangun belakang)
```

### 2.5 Fungsi Penting di process.py

| Fungsi | Baris | Kegunaan |
|---|---|---|
| `Com_decision()` | 445 | Penentu role robot |
| `walk_follow_head(pan, tilt)` | 848 | Jalan mengikuti bola |
| `walk_ball_position_to_kick(target_pan, target_tilt, heading)` | 1199 | Posisi siap nendang |
| `walk_rotate2(heading)` | 932 | Rotasi ke arah tertentu |
| `walk_to_position(x, y, o)` | 864 | Navigasi odometry ke titik |
| `Heading_Gap(goal, yaw)` | 815 | Hitung selisih sudut terpendek |
| `check_ball_lost(threshold)` | 793 | Deteksi bola hilang |
| `calculate_pid(val, set_point)` | 723 | PID controller |
| `Motion_Kick(num)` | 1290 | Eksekusi gerakan nendang |

### 2.6 Fungsi Mapping

```python
map_value(val, smin, smax, tmin, tmax)   # dengan clamping (dibatasi min/max)
map_value2(val, smin, smax, tmin, tmax)   # tanpa clamping
```

Contoh: `map_value2(pan_degree, -89, 89, -15, 15)` → sudut pan -89° sd 89° dipetakan ke -15 sd 15 (a_move).

### 2.7 Timer System

```python
def Ticks(): return int(time.time() * 1000)  # millisecond timestamp
```
Digunakan untuk timing: `if Ticks() - start_timer >= 1000:` → setiap 1 detik.

---

## 3. driver.py — Tangan Kiri Robot (Hardware Interface)

**File:** `motion/src/driver.py` (587 baris)

### 3.1 Tugas Utama

1. **Menerima perintah** dari process.py via 6 topic subscriber
2. **Mengontrol module** robotis: walking, action, head
3. **PID head tracking** untuk bola dan gawang
4. **Publish joint kaki** ke odometry

### 3.2 Subscriber & Callback

| Topic | Callback | Fungsi |
|---|---|---|
| `/DEWO/MotionControl/MotionModule` | `MotionModule_Callback` | Ganti module (walk/action/head) |
| `/DEWO/MotionControl/Command` | `Command_Callback` | Start/stop/reset |
| `/DEWO/MotionControl/WalkingParams` | `WalkingParams_Callback` | Set parameter jalan |
| `/DEWO/MotionControl/ActionNum` | `ActionNum_Callback` | Eksekusi action page |
| `/DEWO/MotionControl/HeadPan` | `HeadPan_Callback` | Set posisi pan kepala |
| `/DEWO/MotionControl/HeadTilt` | `HeadTilt_Callback` | Set posisi tilt kepala |
| `/DEWO/MotionControl/HeadScan` | `HeadScan_Callback` | Aktifkan scan kepala |
| `/DEWO/image_processing/deteksi_bola/coordinate` | `point_of_ball` | PID tracking bola |
| `/DEWO/image_processing/deteksi_bola/goal_coordinate` | `point_of_goal` | PID tracking gawang |

### 3.3 PID Head Tracking

```python
p_gain = 0.265   # Proporsional (bola)
i_gain = 0.0     # Integral
d_gain = 0.02    # Derivatif

p_gain = 0.08    # Proporsional (gawang) — lebih lambat
```

Rumus: `x_err_target = x_err * p_gain + x_err_diff * d_gain + x_err_sum * i_gain`

Error dihitung dari: `x_err = -arctan(x * arctan(w))` di mana x = posisi bola relatif, w = FOV kamera.

Hasil PID dipublish ke `/DEWO/MotionControl/pan_tilt_error` yang dibaca process.py.

### 3.4 Walking Parameter (Aktif)

Ini adalah parameter yang *sedang dipakai* (baris 489-516):

```python
init_x_offset = -0.003     # Mundur sedikit
init_y_offset = 0.038      # Geser kanan
init_z_offset = 0.025      # Tinggi badan
hip_pitch_offset = 15°     # Condong ke depan
dsp_ratio = 0.29           # Rasio double support
step_fb_ratio = 0.28       # Rasio langkah
z_move_amplitude = 0.09    # Tinggi angkat kaki
balance_enable = True      # Balance aktif
```

Ada 7 versi parameter yang dikomentari (evolusi tuning).

### 3.5 Publisher Utama driver.py

| Publisher | Topic | Untuk |
|---|---|---|
| `pub_WalkingParams` | `/robotis/walking/set_params` | Set gait ke walking module |
| `pub_WalkingCommand` | `/robotis/walking/command` | Start/stop jalan |
| `pub_EnableCtrlModule` | `/robotis/enable_ctrl_module` | Ganti module kontrol |
| `pub_InitPose` | `/robotis/base/ini_pose` | Reset ke pose awal |
| `pub_PageNum` | `/robotis/action/page_num` | Eksekusi action |
| `pub_HeadJoint` | `robotis/head_control/set_joint_states` | Kontrol kepala |
| `pub_HeadScan` | `robotis/head_control/scan_command` | Scan kepala |
| `pub_JointKaki` | `/DEWO/Odometry/jointkaki` | Data kaki untuk odometry |
| `pantilterror` | `/DEWO/MotionControl/pan_tilt_error` | Error PID ke process |
| `pub_YawPos` | `/KRSBI/Manuvering/Sensor/Gyro/Yaw` | Gyro yaw integrator |

### 3.6 Main Loop driver.py

```
Loop tiap 100ms (10Hz):
  if tracking aktif & bola FOUND → set_head_joint_offset(x_err, y_err)
  if tracking aktif & bola NOTFOUND → head_start_scan() (setelah 20 frame)
  if positioning aktif & goal FOUND → set_head_joint_offset(x_err, y_err)
  if positioning aktif & goal NOTFOUND → head_start_scan() (setelah 5 frame)
  if walkingParams berubah → publish ke robotis
  if headJoint berubah → publish ke robotis
  if 100ms elapsed → publish gyroYawIntegrator
```

**Poin penting:** Driver tidak membuat keputusan sendiri. Dia eksekutor setia perintah dari process.py.

---

## 4. odometry_baru_2025.py — Indra Posisi (Dead-Reckoning)

**File:** `motion/src/odometry_baru_2025.py` (444 baris)

### 4.1 Cara Kerja

Odometry menghitung posisi robot (x, y, theta) dari **kinematika kaki**. Setiap langkah, sudut joint kaki dibaca → dihitung displacement → diakumulasi ke posisi global.

**Ini bukan GPS.** Error akan terakumulasi (drift) seiring waktu.

### 4.2 Kinematika Kaki (ForwardOdom)

Model kaki robot OP3 sebagai 3 segmen:

```
P = 12 cm (paha/thigh)
B = 12 cm (tulang kering/shin)  
K = 3.05 cm (telapak kaki/foot offset)

Rumus forward kinematics:
X = P*sin(PG) + B*sin(PG+LT) + K*sin(PG+LT-GL)
H = P*cos(PG) + B*cos(PG+LT) + K*cos(PG+LT-GL)

PG = pinggul (hip) angle
LT = lutut (knee) angle
GL = kaki (ankle) angle
```

### 4.3 Side Kinematics (SideOdom)

```python
base = PB * sin(-PG)
step = K * sin(-PG + GL)
Yr = base + step - y_offset/2    # kaki kanan
Yl = base + step + y_offset/2    # kaki kiri
```

PB = leg height (hasil dari ForwardOdom)

### 4.4 Angle Kinematics (AngleOdom)

```python
Angle = abs(Rpg) + abs(Lpg)   # Rpg = sudut yaw pinggul kanan, Lpg = kiri
```

### 4.5 Akumulasi Posisi Global

```python
W_global = deg2rad(W_global)              # heading dalam radian

# Forward movement
X_global += X * cos(W_global)
Y_global -= X * sin(W_global)

# Side movement  
X_global += Y * cos(pi/2 + W_global)
Y_global -= Y * sin(pi/2 + W_global)
```

### 4.6 Phase-Locked Computation

Odometry **tidak** menghitung tiap frame. Hanya pada fase tertentu:

```python
if fase in ["1", "2", "3"]:  # fase kaki tertentu (single support)
    trigger_odometri = True   # hitung displacement
```

Ini mencegah double-counting (satu langkah dihitung 2x).

### 4.7 Empat Mode Task Odometry

| Task | Fungsi | Komponen |
|---|---|---|
| `"move"` | Jalan maju/mundur | ForwardOdom + XMove |
| `"rotate"` | Berputar di tempat | SideOdom + YMove + AngleOdom |
| `"stay"` | Diam (hitung rotasi) | AngleOdom aja |
| `"positioning"` | Gabungan move + rotate | Semua komponen |

### 4.8 Goal Angle Calculation

```python
delta_x = goal_x - X_global
delta_y = goal_y - Y_global
w_goal = arctan2(-delta_y, delta_x)   # arah ke goal
```

Tanda negatif di `delta_y` karena koordinat Y di lapangan terbalik dengan Y di pygame.

### 4.9 Perbedaan odometry.py vs odometry_baru_2025.py

| Aspek | odometry.py | odometry_baru_2025.py |
|---|---|---|
| Walking params from | `/robotis/walking/set_params` | `/robotis/quintic_walk/set_params` |
| Phase topic | `/DEWO/phase` | `/DEWO/Walking/Phase` |
| YMove logic | `YMove(-L_side, R_side, -Angle)` | Tergantung `swim_cek` (arah side step) |
| Trigger system | `count = True` manual | `trigger_odometri` otomatis |
| Swim detection | Tidak ada | Ada: `swim_cek = "kiri"/"kanan"` |

---

## 5. lapangan.py — Mata Pelatih (Field Visualization)

**File:** `motion/src/lapangan.py` (136 baris)

### 5.1 Spesifikasi Visual

- **Library:** Pygame
- **Resolusi:** 1040 x 660 pixel
- **FPS:** 100
- **Judul:** "KRSBI-H: 'GANDAMANA' UNIVERSITAS NEGERI SURABAYA"

### 5.2 Dimensi Lapangan

| Parameter | Internal | Visual (P=1) |
|---|---|---|
| Panjang | 900 | 900 px |
| Lebar | 600 | 600 px |
| Goal Area | 100 x 500 | 100 x 500 px |
| Gawang | 260 x 60 | 260 x 60 px |
| Lingkaran tengah | radius 75 | 75 px |
| Titik penalty | radius 210 | 210 px |
| Offset kiri (nolX) | - | 69 px |
| Offset atas (nolY) | - | 30 px |

### 5.3 Warna & Area

```python
CYAN = Rect(0,0, 520, 660)      # Setengah kiri = CYAN
MAGENTA = Rect(520,0, 520, 660)  # Setengah kanan = MAGENTA
background = dark red
lapangan = green (0,200,0)
```

### 5.4 Yang Digambar

1. Background dark red → area CYAN (biru) + MAGENTA (merah)
2. Background gawang (hijau) + lapangan (hijau)
3. Garis tepi: atas (biru/merah untuk tim), bawah (hitam untuk juri)
4. Garis samping kiri-kanan (putih)
5. Garis tengah (putih)
6. Goal area kiri-kanan (putih)
7. Lingkaran tengah (putih)
8. Gawang kiri-kanan dengan kedalaman 60px (putih)
9. Titik tengah + titik penalty kiri-kanan
10. Posisi robot (dari gambar PNG, di-rotate sesuai yaw)

### 5.5 Konversi Koordinat

```python
Xglobal = (data.x / P) + nolX    # koordinat internal → pixel
Yglobal = (data.y / P) + nolY
# P = 1, jadi 1 unit internal = 1 pixel
# nolX = 69, nolY = 30 (offset ke tengah)
```

### 5.6 Cara Menjalankan

```bash
rosrun kri2027 lapangan.py
```

Atau langsung: `python motion/src/lapangan.py` (pastikan ROS master jalan).

---

## 6. Data Flow & Topic Map

### 6.1 Diagram Aliran Data Lengkap

```
Sensor (OpenCR):
  yaw ──────► /robotis/open_cr/yaw ──────────► process.py
  IMU ──────► /robotis/open_cr/imu ──────────► process.py (fall detection)
                                           ──► driver.py (gyro integrator)
  button ───► /robotis/open_cr/button ───────► process.py

Vision:
  Ball ─────► /DEWO/.../ball_state ─────────► process.py (status)
  Ball ─────► /DEWO/.../coordinate ─────────► process.py (posisi)
                                           ──► driver.py (PID tracking)
  Goal ─────► /DEWO/.../goal_state ────────► process.py (status)
  Goal ─────► /DEWO/.../goal_coordinate ────► process.py (posisi)
                                           ──► driver.py (PID tracking)

Game Controller:
  GC ───────► /DEWO/GameController/allstate ─► process.py

Decision (process.py → driver.py):
            MotionModule ─────► driver.py
            Command ──────────► driver.py
            WalkingParams ────► driver.py
            ActionNum ────────► driver.py
            HeadPan/Tilt ─────► driver.py
            HeadScan ─────────► driver.py

Odometry:
  joint kaki ──► /DEWO/Odometry/jointkaki ──► odometry_baru_2025.py
  goal_pos ────► /DEWO/Odometry/goal_position─► odometry_baru_2025.py
  position ────► /DEWO/Odometry/position ────► process.py
  goalangle ───► /DEWO/Odometry/goalangle ───► process.py

Display:
  position ───► /DEWO/Odometry/position ────► lapangan.py
  yaw ─────────► /DEWO/MotionControl/yaw ───► lapangan.py
```

### 6.2 Topic per File

| File | Subscribe | Publish |
|---|---|---|
| **process.py** | 8 topic (yaw, IMU, button, joint, ball, goal, robotinfo, gc) | 14 topic (motion, odometry, communication, vision control) |
| **driver.py** | 10 topic (motion module, command, walking params, action, head, ball, goal) | 8 topic (walking, joint, head, status, yaw) |
| **odometry_baru_2025.py** | 8 topic (joint, walking param, task, cmd, goal position, phase, yaw, initpoint) | 2 topic (position, goalangle) |
| **lapangan.py** | 2 topic (position, yaw) | 0 (read-only) |

---

## 7. Parameter Walking & Cara Tuning

### 7.1 Parameter Saat Ini (Aktif)

Semua di `driver.py` baris 489-516:

| Parameter | Nilai | Efek |
|---|---|---|
| `init_x_offset` | -0.003 | Posisi awal X (negatif = mundur) |
| `init_y_offset` | 0.038 | Posisi awal Y (positif = ke kanan) |
| `init_z_offset` | 0.025 | Posisi awal Z (tinggi badan) |
| `hip_pitch_offset` | 15° | Condong badan ke depan (stabilitas) |
| `dsp_ratio` | 0.29 | Waktu 2 kaki di tanah (makin besar = stabil) |
| `step_fb_ratio` | 0.28 | Rasio maju/mundur langkah |
| `z_move_amplitude` | 0.09 | Tinggi angkat kaki |
| `balance_enable` | True | Balance aktif/tidak |
| `balance_hip_roll_gain` | 0.35 | Gain balance hip roll |
| `balance_knee_gain` | 0.30 | Gain balance knee |
| `balance_ankle_roll_gain` | 0.70 | Gain balance ankle roll |
| `balance_ankle_pitch_gain` | 1.0 | Gain balance ankle pitch |
| `period_time` | 0.63 (default) | Kecepatan jalan (lebih kecil = lebih cepat) |

### 7.2 Parameter Dinamis (dari process.py)

Saat jalan, process.py mengirim:
```python
walking(x_move, y_move, angle_move, period_time)
```

- `x_move`: Kecepatan maju/mundur (±0.020 maks)
- `y_move`: Kecepatan samping (±0.015 maks)
- `angle_move`: Kecepatan rotasi (±15 maks)
- `period_time`: 0.63 (normal), 0.45 (setelah kick)

### 7.3 Cara Tuning Walking

**Langkah-langkah tuning:**

1. **Init offset dulu:**
   - `init_z_offset`: Naikkan kalau robot nyeret kaki, turunkan kalau robot oleng
   - `hip_pitch_offset`: Naikkan kalau robot jatuh ke belakang, turunkan kalau jatuh ke depan
   - `init_x_offset/y_offset`: Atur biar posisi kaki simetris

2. **Balance:**
   - `balance_enable=True` dulu
   - `balance_ankle_pitch_gain`: Naikkan kalau robot oleng depan-belakang
   - `balance_hip_roll_gain`: Naikkan kalau robot oleng kiri-kanan

3. **Kecepatan & Ketinggian:**
   - `z_move_amplitude`: Naikkan kalau kaki nyangkut di lantai
   - `period_time`: Kecilin biar cepet, besarin biar stabil
   - `dsp_ratio`: Besarin biar stabil (tapi jalan makin lambat)

### 7.4 Action Numbers

| Action | Number | Fungsi |
|---|---|---|
| Kick Kanan | 230 | Tendang pake kaki kanan |
| Kick Kiri | 225 | Tendang pake kaki kiri |
| Bangun Depan | 122 | Get up dari jatuh ke depan |
| Bangun Belakang | 66 | Get up dari jatuh ke belakang |

Pemilihan kaki tendang:
```python
if ball_x_angle < 0:   # bola di sebelah kiri robot
    kick right (230)    # tendang kaki kanan
else:                   # bola di sebelah kanan robot
    kick left (225)     # tendang kaki kiri
```

---

## 8. Latihan & Eksperimen

### 8.1 Latihan 1: Pahami State Machine

1. Buka `process.py` baris 1322-1785 (main loop)
2. Cari bagian `if Status == 'RUN':` dan `elif robotstatus == "UTAMA":`
3. Trace perjalanan Task: `initial → follow_ball → heading_to_goal_first → positioning_kick → kick_ball`
4. Tandai dengan stabile notes URL / nomor baris setiap transisi

### 8.2 Latihan 2: Tracing Data Flow

Jawab pertanyaan ini dengan membaca kode:
1. Dari mana `goal_orientation` berasal? (Hint: cari `goalangle_Callback`)
2. Bagaimana `walk_follow_head` mengubah posisi bola jadi kecepatan jalan?
3. Apa bedanya `map_value` dan `map_value2`? (Cek baris 758-764)

### 8.3 Latihan 3: Simulasi Mental

Bayangkan robot sebagai CYAN di posisi (400, 300, 0):
- Ball terdeteksi di (350, 250) dengan ukuran 500px
- Yaw robot = 10°, Goal orientation = 30° (goal CYAN di x=900)
- `robotstatus = "UTAMA"`, `Task = "follow_ball"`

Tentukan:
1. Berapa `pan_degree` dan `tilt_degree`?
2. Berapa nilai `a_move` dan `x_move` dari `walk_follow_head`?
3. Kapan Task berubah ke `heading_to_goal_first`?

### 8.4 Latihan 4: Tuning Parameter

Di `driver.py`, ada 7 versi parameter walking yang dikomentari (baris 373-487). Bandingkan:
- Versi 1 (baris 373-400): `init_x_offset = -0.034`
- Versi 7 (baris 489-516): `init_x_offset = -0.003`

Apa yang berubah? Cari pola: apakah tim cenderung menaikkan atau menurunkan `hip_pitch_offset`?

### 8.5 Eksperimen: Ubah Kecepatan Jalan

Di `process.py`, cari semua pemanggilan `walking(x, y, o, t)`:
1. Ganti `period_time` dari 0.63 ke 0.50 — apa efeknya?
2. Ganti `a_move_max` dari 15.0 ke 20.0 — apa efeknya?
3. Ganti `x_move_max` dari 20.0 ke 30.0 — apa efeknya?

### 8.6 Cara Menjalankan untuk Testing

**Minimal (tanpa robot):**
```bash
# 1. Jalankan roscore
roscore

# 2. Jalankan lapangan (visual)
rosrun kri2027 lapangan.py

# 3. Jalankan odometry
rosrun kri2027 odometry_baru_2025.py

# 4. Publish posisi manual untuk test
rostopic pub /DEWO/Odometry/position geometry_msgs/Pose2D "x: 450.0 y: 300.0 theta: 90.0"
```

**Full (dengan robot):**
```bash
roslaunch op3_manager op3_manager.launch
rosrun vision2027 vision_2027.py
rosrun communication2027 sendernasional.py
rosrun communication2027 com_receivernasional.py
rosrun kri2027 process.py
rosrun kri2027 driver.py
rosrun kri2027 lapangan.py
rosrun protocolgc receiver8.py
rosrun kri2026 odometry_baru_20.py
rostopic pub /robotis/open_cr/button std_msgs/String "data: 'start'"
```

### 8.7 Cheatsheet Debugging

| Masalah | Cek di |
|---|---|
| Robot tidak jalan | `driver.py` baris 75-80 (Command_Callback 'start') |
| Robot jalan tapi oleng | `driver.py` baris 489-516 (Walking params) |
| Robot tidak lihat bola | `process.py` baris 792-801 (check_ball_lost) |
| Posisi odometry ngaco | `odometry_baru_2025.py` baris 150-166 (position function) |
| Head tracking tidak stabil | `driver.py` baris 224-227 (PID gains) |
| Robot tidak tendang | `process.py` baris 1644-1654 (positioning_kick condition) |
| Robot jatuh terus | `process.py` baris 370-377 (fall detection threshold) |

---

> **Tips Belajar:** Baca kode dari `lapangan.py` dulu (paling pendek 136 baris), lalu `odometry_baru_2025.py` (444 baris), lalu `driver.py` (587 baris), terakhir `process.py` (1785 baris). Mapping setiap fungsi yang dipanggil — pahami dulu apa yang dilakukan, baru bagaimana cara kerjanya.
