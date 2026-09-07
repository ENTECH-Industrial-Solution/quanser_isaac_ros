# entech_quanser_ros2_ws

ROS 2 Jazzy workspace สำหรับขับหุ่นยนต์ Quanser อัตโนมัติใน NVIDIA Isaac Sim
ROS 2 Jazzy workspace for driving Quanser robots autonomously inside NVIDIA Isaac Sim.

**[ภาษาไทย](#ภาษาไทย) · [English](#english)**

```
Isaac Sim ──── /clock /scan /imu /odom + TF odom→base_link ────▶ ROS 2
   (sim)  ◀──────────────── /cmd_vel_twist ─────────────────  (SLAM / Nav2 / lane following)
```

Isaac Sim เป็นคนจำลองหุ่นยนต์และเป็นเจ้าของ `odom -> base_link` เสมอ
ฝั่ง ROS ทำ SLAM / นำทาง / ขับตามเลน แล้วส่งคำสั่งขับกลับเข้า drive graph ของ Isaac

---

## ภาษาไทย

### โครงสร้างไฟล์

```
entech_quanser_ros2_ws/
├── src/
│   ├── qcar2_isaac_nav2/           แพ็กเกจหลัก — ทั้ง 4 เส้นทางของ QCar2
│   └── qbot_platform_issac_nav2/   SLAM + Nav2 ของ QBot Platform
├── build/  install/  log/          ของที่ colcon build สร้าง (อยู่ใน .gitignore)
├── CLAUDE.md                       บันทึกสำหรับ Claude Code
└── README.md                       ไฟล์นี้
```

ทั้งสองแพ็กเกจเป็น `ament_cmake` และวางเหมือนกัน — `launch/ config/ src/ scripts/ rviz/ maps/`
รายละเอียดข้างในดูที่ README ของแพ็กเกจนั้น ๆ

### แพ็กเกจ

| แพ็กเกจ | หุ่นยนต์ | มีอะไร |
|---|---|---|
| [`qcar2_isaac_nav2`](src/qcar2_isaac_nav2/) | QCar2 (รถ Ackermann) | **ตัวหลัก** — 4 เส้นทางด้านล่าง |
| [`qbot_platform_issac_nav2`](src/qbot_platform_issac_nav2/) | QBot Platform (ต่างล้อ) | Cartographer + AMCL + Nav2 อย่างเดียว |

### สี่เส้นทางของ QCar2 (รันแยกกัน เลือกอย่างใดอย่างหนึ่ง)

| เส้นทาง | เซนเซอร์ | ทำอะไร | launch |
|---|---|---|---|
| ใช้ LiDAR ในการนำทางของหุ่นยนต์ | `/scan` | Cartographer เก็บแมพ → เรียกใช้ AMCL + Nav2 แล้วไปด้วย goal pose | `qcar2_mapping_launch.py` → `qcar2_navigation_launch.py` |
| นำทางด้วยกล้อง Realsense Depth Camera | RGB-D | RTAB-Map V-SLAM → Nav2 | `qcar2_vslam_mapping_launch.py` → `qcar2_vslam_navigation_launch.py` |
| ขับตามเลน + หลบสิ่งกีดขวาง | CSI + lidar + depth camera | ไม่ใช้ Nav2 ไม่มีแมพ ไม่มี goal — กล้องหาเลน แล้วหลบของที่ขวาง | `qcar2_lane_follow_launch.py` |
| ตรวจจับวัตถุ | CSI 4 ตัว 390° | YOLO → detections + ภาพ mosaic ไม่แตะการขับ | `qcar2_yolo_launch.py` |

ทั้งสี่ใช้ Nav2/bridge ครึ่งเดียวกัน ต่างกันแค่ครึ่งที่เป็นเซนเซอร์

### เริ่มเร็วสุดโดยการบังคับตัวหุ่นยนต์โดยตรง

โดยเริ่มขั้นตอน

1. **กด PLAY ใน isaac sim เพื่อเริ่มการทำงานของหุ่นยนต์**

2. **รันคำสั่งด้านล้างเพื่อบังคับหุ่นยนต์**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_twist
```

### Build

```bash
cd ~/entech_quanser_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qcar2_isaac_nav2
source install/setup.bash
```

`config/ launch/ rviz/ behavior_trees/ maps/` ติดตั้งผ่าน `install(DIRECTORY ...)` —
**แก้ YAML / launch / rviz / map แล้วต้อง build ใหม่ทุกครั้ง** ไม่งั้นจะเหมือนแก้แล้วไม่มีอะไรเกิดขึ้น

ไม่มี unit test — ความถูกต้องคือ "รถไปถึงเป้าใน Isaac Sim ได้จริงไหม" `colcon test` รันแค่ linter

### กฎเหล็ก 3 ข้อ

1. **กด PLAY ใน Isaac Sim ก่อน launch เสมอ และห้ามกด Stop/Play ระหว่างที่ ROS รันอยู่** —
   `/clock` จะรีเซ็ตเป็น 0, `map -> odom` หาย, ทุก goal fail ใน ~13 ms ต้อง restart ฝั่ง ROS ใหม่
2. **ห้ามเปิดสอง stack ซ้อนกัน** — `ros2 node list | sort | uniq -d` ต้องไม่มีอะไรออกมา
3. **`angular.z` ที่ส่งเข้า Isaac ของ QCar2 คือ *มุมเลี้ยวล้อหน้า* หน่วยเรเดียน ไม่ใช่ yaw rate** —
   [`twist_stamped_to_twist.py`](src/qcar2_isaac_nav2/src/twist_stamped_to_twist.py) แปลงหน่วยด้วย
   โมเดลจักรยาน `δ = atan(ω·L/v)` (L = 0.258 ม.) ไม่ใช่แค่เปลี่ยนชนิดข้อความ ถอดออกแล้วรถจะวิ่งวน
   เป็นวงกลมทั้งที่ log ของ Nav2 ดูปกติทุกบรรทัด (ของ QBot เป็นรถต่างล้อ ตัวแปลงเป็น shim เฉย ๆ)

### รายละเอียด

**[`src/qcar2_isaac_nav2/README.md`](src/qcar2_isaac_nav2/README.md)** — วิธีรันทีละขั้น, หน้าที่ของทุกไฟล์,
ค่าพารามิเตอร์แต่ละตัวมาจากการวัดอะไร, และกับดักที่เคยเสียเวลามาแล้ว มีสารบัญให้กดข้ามได้
อ่านก่อนแก้ค่าใน `config/` — ค่าส่วนใหญ่มาจากการวัดจริง ไม่ใช่ค่า default

---

## English

### Layout

```
entech_quanser_ros2_ws/
├── src/
│   ├── qcar2_isaac_nav2/           the main package — all four QCar2 routes
│   └── qbot_platform_issac_nav2/   SLAM + Nav2 for the QBot Platform
├── build/  install/  log/          colcon build output (gitignored)
├── CLAUDE.md                       notes for Claude Code
└── README.md                       this file
```

Both packages are `ament_cmake` and laid out the same way — `launch/ config/ src/ scripts/ rviz/
maps/`. For what is inside one, see that package's own README.

### Packages

| Package | Robot | What it holds |
|---|---|---|
| [`qcar2_isaac_nav2`](src/qcar2_isaac_nav2/) | QCar2 (Ackermann car) | **The main one** — the four routes below |
| [`qbot_platform_issac_nav2`](src/qbot_platform_issac_nav2/) | QBot Platform (differential drive) | Cartographer + AMCL + Nav2 only |

### The four QCar2 routes (run one at a time)

| Route | Sensors | What it does | Launch |
|---|---|---|---|
| Navigating the robot with the LiDAR | `/scan` | Cartographer builds the map → AMCL + Nav2 then drive to a goal pose | `qcar2_mapping_launch.py` → `qcar2_navigation_launch.py` |
| Navigating with the RealSense depth camera | RGB-D | RTAB-Map V-SLAM → Nav2 | `qcar2_vslam_mapping_launch.py` → `qcar2_vslam_navigation_launch.py` |
| Lane following + obstacle avoidance | CSI + lidar + depth camera | No Nav2, no map, no goal — the camera finds the lane and the car steers around whatever blocks it | `qcar2_lane_follow_launch.py` |
| Object detection | 4 CSI cameras, 390° | YOLO → detections + a mosaic image; touches nothing that drives | `qcar2_yolo_launch.py` |

All four share the same Nav2/bridge half; only the sensing half differs.

### Quickest start — drive the robot yourself

Steps:

1. **Press PLAY in Isaac Sim so the robot starts running**

2. **Run the command below to drive it**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_twist
```

### Build

```bash
cd ~/entech_quanser_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qcar2_isaac_nav2
source install/setup.bash
```

`config/ launch/ rviz/ behavior_trees/ maps/` are installed with `install(DIRECTORY ...)` —
**every edit to a YAML, launch, rviz or map file needs a rebuild**, otherwise the change looks like
it did nothing.

There are no unit tests. Correctness here means "does the car actually reach the goal in Isaac Sim";
`colcon test` only runs linters.

### Three hard rules

1. **Press PLAY in Isaac Sim before every launch, and never press Stop/Play while ROS is running** —
   `/clock` resets to 0, `map -> odom` disappears, and every goal fails in ~13 ms. Restart the ROS
   side afterwards.
2. **Never leave two stacks running** — `ros2 node list | sort | uniq -d` must print nothing.
3. **On QCar2, the `angular.z` sent into Isaac is a front-wheel *steering angle* in radians, not a
   yaw rate** — [`twist_stamped_to_twist.py`](src/qcar2_isaac_nav2/src/twist_stamped_to_twist.py)
   converts the unit with the bicycle model `δ = atan(ω·L/v)` (L = 0.258 m). It is not just a change
   of message type: remove it and the car drives in circles while every Nav2 log line still looks
   healthy. (QBot is differential drive, so its converter really is just a shim.)

### Details

**[`src/qcar2_isaac_nav2/README.md`](src/qcar2_isaac_nav2/README.md)** (Thai) — step-by-step run
instructions, what every file does, what measurement each tuned parameter came from, and the traps
that have already cost time; it has a table of contents to jump around with. Read it before changing
anything in `config/` — most values there are measured, not defaults.