# qcar2_isaac_nav2

<a id="overview"></a>
## 1. ภาพรวม

ROS 2 Jazzy package (`ament_cmake`) สำหรับขับ **Quanser QCar2** (รถ Ackermann) ใน
**NVIDIA Isaac Sim** ด้วย **Nav2 1.3.12** — รวม 5 ความสามารถที่แยกกันเป็นอิสระ:

| # | เรื่อง | ใช้เซนเซอร์ | launch |
|---|---|---|---|
| 1 | [SLAM (lidar)](#slam) | lidar + odom | `qcar2_mapping_launch.py` → `qcar2_navigation_launch.py` |
| 2 | [V-SLAM (กล้อง)](#vslam) | RGB-D | `qcar2_vslam_mapping_launch.py` → `qcar2_vslam_navigation_launch.py` |
| 3 | [CSI 360° + YOLO](#csi) | CSI 4 ตัว | `qcar2_yolo_launch.py` |
| 4 | [ขับตามเลน](#lane) | CSI หน้า | `qcar2_lane_follow_launch.py` |
| 5 | [หลบสิ่งกีดขวาง](#obstacle) | lidar + depth | (มากับข้อ 4) / depth เข้า costmap ของข้อ 1–2 |

ข้อ 1–2 ใช้ Nav2 (มีแมพ มี goal pose) ครึ่ง Nav2 ใช้ร่วมกันทั้งคู่ ต่างกันแค่ครึ่งเซนเซอร์
ข้อ 3–4 **ไม่ใช้ Nav2 เลย** และข้อ 4 ห้ามรันพร้อม Nav2

### เส้นทางของคำสั่งขับ

```
Isaac Sim ──/clock /scan /imu /odom, TF odom->base_link──> Nav2
Nav2 controller_server ──/cmd_vel_nav (TwistStamped, yaw rate rad/s)──┐
                                                                      ▼
                                              src/twist_stamped_to_twist.py
                                                                      │
Isaac Sim QCar2 drive graph <──/cmd_vel_twist (Twist, มุมเลี้ยว rad)──┘
```

`twist_stamped_to_twist.py` ไม่ใช่แค่แปลงชนิดข้อความ แต่ **แปลงหน่วย**: Nav2 ส่ง yaw rate
ส่วน Isaac อ่านเป็นมุมเลี้ยวล้อหน้า จึงใช้ `δ = atan(ω·L/v)`, `L = 0.258 m`
ถอดตัวนี้ออกเมื่อไร รถจะเลี้ยวแรงเกินคำสั่งหลายเท่าแล้ว **วิ่งวน** ทั้งที่ log ทุกบรรทัดดูปกติ

Isaac Sim เป็นเจ้าของ `odom -> base_link` เสมอ ต่างกันแค่ใคร publish `map -> odom`
(Cartographer / AMCL / RTAB-Map)

### Build

```bash
cd ~/entech_quanser_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qcar2_isaac_nav2
source install/setup.bash
```

`config/ launch/ rviz/ behavior_trees/ maps/` ถูก install ผ่าน CMake —
**แก้ YAML/lua/launch/rviz/map แล้วต้อง `colcon build` ใหม่เสมอ** ไม่งั้นไม่มีผล (กับดักคลาสสิกของ workspace นี้)

### กฎเหล็ก

- กด **PLAY** ใน Isaac Sim ก่อน launch เสมอ และ **ห้ามกด Stop/Play ระหว่างที่ ROS รันอยู่** —
  `/clock` รีเซ็ตเป็น 0, Cartographer ตาย, `map -> odom` หาย, goal fail ใน ~13 ms
- **ห้ามเปิดสอง stack ซ้อนกัน** — `ros2 node list | sort | uniq -d` ต้องว่าง
- `enable_stamped_cmd_vel: true` ต้องมีทุกโหนดของ Nav2 ที่แตะ cmd_vel (Jazzy default เป็น `false`)
  ไม่งั้น Nav2 วางแผนสวยแต่รถไม่ขยับ — เช็ค `ros2 topic info -v /cmd_vel_nav` ต้องมีชนิดเดียว
- ต้อง override behavior tree **ทั้งสองไฟล์** (`navigate_to_pose` + `navigate_through_poses`)
  เพราะรถ Ackermann หมุนอยู่กับที่ไม่ได้ จึงตัด `Spin` ออก
- **ห้ามใส่ list ว่างใน YAML** (`polygons: []` ฯลฯ) — โหนดจะ abort ตอนสตาร์ต ใช้ค่าจริง + `enabled: False` แทน

---

<a id="tree"></a>
## 2. โครงสร้างไฟล์

```
src/qcar2_isaac_nav2/
├── launch/
│   ├── qcar2_mapping_launch.py               [1] เก็บแมพด้วย Cartographer
│   ├── qcar2_navigation_launch.py            [1] map_server + AMCL + Nav2
│   ├── qcar2_vslam_mapping_launch.py         [2] เก็บแมพด้วย RTAB-Map RGB-D
│   ├── qcar2_vslam_navigation_launch.py      [2] RTAB-Map localization + Nav2
│   ├── qcar2_yolo_launch.py                  [3] YOLO บนกล้อง CSI
│   ├── qcar2_lane_follow_launch.py           [4][5] ขับตามเลน + หลบ (จูนสี/ขับ)
│   ├── depth_scan_launch.py                  [5] depth -> /scan_depth (ใช้ร่วม 3 เส้นทาง)
│   ├── qcar2_cartographer_launch.py          (legacy) cartographer อย่างเดียว
│   └── qcar2_slam_and_nav_bringup_launch.py  (legacy) SLAM+Nav2 พร้อมกัน ไม่ใช้แมพเซฟ
├── config/
│   ├── qcar2_mapping.lua                     [1] จูน Cartographer (กิน /odom ด้วย)
│   ├── qcar2_nav2_amcl.yaml                  [1] Nav2 + AMCL ครบชุด
│   ├── qcar2_nav2_vslam.yaml                 [2] ก๊อปจากไฟล์บน ตัด /scan ใช้ VoxelLayer
│   ├── lane_avoid.yaml                       [4][5] profile ค่าขับ + ค่าหลบ
│   ├── lane_colors.yaml                      [4] ค่า HSV ตั้งต้น
│   └── qcar2_2d.lua / qcar2_slam_and_nav.yaml  (legacy)
├── behavior_trees/
│   ├── navigate_to_pose_ackermann.xml        ตัด Spin ใช้ BackUp แทน
│   └── navigate_through_poses_ackermann.xml  ต้องแก้คู่กัน ไม่งั้น bt_navigator activate ไม่ผ่าน
├── src/
│   ├── twist_stamped_to_twist.py             bridge Nav2 -> Isaac (แปลงหน่วยมุมเลี้ยว)
│   ├── yolo_detector.py                      [3] YOLO 4 กล้อง + mosaic (ไม่ใช้ cv_bridge)
│   ├── lane_follower.py                      [4] หาเลน + ขับ + โหมดจูนสี (ไฟล์เดียว 2 โหมด)
│   └── obstacle_avoider.py                   [5] ตัดสินใจ -> /lane/avoid (ไม่แตะ cmd_vel)
├── scripts/
│   ├── isaac_add_rgbd_camera.py              [2] กล้อง RGB-D registered ตัวเดียว
│   ├── isaac_add_depth_camera.py             [5] กล้อง depth + graph (asset มีแต่ Xform เปล่า)
│   ├── isaac_add_csi_cameras.py              [3] เปิด active ของ CSI 3 ตัว + normalize ทั้ง 4
│   ├── isaac_camera_streams.py               preset เลือกว่า render กล้องตัวไหน (= งบ GPU)
│   ├── isaac_apply_and_save.py               รันสคริปต์ข้างบนแบบ headless แล้วเซฟ stage
│   ├── isaac_sim_control.py                  play/stop/reset/วางรถ ผ่าน /tmp/qcar2_sim_cmd.json
│   ├── save_map.sh                           [1] เซฟ .pgm/.yaml/.pbstream
│   └── save_vslam_map.sh                     [2] เซฟ .pgm/.yaml จาก /map ของ RTAB-Map
├── rviz/
│   ├── qcar2_mapping.rviz / qcar2_nav2.rviz              [1]
│   ├── qcar2_vslam.rviz / qcar2_vslam_nav.rviz           [2]
│   ├── qcar2_yolo.rviz                                   [3] mosaic + detections
│   ├── qcar2_lane_avoid.rviz                             [4][5] debug image + scan
│   ├── qcar2_depth_view.rviz                             ดู depth ดิบ
│   └── qcar2.rviz                                        รวมทุก display เปิดเอง
├── maps/
│   ├── qcar2_map.yaml/.pgm/.pbstream          [1] แมพ lidar + เซสชัน Cartographer
│   ├── qcar2_vslam_map.yaml/.pgm              [2] ภาพฉาย 2 มิติของแมพกล้อง
│   └── qcar2_vslam_cloud_cloud.ply            [2] cloud 3 มิติที่ export ไว้
├── CMakeLists.txt / package.xml               build + dependencies
├── setup.py / setup.cfg / rt_models/          (legacy) ไม่ได้ใช้
└── README.md
```

ไฟล์ที่ **ไม่อยู่ใน repo แต่ขาดไม่ได้**:
`~/.ros/qcar2_vslam.db` (pose graph ของ V-SLAM) และ `~/.ros/qcar2_lane_colors.yaml` (สีเลนที่จูนแล้ว)

---

<a id="toc"></a>
## 3. สารบัญ

- [1. ภาพรวม](#overview) · [เส้นทางของคำสั่งขับ](#overview) · [Build](#overview) · [กฎเหล็ก](#overview)
- [2. โครงสร้างไฟล์](#tree)
- [3. สารบัญ](#toc)
- [4. เจาะลึกแต่ละเรื่อง](#deep)
  - [4.1 SLAM — เก็บแมพและนำทางด้วย lidar](#slam)
  - [4.2 V-SLAM — เก็บแมพและนำทางด้วยกล้อง RGB-D](#vslam)
  - [4.3 CSI 360° + YOLO](#csi)
  - [4.4 Lane following — ขับตามเลนด้วยกล้อง](#lane)
  - [4.5 Obstacle detection — หลบสิ่งกีดขวาง](#obstacle)
- [5. แก้ปัญหา](#trouble)

---

<a id="deep"></a>
## 4. เจาะลึกแต่ละเรื่อง

<a id="slam"></a>
### 4.1 SLAM — เก็บแมพและนำทางด้วย lidar

2 เฟส: Cartographer สร้างแมพ → เซฟ → `map_server` + AMCL นำทางบนแมพนั้น

```bash
# เฟส 1 — เก็บแมพ (terminal 1)
ros2 launch qcar2_isaac_nav2 qcar2_mapping_launch.py
# (terminal 2) ขับช้า ๆ — /scan ของ Isaac ออกแค่ ~4 Hz
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.6 -p turn:=0.5 -r /cmd_vel:=/cmd_vel_twist
# (terminal 3) เซฟแล้ว build ให้ install เห็น
ros2 run qcar2_isaac_nav2 save_map.sh qcar2_map && colcon build --packages-select qcar2_isaac_nav2

# เฟส 2 — นำทาง
ros2 launch qcar2_isaac_nav2 qcar2_navigation_launch.py    # map:=/abs/path.yaml ได้
```

รอ `Managed nodes are active` **ครบ 2 ตัว** แล้วใน RViz กด 2D Pose Estimate (ถ้ารถไม่ได้อยู่จุดเดิม)
ตามด้วย 2D Goal Pose · ต้องขับเก็บแมพให้ครบทุกทาง เพราะ `allow_unknown: false`

**ค่าที่วัดมา ไม่ได้เดา** — จาก TF: wheelbase 0.258 m, ตัวถัง ~0.39 × 0.19 m
→ `footprint [[0.21,0.10],[0.21,-0.10],[-0.19,-0.10],[-0.19,0.10]]`, `minimum_turning_radius 0.5`
(ต้องตรงกับ `wheelbase` ใน bridge เสมอ)

| ค่า | ทำไม |
|---|---|
| `motion_model_for_search: REEDS_SHEPP` | Dubins เดินหน้าอย่างเดียว เป้าหมายข้างหลังจะวนหลายเมตร · คู่กับ `PreferForwardCritic 1.5` (5.0 จะบล็อกช่วงถอยจนรถนิ่ง) |
| `yaw_goal_tolerance: 3.15` | ไม่สนทิศตอนจอด รถหมุนอยู่กับที่ไม่ได้ ถ้าบังคับมุมจะวนรอบเป้าไม่จบ |
| `CostCritic consider_footprint: true` | ถ้า false เช็คชนเป็นวงกลม r=0.10 ทั้งที่รถยาว 0.40 → เฉี่ยวกำแพงแล้วค้าง (ลด `batch_size` เป็น 1000 ชดเชย) |
| `inflation_radius: 0.30` | 0.75 ทำให้ประตูทุกบานดูวิ่งผ่านไม่ได้ |
| `local_costmap` frame `odom` ไม่มี static layer | ถ้าใช้ `map` costmap จะกระตุกทุกครั้งที่ AMCL แก้ตำแหน่ง |

**path ของแมพถูกยัดเข้า params ใน `OpaqueFunction`** ไม่ได้ส่งผ่าน `map:=` ของ nav2
เพราะ argument นั้นมาเป็น params file scope `/**:` และ ROS 2 ให้ key `map_server:` ชนะ wildcard เสมอ
→ `yaml_filename` ว่างชนะ แล้ว map_server ขึ้นมาแบบไม่มีแมพ

ผลวัดจริง goal เดียวกันก่อน/หลังจูน: 9.07 m / 297° / timeout → **1.84 m / 65° / SUCCEEDED 13 s**
(ระยะเส้นตรง 1.92 m) — เกณฑ์สุขภาพคือ ระยะที่วิ่ง ÷ ระยะเส้นตรง ~0.9–1.3 และมุมรวม < ~80°

---

<a id="vslam"></a>
### 4.2 V-SLAM — เก็บแมพและนำทางด้วยกล้อง RGB-D

RTAB-Map ทำ RGB-D SLAM แทน lidar: ดึง feature จากภาพสี ปิด loop จากหน้าตาสถานที่ สะสม depth
เป็น voxel map 3 มิติ แล้วฉายลงเป็น occupancy grid ปกติให้ costmap กินต่อ
ครึ่ง Nav2 เหมือนข้อ 4.1 ทุกอย่าง — `qcar2_nav2_vslam.yaml` คือไฟล์เดิมที่ตัด `/scan` ทิ้ง
และเปลี่ยน obstacle layer เป็น VoxelLayer ที่กิน point cloud **จูน planner/controller เมื่อไรต้องแก้ให้ตรงกันทั้งสองไฟล์**

**ขั้นที่ 0 (ครั้งเดียว ตอนหยุด sim)** — paste `scripts/isaac_add_rgbd_camera.py` ลง Script Editor
สร้าง Camera ใต้ `realsenseRGB` ให้ **render product ตัวเดียว** ป้อนทั้งสี/depth/camera_info
→ stamp ตรงกัน จึงใช้ `approx_sync: false` ได้
(`/realsense_depth` เดิมของ asset ห่างจากเลนส์สี 37 มม. = เพี้ยน ~18 px ที่ 1 ม. **ใช้กับ RGB-D SLAM ไม่ได้**)

```bash
# เฟส 1 — เก็บแมพ
ros2 launch qcar2_isaac_nav2 qcar2_vslam_mapping_launch.py     # ขับช้า ๆ ให้ครบทุกทาง
ros2 run qcar2_isaac_nav2 save_vslam_map.sh qcar2_vslam_map    # เซฟตอน launch ยังรันอยู่
colcon build --packages-select qcar2_isaac_nav2                # แล้วค่อย Ctrl-C ปิด db ให้เรียบร้อย

# เฟส 2 — นำทาง
ros2 launch qcar2_isaac_nav2 qcar2_vslam_navigation_launch.py
```

ได้ **2 ชิ้น ต้องเก็บทั้งคู่**: `~/.ros/qcar2_vslam.db` (pose graph + visual words สำหรับ relocalise)
และ `maps/qcar2_vslam_map.yaml/.pgm` (static layer ของ costmap)
ที่ต้องมี `.pgm` เพราะให้ RTAB-Map เสิร์ฟ `/map` เอง = อุ้ม grid ของทุก node ไว้ใน working memory
→ 6 GB จน OOM killer ลาก Isaac Sim ไปด้วย

| argument | default | ความหมาย |
|---|---|---|
| `start_at_origin` | `true` | เชื่อว่ารถเริ่มที่จุดกำเนิดแมพ ไม่ต้อง relocalise (จริงใน Isaac เพราะ respawn ที่เดิม, ~0.7 GB) · `false` = ให้หาตัวเองจากกล้องจริง ~2.7 GB, ~0.6 s/เฟรม |
| `database_path` | `~/.ros/qcar2_vslam.db` | ฐานข้อมูลจากเฟส 1 |
| `use_depth_scan` | `true` | ป้อน `/scan_depth` เข้า local costmap เพิ่ม |

**กับดักที่เงียบสนิท**

- `RGBD/MaxOdomCacheSize: 1` — default 10 รอ localization ที่ตรงกันหลายครั้ง แต่รถ**จอดนิ่ง**
  ไม่ผลิต pose ใหม่ cache ไม่มีวันเต็ม → `map -> odom` ไม่เคยออก ทุก goal fail ว่า `map does not exist`
  โดยไม่มีบรรทัดไหนเป็น error
- `Mem/InitWMWithAllNodes` ต้องผูกกับ `start_at_origin` (working memory ว่าง + ไม่บอกตำแหน่ง = ไม่ publish ตลอดกาล)
- เปิดแมพเดิมแล้วเห็นนิดเดียว = ปกติ ยังไม่ relocalise ไม่ใช่แมพหาย
- **อย่าเรียก `/rtabmap/backup`** มันคือ save + copy + **re-init working memory** แล้วแมพจะยุบเหลือเท่าที่เห็น
- `Not enough inliers 0/15` → ไปดู**ภาพสี**ก่อน มักเป็น exposure พังจนขาวโพลน ไม่ใช่พารามิเตอร์ SLAM
- depth ของ Isaac เป็น 32FC1 ต้องตั้ง `Mem/DepthCompressionFormat=.png`
- odometry ยังมาจาก Isaac ไม่ใช่ `rgbd_odometry` เพราะตัวหลังจะ publish `odom -> base_link` ตัวที่สองมาแย่งกัน

---

<a id="csi"></a>
### 4.3 CSI 360° + YOLO

กล้อง CSI 4 ตัวรอบคัน ตัวละ 97.6° รวม 390° รัน YOLO ตัวเดียวร่วมกัน
**เป็น sensing ล้วน ๆ ไม่แตะ TF / costmap / cmd_vel** จึงเปิดปิดทับ stack ที่รันอยู่ได้

```
csi_front/back/left/right ──> yolo_detector ──> /csi_<pos>/detections  (Detection2DArray)
                                             ├─> /csi_<pos>/annotated
                                             └─> /csi/mosaic  (2x2 — RViz ไม่มี grid layout)
```

```bash
pip install --user --break-system-packages ultralytics   # ครั้งเดียว (weights ลง ~/.cache/qcar2_yolo/)
# ครั้งเดียว ตอนหยุด sim: paste scripts/isaac_add_csi_cameras.py ลง Script Editor
QCAR2_CAMERA_PRESET=csi python3 scripts/isaac_camera_streams.py
ros2 launch qcar2_isaac_nav2 qcar2_yolo_launch.py         # cameras:= model:= confidence:= imgsz:=
```

**asset มี graph ครบทั้ง 4 ตัวอยู่แล้ว ต่อสายถูกหมด** แค่ 3 ตัวถูกปิดด้วย `active = False`
prim ที่ปิดจะ "หายไป" ทั้งดุ้น (ลูกไม่ถูก compose, traverse ไม่เจอ) มันดูเหมือน**ไม่มีอยู่จริง**
ถ้าเชื่อแล้วไปสร้าง graph ใหม่ = สร้างซ้อนบนของที่มีอยู่ใน `qcar2.usd`
สคริปต์จึงแค่เปิดสวิตช์ + ตั้ง `topicName`, `frameSkipCount=5` (~10 Hz), `enabled` ของ render product
เป็น USD attribute ล้วน ๆ และ `active` เขียนเป็น override บน root layer — `qcar2.usd` ไม่เคยถูกแตะ

**render product คืองบ GPU และ `frameSkipCount` ไม่ช่วย** — RTX render ทุกเฟรมไม่ว่ามีคน subscribe หรือไม่
ทางเดียวที่ได้ GPU คืนคือปิด render product `isaac_camera_streams.py` จึงเป็น preset:

| preset | เปิด | ใช้ตอน |
|---|---|---|
| `vslam` | RGB-D (1) | เส้นทาง V-SLAM |
| `csi` | CSI 4 ตัว (4) | YOLO 360° |
| `csi_front` | CSI หน้า (1) | YOLO คู่กับ V-SLAM (`cameras:=front`) |
| `lane_avoid` | CSI หน้า + depth (2) | ขับตามเลน + หลบ |
| `both` / `none` | 5 / 0 | ครบทุกอย่าง / lidar อย่างเดียว |

4 กล้องพร้อมกันเคยทำให้คู่ RGB-D เหลือ **1.6 Hz** — CSI ครบวงกับ V-SLAM ไม่ได้ตั้งใจให้รันพร้อมกัน
วัดได้ 4 กล้อง × ~9.5 Hz, drop 0% บน yolo11n fp16 (แต่วัดตอน sim หยุด = เพดานบน)

**กับดัก**

- **ห้ามใช้ `cv_bridge`** — Jazzy build มากับ NumPy 1.x แต่ `~/.bashrc` ดัน NumPy 2.5.2 ของ Isaac ขึ้นหน้า
  แปลงภาพทีเดียว **segfault (139)** ไม่มี traceback เหมือน driver พังมากกว่า dependency ชน
  `yolo_detector.py` จึงแปลงเองด้วย NumPy (`decode_rgb`/`encode_rgb`) — **อย่าแก้กลับ**
- ภาพขาวโพลน = Isaac เขียน `omni:rtx:autoExposure:enabled = False` ลงกล้องที่มี render product
  (สคริปต์บังคับเปิดคืนให้) เจอ 0 object ให้ **ดูภาพก่อน** อย่าเพิ่งลด `confidence`
- `ros2 run` ทิ้ง python orphan — `pgrep -af yolo_detector` ต้องมีไม่เกิน 1 ก่อนเชื่อผลลัพธ์
- ultralytics 8.4 เปลี่ยน `half` เป็น `quantize` (node แปลงให้แล้ว) ไม่งั้น warn ทุก predict จนกลบ log
- ไม่มี camera_info: YOLO เป็น 2D และ RViz *Image* display ไม่อ่านมัน

---

<a id="lane"></a>
### 4.4 Lane following — ขับตามเลนด้วยกล้อง

**ไม่ใช้ Nav2 เลย** ไม่มีแมพ ไม่มี costmap ไม่มี goal — กล้องหน้าเห็นเส้นเลน คำนวณว่าเบี่ยงจากกลางเลนเท่าไร
แล้วส่งมุมเลี้ยวเข้า `/cmd_vel_twist` ตรง ๆ · **ห้ามรันพร้อม Nav2** (สอง publisher แย่งกันสั่งเลี้ยว)

```
/csi_front/image_raw ──> lane_follower.py ──> /cmd_vel_twist ──> Isaac
                              ▲    └────────> /lane/debug_image
                   /lane/avoid │ (follow | avoid | stop) จาก obstacle_avoider
```

```bash
QCAR2_CAMERA_PRESET=lane_avoid python3 scripts/isaac_camera_streams.py   # ครั้งเดียว
ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py tune:=true      # จูนสี: s=เซฟ q=ออก
ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py                 # ขับ
```

**กฎเดียวใช้ได้ทั้งถนน 1 เลน (ขาว 2 ข้าง) และ 2 เลน (น้ำเงินกลาง)** เพราะกฎมองว่า "เส้นที่ใกล้ที่สุด
ซ้าย/ขวาคืออะไร" ไม่ได้มองว่าถนนแบบไหน — เห็นครบสองข้าง = เล็งกึ่งกลาง, เห็นข้างเดียว = ห่างครึ่งเลน
โดยครึ่งความกว้างเลน**วัดแล้วจำ (EMA)** ตอนที่เห็นครบ ไม่ใช่ค่า default
รถขับชิดขวา เส้นน้ำเงินจึงต้องอยู่ซ้ายเสมอ ถ้าเจอน้ำเงินทางขวา = หลุดไปเลนสวน โค้ดจะเล็งข้ามกลับ

**ที่เก็บค่า 2 ที่ แยกหน้าที่กันชัด**

| ไฟล์ | เก็บอะไร | เขียนโดย |
|---|---|---|
| `~/.ros/qcar2_lane_colors.yaml` | HSV, `roi_top`, `band_frac` | `tune:=true` (กด `s` = **เขียนทับทั้งไฟล์** อย่างอื่นที่ใส่ไว้หายเงียบ) |
| `config/lane_avoid.yaml` (profile) | ค่าขับ + ค่าหลบ **ทุกตัว** | มือ (ก๊อปไป `~/.ros/qcar2_lane_avoid.yaml` แล้วแก้ได้โดยไม่ต้อง build) |

ลำดับความสำคัญ **command line > profile > default ในโค้ด** — `declare_parameter('trigger_distance', 2.00)`
ในซอร์สจึงไม่ใช่ค่าที่รถใช้ ยกเว้นสั่ง `profile:=none` (พิสูจน์ด้วย `ros2 param get`)

| argument | หมายเหตุ |
|---|---|
| `speed`, `kp`, `kd` | เกนพื้นฐาน · กล้องออก ~10 Hz เร็วเกินนี้คือเลี้ยวจากภาพเก่า (เข้าโค้งลดให้เอง) |
| `k_head` | เกนต่อ**ทิศทางเลน** จากสองแถบ look-ahead — กันเปลี่ยนเลนแล้วเลยเป้า |
| `max_error_rate` | จำกัดความเร็วที่จุดเล็งเลื่อนข้าง กันหักกระชากตอนสลับโหมด |
| `max_steering_angle` | ต้องเท่ากับใน bridge และลิมิตของ Isaac (0.50) |
| `blue_is_centre_line` | `true` = ข้ามเส้นน้ำเงินไปเลนซ้ายแล้วกลับ · `false` = เบี่ยงชิดซ้ายในคอริดอร์เดิม (`avoid_offset_frac`) |
| `max_run_frac` | เส้นจริงกว้าง ~8% ของภาพ — ของขาว/กำแพง/พื้นสว่างเป็น run หลายร้อย px ถ้าไม่ตัดจะเล็งเข้าไปชน |
| `tune`, `colors`, `avoid`, `profile` | สลับโหมด / เลือกไฟล์ / เปิดปิดตัวหลบ |

ดูว่ามันเห็นอะไร: `ros2 run rqt_image_view rqt_image_view /lane/debug_image`
(หน้าต่างจูนคือ**ภาพเดียวกับที่ตัวขับเห็น** — คำถามคือ "จุดเล็งไปกลางเลนไหม" ไม่ใช่ "mask ติดเส้นไหม")

---

<a id="obstacle"></a>
### 4.5 Obstacle detection — หลบสิ่งกีดขวาง

`src/obstacle_avoider.py` **ไม่สั่งเลี้ยวเอง** อ่านเซนเซอร์ ตัดสินใจ แล้ว publish คำเดียวลง
`/lane/avoid` ที่ 10 Hz: `follow` / `avoid` (เกาะเลนซ้าย) / `stop` (ใกล้เกินกว่าจะหลบทัน)
ที่ต้องแยกโหนดเพราะ **`/cmd_vel_twist` ต้องมี publisher เจ้าเดียว** ผลพลอยได้คือเปิดปิดได้ระหว่างที่รถวิ่งอยู่

**สองเซนเซอร์ ทางเดินเดียว** — `/scan` (lidar 4 Hz รอบตัว) + `/scan_depth` (depth 10 Hz เฉพาะหน้า)
เป็น LaserScan เหมือนกัน `scan_topics` จึงเป็นลิสต์ และ **ทุกอย่างวัดใน `base_link` ผ่าน TF**
(เซนเซอร์สองตัวห่างกัน ~0.1 ม. = 8% ของระยะ trigger)
เสริมกันไม่ซ้ำกัน: lidar เป็นตัวเดียวที่มองข้าง/หลังได้ ส่วน depth เร็วกว่า 2–3 เท่าและเห็นของเตี้ยกว่า

**สองกรอบที่ใช้ตัดสิน และอยู่คนละเฟรมกัน** — นี่คือหัวใจของการออกแบบ

| กรอบ | เฟรม | ใช้ตอบ |
|---|---|---|
| FRONT | `base_link` (หันตามรถ) | เลนที่วิ่งอยู่โดนขวางไหม — `abs(y) ≤ corridor_half_width`, `x ≤ trigger_distance` |
| BEHIND | เลนเดิมที่ออกมา (คาไว้ตอนเริ่มหลบ) | เลนที่จะกลับเข้าไปโล่งจริงไหม — `abs(y) ≤ lane_half_width`, `side_x_min..max` (เริ่มจาก**หลังท้ายรถ**) |

**ออกง่าย กลับยาก**: FRONT ติด = เข้า `avoid` ทันที แต่จะกลับได้ต่อเมื่อครบทั้ง BEHIND โล่งต่อเนื่อง
`side_clear_time` **และ** ออกมาแล้วจริง `min_clearance` **และ** FRONT โล่ง
(FRONT ยังไม่โล่ง = ไม่กลับเด็ดขาด — แถวของสิ่งกีดขวางคือการหลบครั้งเดียว ไม่ใช่ครั้งละก้อน
ไม่งั้นกลายเป็นส่ายเข้าออกทุก ~0.5 ม.)

**ลิดาร์เป็นคนตัดสินว่าพ้นแล้ว ไม่ใช่ระยะ** — `pass_distance` เหลือหน้าที่เดียว: ใช้เมื่อกรอบข้าง
**ไม่เคยจับอะไรได้เลย** ตลอดการหลบ (ของเตี้ยกว่าระนาบลิดาร์ มองจากข้างเหมือนถนนโล่งเป๊ะ)
และนับ **จากตอนเทียบข้าง** ไม่ใช่จากตอนเริ่มหลบ ตัวเลขจึงหมายถึงความยาวสิ่งกีดขวาง + ความยาวรถ
และไม่ต้องจูนใหม่เมื่อเปลี่ยน `trigger_distance` · log บอกว่าใช้ทางไหน:
`lane behind clear for 0.8 s, 0.42 m past` (ลิดาร์) หรือ `never seen abeam, 0.50 m past` (ระยะ)

**สองระนาบ สองความสูง** — วัดจริง: กล่องกีดขวางสูง 0.11–0.19 ม. **ต่ำกว่าระนาบลิดาร์ 0.194 ม.**
สแกน 1600 เรย์ไม่มีเรย์เสียแม้แต่อันเดียวทั้งที่จมูกรถชนอยู่
กล้อง depth ช่วยได้ก็ต่อเมื่อ **`scan_height: 40`** (nav ใช้ 10 ซึ่งเป็นระนาบ 0.176 ม. สูงกว่ากล่องเหมือนกัน)
เพราะแถวที่**ต่ำกว่า**แกนกล้องคือแถวที่ก้มลงไปโดน — `fy = 484.2`, ±20 แถว = ก้ม 2.37°:
ที่ 2.0 ม. เห็นของสูง 0.093 ม. (ยิ่งไกลยิ่งเห็นของเตี้ยกว่า ซึ่งเข้าทางพอดี)

**`trigger_distance` คือปุ่มหลัก และถูกกำหนดด้วยเรขาคณิต ไม่ใช่รสนิยม** — `R = 0.258/tan(0.50) = 0.472` ม.
เปลี่ยนเลน 0.66 ม. แบบสองส่วนโค้ง: เหวี่ยง 30° ต้องใช้ 2.46 ม. · 40° ใช้ 1.81 ม. · 50° ใช้ 1.42 ม.
กล้อง CSI เห็น ±48.8° เกิน ~40° เส้นกลางเลนหลุดเฟรม แล้ว follower ขึ้น `lane lost` **กลางการแซง**
ต่ำกว่า ~1.0 ม. กล้องประคองเส้นผ่านการเหวี่ยงไม่ได้ ไม่ว่าจะจูนอย่างอื่นดีแค่ไหน (+ หัวรถล้ำ `base_link` อีก ~0.2 ม.)

**เงียบ ≠ ปลอดภัย** — ถ้า scan ทุกตัวเงียบ มันจะ**ค้างสถานะเดิม** ไม่ตกกลับไป `follow`
(lidar หลุดกลางคันต้องไม่ทำให้รถตัดกลับเข้าไปชน) ส่วน `/odom` ที่หายจะข้ามเงื่อนไขระยะพร้อม warning
แทนที่จะค้างใน `avoid` ตลอดกาล

**อีกทางหนึ่ง: depth เข้า Nav2 costmap** — `depth_scan_launch.py` แปลง `/realsense_depth` เป็น
`/scan_depth` เข้า **local costmap เท่านั้น** (`use_depth_scan:=false` เพื่อปิด)
เข้า global ไม่ได้เพราะ FOV แค่ ~67° จะ clear กำแพงในแมพทิ้งทันทีที่รถหันหนี
ต้องมี frame `depth_scan_link` แยก เพราะ `depthimage_to_laserscan` ไม่ประทับ frame กล้องให้
ถ้าใช้ optical frame ตรง ๆ สิ่งกีดขวางจะถูกหมุน 90° ลงไปใต้พื้น

---

<a id="trouble"></a>
## 5. แก้ปัญหา

| อาการ | สาเหตุที่เจอบ่อยที่สุด |
|---|---|
| รถวิ่งวน | รถจอดค้างในเขต inflation (cell `253`) — teleop ออกที่โล่งก่อน · ถ้าวนในที่โล่ง ให้กลับไปตรวจการแปลงมุมเลี้ยวใน bridge |
| Nav2 บอกว่ากำลังวิ่งแต่รถนิ่ง | `ros2 topic info -v /cmd_vel_nav` มีสองชนิดปนกัน (`enable_stamped_cmd_vel`) |
| goal fail ใน ~13 ms | TF ขาด — `tf2_echo map base_link` · มักเพราะกด Stop/Play ใน Isaac Sim |
| `Received map message is malformed` | มีสอง stack รันซ้อนกัน — `ros2 node list \| sort \| uniq -d` |
| RViz ไม่เห็นแมพ | `/map` เป็น transient local ไฟล์ `.rviz` ในนี้ตั้งถูกแล้ว ถ้าตั้งเองต้องไม่ใช่ Volatile |
| แก้ config แล้วไม่มีอะไรเปลี่ยน | ลืม `colcon build` (หรือใช้ไฟล์ใน `~/.ros/` แทน) |

**restart ฝั่ง ROS อย่างเดียว** (อย่าแตะ Isaac Sim — มันถือ scene state และเปิดใหม่กินเวลาเป็นนาที):

```bash
pkill -f 'ros2 launch qcar2_isaac_nav2'
pgrep -f '/opt/ros/jazzy/lib/nav2' | xargs -r kill -9
pgrep -f 'cartographer_ros/'       | xargs -r kill -9
pgrep -f 'rtabmap'                 | xargs -r kill -9
ros2 daemon stop && ros2 daemon start
```

> `pkill -f` แมตช์ command line ของ shell ตัวเองด้วย — คำสั่งที่มีทั้ง pattern และ launch string
> จะฆ่า shell กลางสคริปต์ (exit 144, บรรทัดถัดไปถูกข้ามเงียบ ๆ) แยกคำสั่ง kill กับ relaunch เสมอ
