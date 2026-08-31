# qcar2_isaac_nav2

นำทางอัตโนมัติสำหรับ Quanser QCar2 ใน NVIDIA Isaac Sim บน ROS 2 Jazzy
แบ่งเป็น 2 เฟส: **เก็บแมพ** แล้วค่อย **นำทาง**

มี 2 เส้นทางให้เลือก ต่างกันแค่ **ครึ่งที่เป็นเซนเซอร์** ครึ่งที่เป็น Nav2 (planner,
controller, BT, bridge) ใช้ร่วมกันทั้งคู่:

```
เส้นทาง LIDAR (ค่าตั้งต้น)
เฟส 1  MAPPING       Cartographer SLAM  ->  ขับเก็บแมพ  ->  save เป็น .yaml/.pgm
เฟส 2  NAVIGATION    map_server + AMCL  ->  Nav2        ->  กด 2D Goal Pose

เส้นทาง CAMERA (V-SLAM, ไม่ใช้ lidar เลย)
เฟส 1  MAPPING       RTAB-Map RGB-D     ->  ขับเก็บแมพ  ->  .db + .yaml/.pgm
เฟส 2  NAVIGATION    map_server + RTAB-Map localization  ->  Nav2
```

Isaac Sim เป็นคนส่ง `/clock`, `/scan`, `/imu`, `/odom` และ TF `odom -> base_link -> {lidar, imu, ...}`
เสมอ ทั้งสองเส้นทาง **ไม่มีใครแตะ `odom -> base_link`** ต่างกันแค่ว่าใคร publish `map -> odom`:
เฟส 1 lidar ได้จาก Cartographer, เฟส 2 lidar ได้จาก AMCL,
ส่วนเส้นทางกล้องได้จาก RTAB-Map ทั้งสองเฟส

---

## กฎเหล็ก

**กด PLAY ใน Isaac Sim ก่อนสั่ง launch เสมอ และห้ามกด Stop/Play ระหว่างที่ ROS รันอยู่**
เพราะ `/clock` จะรีเซ็ตเป็น 0 แล้ว Cartographer ตายทันที ทำให้ `map -> odom` หาย
และทุก goal จะ fail ใน ~13 ms

**ห้ามเปิดสอง stack ซ้อนกัน** — เช็คด้วย `ros2 node list | sort | uniq -d` ต้องไม่มีอะไรออกมา

ถ้าต้อง restart:

```bash
pkill -f 'ros2 launch qcar2_isaac_nav2'
pgrep -f '/opt/ros/jazzy/lib/nav2' | xargs -r kill -9
pgrep -f 'cartographer_ros/'       | xargs -r kill -9
pgrep -f 'rtabmap'                 | xargs -r kill -9
ros2 daemon stop && ros2 daemon start
```

---

## วิธีรัน

### เฟส 1 — เก็บแมพ

```bash
source install/setup.bash

# terminal 1 : SLAM + RViz
ros2 launch qcar2_isaac_nav2 qcar2_mapping_launch.py

# terminal 2 : ขับรถ (ขับช้า ๆ เพราะ /scan ออกแค่ ~4 Hz)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.6 -p turn:=0.5 -r /cmd_vel:=/cmd_vel_twist

# terminal 3 : เซฟแมพ แล้ว build ให้ install เห็น
ros2 run qcar2_isaac_nav2 save_map.sh qcar2_map
colcon build --packages-select qcar2_isaac_nav2
```

ต้องขับให้ครบทุกทางที่จะใช้นำทางจริง เพราะ `allow_unknown: false` planner จะไม่วิ่งผ่านพื้นที่ที่ยังไม่ได้แมพ

### เฟส 2 — นำทาง

```bash
source install/setup.bash
ros2 launch qcar2_isaac_nav2 qcar2_navigation_launch.py
# ระบุแมพอื่นได้: map:=/abs/path/to/other_map.yaml
```

รอให้ขึ้น `Managed nodes are active` **ครบทั้ง 2 ตัว** แล้วใน RViz กด
**2D Pose Estimate** (เฉพาะกรณีรถไม่ได้อยู่จุดเดียวกับตอนเริ่มแมพ) ตามด้วย **2D Goal Pose**

เช็คว่าปกติ:

```bash
ros2 run tf2_ros tf2_echo map base_link   # ต้องหาเจอ
ros2 topic hz /cmd_vel_twist              # ต้องมีค่าเมื่อมี goal
ros2 node list | sort | uniq -d           # ต้องว่าง
```

---

## เส้นทางกล้อง — V-SLAM ด้วย RGB-D (ไม่ใช้ lidar)

เก็บแมพด้วย **กล้อง depth** แทน lidar ใช้ RTAB-Map ทำ RGB-D SLAM: ดึง visual feature
จากภาพสี, ปิด loop จากหน้าตาของสถานที่, แล้วสะสมภาพ depth เป็น **voxel map 3 มิติ**
Nav2 วางแผนใน 2 มิติอยู่ดี RTAB-Map เลยฉาย voxel map ลงมาเป็น occupancy grid ปกติบน `/map`
ให้ costmap กินต่อโดยไม่ต้องแก้อะไร

แมพ 3 มิติตัวจริงอยู่ใน `/cloud_map` (และไฟล์ `.ply` ที่ export ไว้) ส่วน `/map` คือเงา 2 มิติของมัน

### ขั้นที่ 0 — ฝั่ง Isaac Sim (ทำครั้งเดียว ตอน **หยุด** sim)

RGB-D SLAM ต้องการ depth ที่ **registered** กับภาพสี — จุดศูนย์กลางเลนส์เดียวกัน intrinsics
เดียวกัน timestamp เดียวกัน — เพราะมันไปอ่านค่า depth ตรงพิกเซลที่เจอ feature พอดี
asset ของ QCar2 จำลอง D435 จริง คือ `realsenseRGB` กับ `realsenseDepth` ห่างกัน 37 มม.
ที่ระยะ 1 ม. เพี้ยนไป ~18 พิกเซล ทุก feature จะได้ depth ของของที่อยู่ข้าง ๆ มัน
`/realsense_depth` เดิมจึงใช้กับงานนี้ **ไม่ได้** (ใช้กับ `depthimage_to_laserscan` ได้ปกติ
เพราะอันนั้นอ่านแค่แถวเดียว)

```bash
# Window > Script Editor ใน Isaac Sim แล้ว paste ไฟล์นี้ Run (sim ต้องหยุดอยู่)
src/qcar2_isaac_nav2/scripts/isaac_add_rgbd_camera.py
```

สคริปต์สร้าง Camera prim เพิ่ม 1 ตัว**ใต้ `realsenseRGB`** (สืบทอด Xform มาเป๊ะ ๆ) แล้วให้
render product ตัวเดียวป้อน publisher 3 ตัว สี/depth จึงออกจาก render product เดียวกัน
= timestamp ตรงกัน ไม่ต้องใช้ approximate sync

กด PLAY แล้วเช็ค:

```bash
ros2 topic hz /realsense/color/image_raw    # ~10 Hz
ros2 topic hz /realsense/depth/image_raw    # เท่ากัน stamp ตรงกัน
ros2 topic echo /realsense/camera_info --once
```

ถ้าเฟรมเรตตกมาก (เคยวัดได้เหลือ 1.6 Hz) แปลว่ากล้องตัวอื่นในซีนแย่ GPU อยู่ — asset มี
render product 3 ตัวที่ publish ทิ้งเปล่า ๆ ปิดด้วย `scripts/isaac_camera_streams.py`
(`frameSkipCount` **ไม่ช่วย** เพราะมันหรี่แค่การ publish ไม่ได้หรี่การ render)

### เฟส 1 — เก็บแมพด้วยกล้อง

```bash
source install/setup.bash

# terminal 1 : RTAB-Map + RViz
ros2 launch qcar2_isaac_nav2 qcar2_vslam_mapping_launch.py

# terminal 2 : ขับช้า ๆ ให้ครบทุกทาง
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r cmd_vel:=/cmd_vel_twist -p speed:=0.3 -p turn:=0.3

# terminal 3 : เซฟ (ต้องเซฟ**ตอนที่ terminal 1 ยังรันอยู่**) แล้ว build
ros2 run qcar2_isaac_nav2 save_vslam_map.sh qcar2_vslam_map
colcon build --packages-select qcar2_isaac_nav2
```

ได้ผลลัพธ์ 2 ชิ้น และ **ต้องเก็บทั้งคู่**:

| ไฟล์ | ใครใช้ |
|---|---|
| `~/.ros/qcar2_vslam.db` | RTAB-Map — pose graph + visual words ที่ใช้ relocalise ตอนเฟส 2 |
| `maps/qcar2_vslam_map.yaml` + `.pgm` | `map_server` — static layer ของ costmap |

ทำไมต้องมี `.pgm` ทั้งที่ RTAB-Map เสิร์ฟ `/map` เองได้: การให้มันเสิร์ฟแปลว่าต้องอุ้ม
occupancy grid ของ **ทุก node** ไว้ใน working memory แมพนี้ (253 node, 12 ม., 3D) ดันโปรเซส
ทะลุ 6 GB จน OOM killer ฆ่า Isaac Sim ทิ้ง ส่วน `.pgm` หนักไม่กี่ร้อย kB และเป็นภาพฉาย 2 มิติ
อันเดียวกับที่ Nav2 กินอยู่แล้ว

ปิดท้ายด้วย Ctrl-C ที่ terminal 1 เพื่อให้ฐานข้อมูลถูกปิดอย่างถูกต้อง

### เฟส 2 — นำทางด้วยกล้อง

```bash
source install/setup.bash
ros2 launch qcar2_isaac_nav2 qcar2_vslam_navigation_launch.py
```

RTAB-Map ขึ้นในโหมด localization (`Mem/IncrementalMemory=false`): โหลด `.db` เดิม เทียบเฟรม
RGB-D สด ๆ กับเฟรมที่เก็บไว้ แล้ว publish `map -> odom` แทน AMCL ที่เหลือเหมือนเดิมทุกอย่าง

**ห้ามเปิด AMCL หรือ stack lidar ค้างไว้พร้อมกัน** — จะมีคน publish `map -> odom` สองคนแย่งกัน
และ `/map` สองเจ้าขนาดไม่เท่ากันจะได้ `Received map message is malformed. Rejecting.` ไม่จบไม่สิ้น

อาร์กิวเมนต์ที่ควรรู้:

| อาร์กิวเมนต์ | ค่าตั้งต้น | ความหมาย |
|---|---|---|
| `start_at_origin` | `true` | สมมติว่ารถเริ่มที่จุดกำเนิดของแมพเลย ไม่ต้องรอ relocalise — จริงใน Isaac Sim เพราะมัน respawn รถที่เดิมเสมอ และกินแรมแค่ ~0.7 GB |
| | `false` | บังคับให้ RTAB-Map **หาตัวเองจากกล้องจริง ๆ** (คือการทดสอบ V-SLAM ที่ซื่อสัตย์ที่สุด) แต่กิน ~2.7 GB และ ~0.6 วิ/เฟรม และแมพต้องถูกเก็บมาตอนกล้อง expose ถูกต้อง |
| `database_path` | `~/.ros/qcar2_vslam.db` | ฐานข้อมูลจากเฟส 1 |
| `map` | `maps/qcar2_vslam_map.yaml` | ภาพฉาย 2 มิติที่ `map_server` เสิร์ฟ |
| `use_depth_scan` | `true` | ป้อน `/scan_depth` เข้า local costmap เพิ่ม (ดูหัวข้อถัดไป) |

### กับดักที่เจอมาแล้ว

**`RGBD/MaxOdomCacheSize` ต้องเป็น 1** — ค่าตั้งต้น 10 ทำให้ RTAB-Map รอ localization ที่ตรงกัน
หลาย ๆ ครั้งก่อนจะยอม publish รถที่ **จอดนิ่ง** อยู่ตอนเริ่มรันไม่ผลิต odometry pose ใหม่เลย
cache เลยไม่มีวันเต็ม log วน `Localization was good, but waiting for another one to be more accurate`
`map -> odom` ไม่เคยออก และทุก goal fail ด้วย `map does not exist` โดยไม่มีบรรทัดไหนเป็น error

**`Mem/InitWMWithAllNodes` ต้องผูกกับ `start_at_origin`** เพราะสองตัวนี้ตอบคำถามเดียวกัน คือ
"RTAB-Map ต้องออกไปหาตัวเองในแมพไหม" ถ้าตั้งผิดคู่กัน (working memory ว่าง + ไม่ได้บอกตำแหน่ง)
มันจะเงียบ ๆ ไม่ publish `map -> odom` ตลอดกาล

**เปิดแมพเดิมแล้วเห็นแมพแค่นิดเดียว** — ปกติ ไม่ใช่แมพหาย: `/map` กับ cloud ประกอบจาก
working memory เท่านั้น ถ้ายังไม่ relocalise สำเร็จ RTAB-Map จะเปิด session ใหม่ที่ยังไม่ต่อกับ
กราฟเก่า `Mem/InitWMWithAllNodes=true` คือตัวที่ดึงทั้งกราฟกลับเข้ามา

**อย่าเรียก `/rtabmap/backup`** ทั้งที่ชื่อเหมือนแค่ก๊อปไฟล์ แต่มันจริง ๆ คือ save + copy +
**re-initialise working memory** พอ `/map` ยุบเหลือเท่าที่กล้องมองเห็นจากจุดที่รถจอด
`save_vslam_map.sh` ก็จะเซฟทับแมพดี ๆ ด้วยเศษ 5x5 ม. อย่างเต็มใจ (สคริปต์เลยไม่เรียกให้)

**loop closure ถูกปฏิเสธด้วย `Not enough inliers 0/15`** — ไปดู**ภาพสี**ก่อน อย่าเพิ่งไปแตะ
พารามิเตอร์ SLAM นั่นคือหน้าตาของกล้องที่ expose พังจนขาวโพลน ไม่ใช่ฉากที่ texture น้อย
แมพที่เก็บมาแบบนั้นสร้างและนำทางได้ปกติด้วย odometry ล้วน ๆ แค่ไม่มีสีและไม่มี feature เลย
พอ expose ถูก ฉากนี้ปิด loop ได้ปกติ (31 global + 6 proximity ในระยะขับ 35 ม.)

**`view_only:=true` ปลอดภัยต่อฐานข้อมูล แต่ไม่ปลอดภัยต่อแรม** — ประกอบ cloud 3 มิติเต็ม ๆ
ของแมพ 12 ม. กินไป 6 GB จน OOM killer เอา Isaac Sim ไปด้วย ปิด Isaac Sim ก่อน หรืออ่าน `.ply` แทน

**depth ของ Isaac เป็น 32FC1** ต้องบอก `Mem/DepthCompressionFormat=.png` ตรง ๆ ไม่งั้น `.rvl`
(ซึ่งเป็นฟอร์แมต 16 บิต) จะ warn แล้วตกกลับไป `.png` ทุกรอบ หรือถ้าบังคับก็ตัดทุกอย่างที่เกิน 65 ม. ทิ้ง

### ทำไม odometry ยังมาจาก Isaac ไม่ใช่ `rgbd_odometry`

wheel odometry ของ Isaac ที่นี่แทบเป็น ground truth และ `rgbd_odometry` จะมีประโยชน์ก็ต่อเมื่อมัน
publish `odom -> base_link` ตัวที่สอง ซึ่งจะไปแย่งกับตัวที่ Isaac เป็นเจ้าของอยู่ RTAB-Map ยังทำงาน
ส่วนที่เป็นภาพครบทุกอย่าง (feature, loop closure, grid) ถ้าอยากได้ visual odometry จริง ๆ ต้องไปปิด
TF publisher ในกราฟ drive ของ Isaac ก่อน

---

## (ทางเลือก) ใช้ depth camera ช่วยหลบสิ่งกีดขวาง

QCar2 มี RealSense อยู่บนตัวรถจริง และใน USD ก็มี prim
`/qcar2/base_link/realsenseDepth` อยู่แล้ว **แต่เป็น Xform เปล่า ๆ** — ไม่มี Camera
อยู่ข้างใน และไม่มี OmniGraph ตัวไหน publish ออกมาเลย (ต่างจากฝั่ง RGB ที่ต่อครบ
เป็น `realsenseRGB/Realsense_RGB` + graph `ros_qcar2_realsense_rgb`)

ข่าวดีคือ `realsenseDepth` อยู่ใน `targetPrims` ของ `ROS2PublishTransformTree` แล้ว
เพราะฉะนั้น TF `base_link -> realsenseDepth` มีให้ใช้ตั้งแต่แรก เหลือแค่ต่อกล้องกับ
ตัว publish

### ขั้นที่ 1 — ฝั่ง Isaac Sim (ทำครั้งเดียว ตอน **หยุด** sim)

เปิด **Window > Script Editor** แล้ววางไฟล์นี้ลงไปทั้งไฟล์ กด Run:

```
src/qcar2_isaac_nav2/scripts/isaac_add_depth_camera.py
```

สคริปต์จะ copy intrinsics จากกล้อง RGB มาสร้าง `realsenseDepth/Realsense_Depth`
แล้วสร้าง graph `ros_qcar2_realsense_depth` เลียนแบบ graph ของ RGB
รันซ้ำได้ (สร้างทับของเดิม) จากนั้นกด PLAY แล้วเช็ค:

```bash
ros2 topic hz /realsense_depth                     # ~10 Hz
ros2 topic echo /realsense_depth_camera_info --once
```

อย่าลืม **Save** stage ไม่งั้นเปิดใหม่ต้องรันสคริปต์อีกรอบ

### ขั้นที่ 2 — ฝั่ง ROS

ไม่ต้องทำอะไร `qcar2_navigation_launch.py` ต่อให้แล้ว:

```
/realsense_depth ──> depthimage_to_laserscan ──> /scan_depth ──> local_costmap obstacle_layer
```

ปิดได้ด้วย `use_depth_scan:=false`

เช็คว่าเข้า costmap จริง:

```bash
ros2 topic hz /scan_depth
grep "Subscribed to Topics" /tmp/nav.log    # ต้องเห็น "scan depth_scan"
```

### ทำไมต้องมี frame `depth_scan_link`

`depthimage_to_laserscan` **ไม่ได้** ประทับ frame ของกล้องลงบน scan ที่มันสร้าง —
ค่ามุมที่มันคายออกมาเป็นแบบ LaserScan ปกติ (หมุนรอบ +z, ศูนย์อยู่ที่ +x) แต่
`realsenseDepth` ของ Isaac เป็น **optical frame** (+z ชี้ไปข้างหน้า, +y ชี้ลง)
ถ้าเอา scan ไปแปะ frame `realsenseDepth` ตรง ๆ สิ่งกีดขวางทุกตัวจะถูกหมุน 90°
ลงไปอยู่ใต้พื้น

launch เลย publish static TF `base_link -> depth_scan_link` ขึ้นมาอีกอัน
ตำแหน่งเดียวกับกล้อง (0.095, -0.003, 0.176 m — ลอกจาก USD) แต่ไม่หมุน
แล้วส่งชื่อนี้ให้ `output_frame`

### ข้อจำกัดที่ต้องรู้

- **เข้าเฉพาะ local costmap** — depth มี FOV แค่ ~67° ถ้าปล่อยให้มันไป
  `clearing` ใน global costmap กำแพงในแมพจะถูกลบทิ้งทันทีที่รถหันหนี
  ถ้าอยากให้ planner หลบของที่ไม่ได้อยู่ในแมพจริง ๆ ให้เพิ่ม `depth_scan` ใน
  global costmap แบบ `clearing: false` เท่านั้น
- **`scan_height: 10` คือค่าที่ห้ามเพิ่มมั่ว ๆ** — มันคือจำนวนแถวพิกเซลรอบแกนกลาง
  ที่เอามายุบเป็น scan กล้องอยู่สูง 0.176 m มองตรง แถวที่ต่ำกว่ากลาง `n` พิกเซล
  จะมองเห็น **พื้น** ที่ระยะ `0.176 / tan(atan(n/fy))` โดย `fy ≈ 484`
  ที่ 10 แถว (±5) พื้นตกอยู่ที่ ~17 m ซึ่งเกิน `range_max: 3.0` เลยถูกทิ้ง
  ถ้าเพิ่มเป็นหลักสิบปลาย ๆ พื้นจะกลายเป็นกำแพงปลอมข้างหน้ารถทันที
- **ยังไม่ได้ต่อเข้า Cartographer** — `num_point_clouds = 0` เหมือนเดิม เฟส 1
  ยังใช้ lidar อย่างเดียว
- ถ้ายังไม่ได้รันสคริปต์ฝั่ง Isaac ก็ไม่พัง — `expected_update_rate` ปริยายเป็น 0.0
  แปลว่า "ไม่มีวันหมดอายุ" source ที่ไม่มีใคร publish เลยเงียบ ๆ ไปเฉย ๆ

---

## กล้อง CSI 360 องศา + YOLO object detection

QCar2 มีกล้อง CSI 4 ตัวรอบคัน (`csi_front`, `csi_back`, `csi_left`, `csi_right`)
ตัวละ **97.6 องศา** รวมเป็น 390 องศา คือปิดวงได้จริงและเหลือซ้อนกันนิดหน่อยตรงมุม

**asset มี graph ให้ครบทั้ง 4 ตัวอยู่แล้ว ต่อสายไว้ถูกต้องหมด** แค่ 3 ตัว
(`csi_back`, `csi_left`, `csi_right`) ถูกปิดไว้ด้วย `active = False` ของ USD เฉย ๆ
งานที่ต้องทำจึงเป็นแค่ "เปิดสวิตช์" ไม่ใช่สร้างใหม่

```
csi_left ─┐                    ┌─ csi_right
          ├─> yolo_detector ───┤
csi_front ┘   (YOLO ตัวเดียว)   └─ csi_back
                    │
                    ├─> /csi_<pos>/detections   vision_msgs/Detection2DArray
                    ├─> /csi_<pos>/annotated    ภาพวาดกรอบแล้ว
                    └─> /csi/mosaic             ภาพ 4 มุมต่อกันเป็น 2x2
```

เป็นส่วนเสริมด้าน sensing ล้วน ๆ **ไม่ยุ่งกับ Nav2 เลย** ไม่ publish TF ไม่เพิ่ม costmap layer
ไม่แตะ cmd_vel เปิด/ปิดทับ stack ที่รันอยู่ได้

### ขั้นที่ 0 — ติดตั้ง ultralytics (ทำครั้งเดียว)

```bash
pip install --user --break-system-packages ultralytics
```

torch 2.10+cu128 มีอยู่แล้วจาก Isaac Sim (มาทาง PYTHONPATH) pip เห็นและไม่โหลดซ้ำ
`--break-system-packages` จำเป็นเพราะ Ubuntu 24.04 mark python ระบบเป็น externally-managed

ตัวโมเดล (`yolo11n.pt`, 5.4 MB) โหลดอัตโนมัติครั้งแรกลง `~/.cache/qcar2_yolo/`

### ขั้นที่ 1 — ฝั่ง Isaac Sim (ทำครั้งเดียว ตอน **หยุด** sim)

สำรอง stage ก่อน เพราะสคริปต์นี้เขียนทับ graph `ros_qcar2_csi_front` ของเดิม:

```bash
cd ~/Documents/Quanser/qcar2_nvidia/isaac_sim/Collected_qcar2_workspace
cp qcar2_workspace.usd qcar2_workspace.usd.bak-precsi
```

แล้วเปิด Window > Script Editor ใน Isaac Sim (sim ต้องหยุด) paste ไฟล์นี้แล้ว Run:

```
src/qcar2_isaac_nav2/scripts/isaac_add_csi_cameras.py
```

สคริปต์ **ไม่ได้สร้าง graph ใหม่** มันเปิด `active` ของ 3 ตัวที่ถูกปิดไว้ แล้วแก้ค่า 3 อย่าง
บน graph ทั้ง 4:

1. `topicName` จาก `csi_back` เปล่า ๆ เป็น `csi_back/image_raw` (เปลี่ยนพร้อมกันทั้ง 4 ตัว
   รวม csi_front ด้วย เพราะกล้องชุดนี้ใช้ด้วยกัน)
2. `frameSkipCount = 5` (~10 Hz) ของเดิม**ไม่ได้ตั้งไว้เลย** คือ publish ทุก tick ~20 Hz
   ซึ่งเป็นเหตุผลที่ csi_front เคยเป็น publisher ที่แพงที่สุดในซีน
3. `inputs:enabled = False` บน render product (สร้างให้ถ้ายังไม่มี) เพื่อให้
   `isaac_camera_streams.py` เอาไปเปิด/ปิดตาม preset ได้

**ทำไมต้องเน้นว่ามันมีอยู่แล้ว** — prim ที่ `active = False` จะ "หายไป" จาก stage ทั้งดุ้น
ลูกของมันไม่ถูก compose เลย traverse หา OmniGraph ก็ไม่เจอ `ros2 topic list` ก็ไม่มี
มันดูเหมือน**ไม่มีอยู่จริง** มากกว่าดูเหมือนถูกปิด ถ้าเชื่อตามนั้นแล้วไปสร้าง graph ใหม่ทับ
จะกลายเป็นสร้างของซ้อนบน prim ที่มีอยู่แล้วใน `qcar2.usd` ที่ถูก reference เข้ามา

สคริปต์นี้แตะแต่ USD attribute ล้วน ๆ ไม่ต้องใช้ omni.graph runtime เลย
เพราะฉะนั้นตรวจสอบกับ stage ที่ copy ออกมาได้ด้วย pxr เปล่า ๆ (ทำมาแล้ว)

**ไม่ได้เพิ่ม camera_info publisher** เพราะต้องไปผ่า graph ที่มาจาก referenced layer
ซึ่งเสี่ยงกว่ามาก และ pipeline นี้ไม่ต้องใช้: YOLO เป็น 2D ล้วน ๆ ส่วน RViz *Image* display
(ต่างจาก *Camera* display) ไม่อ่าน camera_info ค่อยเพิ่มตอนที่ต้อง project detection ไป 3 มิติ

**ย้อนกลับ**: ตั้ง `active = False` กลับ หรือใช้ backup — `active` ถูกเขียนเป็น override
ลง root layer ของ stage เท่านั้น ตัว `qcar2.usd` ที่ถูก reference ไม่เคยโดนแก้

จากนั้นเปิด render product ของ CSI (ดูขั้นที่ 2)

### ขั้นที่ 2 — เลือกว่าจะให้กล้องตัวไหน render

**นี่คือเรื่องสำคัญที่สุดของหัวข้อนี้** render product ถูก RTX render ทุกเฟรมไม่ว่าจะมีคน
subscribe หรือไม่ และ `frameSkipCount` หรี่แค่การ publish **ไม่ได้หรี่การ render**
ทางเดียวที่ได้ GPU คืนคือปิด render product ทิ้ง

`scripts/isaac_camera_streams.py` จึงทำงานเป็น preset เลือกด้วยตัวแปร `PRESET` ใน
Script Editor หรือ env `QCAR2_CAMERA_PRESET` ตอนรัน headless:

| preset | เปิดอะไร | ใช้ตอนไหน |
|---|---|---|
| `vslam` | RGB-D อย่างเดียว (1 render product) | เส้นทางกล้อง V-SLAM |
| `csi` | CSI ครบ 4 ตัว (4 render products) | YOLO 360 องศา |
| `csi_front` | CSI หน้าอย่างเดียว (1) | อยากรัน YOLO คู่กับ V-SLAM |
| `both` | ทั้งหมด (5) | ได้ทุกอย่างแต่ช้าลงทั้งกระดาน |
| `none` | ไม่เปิดเลย | นำทางด้วย lidar |

บน RTX 5060 นี้เคยวัดได้ว่าเปิดกล้อง 4 ตัวพร้อมกันทำให้คู่ RGB-D ที่ RTAB-Map กินเหลือ **1.6 Hz**
เพราะฉะนั้น CSI ครบวง กับ V-SLAM **ไม่ได้ตั้งใจให้รันพร้อมกัน**บนเครื่องนี้
ถ้าอยากได้ทั้งคู่จริง ๆ ใช้ preset `csi_front` คู่กับ `cameras:=front`

### ขั้นที่ 3 — รัน

```bash
source install/setup.bash
ros2 launch qcar2_isaac_nav2 qcar2_yolo_launch.py
```

RViz จะเปิดขึ้นมาพร้อม `/csi/mosaic` คือภาพ 4 มุมต่อกันเป็นตาราง 2x2 เรียงตามเข็มนาฬิกา
(หน้า, ขวา / หลัง, ซ้าย) แต่ละช่องมีขอบสีประจำกล้องเพราะวิวโกดังสีเทาทั้ง 4 มุมแยกกันไม่ออกจริง ๆ

เช็ค:

```bash
ros2 topic echo /csi_front/detections
ros2 topic hz /csi/mosaic
```

### ทำไมต้องมี /csi/mosaic

RViz **ไม่มี** grid layout ถ้าใส่ Image display 4 อันมันจะ dock เป็น 4 แท็บ คือเห็นทีละกล้อง
ทางเดียวที่จะได้ 2x2 จริง ๆ คือไปเขียน QMainWindow geometry เป็น hex ลงใน `.rviz`
ซึ่งพังทันทีที่มีคนลากขนาด panel ต่อภาพที่ฝั่ง node เลยได้ topic เดียวที่เห็นครบวง
และ record/replay เป็นสตรีมเดียวได้ด้วย

### อาร์กิวเมนต์

| อาร์กิวเมนต์ | ค่าตั้งต้น | ความหมาย |
|---|---|---|
| `cameras` | `front,back,left,right` | เลือกกล้องที่จะรัน YOLO |
| `model` | `yolo11n.pt` | nano เหมาะกับที่นี่เพราะ 4 กล้องแชร์ GPU ตัวเดียวที่ยัง render ซีนอยู่ `yolo11s.pt` แม่นกว่าแต่เฟรมเรตราวครึ่งเดียว |
| `confidence` | `0.35` | ต่ำกว่า default 0.5 เพราะโกดังใน Isaac อยู่นอก distribution ของโมเดลที่เทรนบน COCO |
| `imgsz` | `640` | ภาพ 820x410 ถูก letterbox มาที่ขนาดนี้ |
| `half` | `true` | fp16 เร็วขึ้นราว 1.5 เท่าบน GPU นี้ บน cpu จะถูกปิดอัตโนมัติ |
| `publish_annotated` | `true` | ปิดได้ถ้ากินแค่ `/detections` |

### กับดักที่เจอมาแล้ว

**ห้ามใช้ `cv_bridge` ในเครื่องนี้** — cv_bridge ของ Jazzy compile มากับ NumPy 1.x แต่
`~/.bashrc` source `setup_python_env.sh` ของ Isaac ซึ่งดัน NumPy 2.5.2 ขึ้นหน้า PYTHONPATH
แปลงภาพทีเดียว **segfault (exit 139)** ไม่มี traceback ให้ดู หน้าตาเหมือน GPU/driver พังมากกว่า
dependency ชนกัน:

```bash
python3 -c "from cv_bridge import CvBridge; import numpy as np; \
    CvBridge().cv2_to_imgmsg(np.zeros((4,4,3),np.uint8),'bgr8')"
# Segmentation fault (core dumped)
```

`src/yolo_detector.py` จึงแปลง `sensor_msgs/Image` เองด้วย NumPy ล้วน ๆ (`decode_rgb`/`encode_rgb`)
ไม่กี่บรรทัดและไม่ต้องพึ่ง extension ที่ compile มา **อย่าแก้กลับไปใช้ cv_bridge**

**ภาพขาวโพลน = auto exposure ถูกปิด** Isaac เขียน `omni:rtx:autoExposure:enabled = False`
ลงบนกล้องทุกตัวที่มีคนเอา render product ไปแปะ ซึ่งกล้อง CSI 3 ตัวไม่เคยมีมาก่อน
`isaac_add_csi_cameras.py` เลยบังคับเปิดไว้ให้ ถ้า YOLO เจอ 0 object **ไปดูภาพก่อน**
อย่าเพิ่งไปลด `confidence`

**`ros2 run` ทิ้ง process ค้าง** kill PID ที่ `ros2 run` คืนมาไม่ได้ฆ่า python ตัวจริง
มันกลายเป็น orphan ที่ยัง subscribe และยังเขียน log ทับอยู่ ตอนเจอผลลัพธ์แปลก ๆ ให้เช็คก่อน:

```bash
pgrep -af yolo_detector    # ต้องมีไม่เกิน 1
```

**ultralytics 8.4 เลิกใช้ `half`** เปลี่ยนเป็น `quantize` แล้ว ถ้าส่ง `half` ไปมันจะ warn
**ทุกครั้งที่เรียก predict** คือ ~40 บรรทัดต่อวินาที กลบ log ของ node จนหมด
node แปลงให้เองแล้ว (`half:=true` -> `quantize='fp16'`)

---

## โครงสร้างไฟล์

```
src/qcar2_isaac_nav2/
├── launch/
│   ├── qcar2_mapping_launch.py            เฟส 1 (lidar)
│   ├── qcar2_navigation_launch.py         เฟส 2 (lidar)
│   ├── qcar2_vslam_mapping_launch.py      เฟส 1 (กล้อง) RTAB-Map RGB-D
│   ├── qcar2_vslam_navigation_launch.py   เฟส 2 (กล้อง) RTAB-Map localization
│   ├── qcar2_yolo_launch.py               YOLO บนกล้อง CSI 360 องศา
│   ├── qcar2_cartographer_launch.py       (ของเดิม) เปิด cartographer อย่างเดียว
│   └── qcar2_slam_and_nav_bringup_launch.py  (ของเดิม) SLAM+Nav2 พร้อมกัน ไม่ใช้แมพที่เซฟ
├── config/
│   ├── qcar2_mapping.lua                  จูน Cartographer ของเฟส 1
│   ├── qcar2_nav2_amcl.yaml               พารามิเตอร์ Nav2 + AMCL ของเฟส 2
│   ├── qcar2_nav2_vslam.yaml              พารามิเตอร์ Nav2 ของเส้นทางกล้อง (VoxelLayer, ไม่มี lidar)
│   ├── qcar2_2d.lua                       (ของเดิม) ใช้กับ cartographer_launch
│   └── qcar2_slam_and_nav.yaml            (ของเดิม) ใช้กับ bringup_launch
├── behavior_trees/
│   ├── navigate_to_pose_ackermann.xml     BT ตัด Spin ออก
│   └── navigate_through_poses_ackermann.xml   BT ตัด Spin ออก
├── src/
│   ├── twist_stamped_to_twist.py          bridge Nav2 -> Isaac Sim
│   └── yolo_detector.py                   YOLO บนกล้อง CSI (ไม่ใช้ cv_bridge)
├── scripts/
│   ├── save_map.sh                        เซฟแมพ (lidar)
│   ├── save_vslam_map.sh                  เซฟแมพ (กล้อง) จาก /map ของ RTAB-Map
│   ├── isaac_add_depth_camera.py          เพิ่มกล้อง depth + ROS2 graph ใน Isaac Sim
│   ├── isaac_add_rgbd_camera.py           เพิ่มกล้อง RGB-D registered สำหรับ V-SLAM
│   ├── isaac_add_csi_cameras.py           สร้าง ROS2 graph ให้กล้อง CSI ครบ 4 ตัว
│   ├── isaac_camera_streams.py            preset เลือกว่าจะ render กล้องตัวไหน
│   └── isaac_apply_and_save.py            รันสคริปต์ 2 ตัวบนแบบ headless แล้วเซฟ stage
├── rviz/
│   ├── qcar2_mapping.rviz                 RViz เฟส 1 (lidar)
│   ├── qcar2_nav2.rviz                    RViz เฟส 2 (lidar)
│   ├── qcar2_vslam.rviz                   RViz เฟส 1 (กล้อง)
│   ├── qcar2_vslam_nav.rviz               RViz เฟส 2 (กล้อง)
│   ├── qcar2_yolo.rviz                    RViz mosaic 4 กล้อง + detections
│   └── qcar2_depth_view.rviz              ดูภาพ depth ดิบ ๆ
├── maps/                                  แมพที่เซฟไว้ (.yaml/.pgm/.pbstream/.ply)
├── CMakeLists.txt / package.xml           build + dependencies
└── rt_models/dummy_model                  (ของเดิม) ไม่ได้ใช้
```

## แต่ละไฟล์ทำอะไร

| ไฟล์ | หน้าที่ |
|---|---|
| `launch/qcar2_mapping_launch.py` | เปิด `cartographer_node` + `cartographer_occupancy_grid_node` + RViz — ตัวสร้าง `/map` และ TF `map -> odom` ระหว่างขับเก็บแมพ |
| `launch/qcar2_navigation_launch.py` | เปิด `map_server` + `amcl` + Nav2 ครบชุด + bridge + RViz พร้อมยัด path ของแมพและ BT เข้า params ตอนรัน |
| `launch/qcar2_vslam_mapping_launch.py` | เปิด `rtabmap` โหมด SLAM กินคู่ RGB-D จาก Isaac สร้าง `/map` + cloud 3 มิติ + TF `map -> odom` — และเป็นที่อยู่ของ `GRID_PARAMS`/`SLAM_2D_PARAMS` ที่อีกไฟล์ import ไปใช้ ทั้งสองไฟล์จะได้ไม่หลุดจากกัน |
| `launch/qcar2_yolo_launch.py` | เปิด `yolo_detector` + RViz mosaic ตรวจ `cameras:=` ตั้งแต่ตอน launch เลยว่าชื่อกล้องถูกไหม จะได้ไม่ไปเงียบ ๆ ตอน subscribe topic ที่ไม่มีอยู่ |
| `launch/qcar2_vslam_navigation_launch.py` | เปิด `rtabmap` โหมด localization + `map_server` + Nav2 + `point_cloud_xyz` (depth -> PointCloud2 ให้ VoxelLayer) + bridge — remap `/map`, `/cloud_map` ฯลฯ ของ RTAB-Map ไปไว้ใต้ `/rtabmap/` กันชนกับ `map_server` |
| `config/qcar2_mapping.lua` | ตั้ง `use_odometry = true` ให้ Cartographer กิน `/odom` ของ Isaac ด้วย ลด drift และคุมความถี่การสร้าง node ไม่ให้กิน CPU |
| `config/qcar2_nav2_amcl.yaml` | ค่าทั้งหมดของเฟส 2 — AMCL, costmap, MPPI controller, SmacPlannerHybrid, footprint, goal tolerance |
| `config/qcar2_nav2_vslam.yaml` | ก๊อปมาจากไฟล์บน เปลี่ยนเฉพาะครึ่งเซนเซอร์: ตัด `/scan` ทิ้งหมด, obstacle layer เป็น **VoxelLayer** กิน PointCloud2 จาก depth, ระยะ 12 ม. ทุกที่ — จูน planner/controller เมื่อไหร่ต้องแก้ให้ตรงกันทั้งสองไฟล์ |
| `behavior_trees/*_ackermann.xml` | BT ที่เอา `Spin` ออกแล้วใช้ `BackUp` แทน เพราะรถ Ackermann หมุนอยู่กับที่ไม่ได้ ต้องแก้ **ทั้งสองไฟล์** ไม่งั้น `bt_navigator` activate ไม่ผ่าน |
| `src/yolo_detector.py` | node เดียวกิน CSI ทุกตัว รัน YOLO ตัวเดียวร่วมกัน (มี lock กันไม่ให้ 4 callback แย่ง CUDA stream เดียวกัน + ทิ้งเฟรมที่ค้างคิวแทนที่จะไล่ทำของเก่า) publish detections/annotated/mosaic |
| `src/twist_stamped_to_twist.py` | แปลง `/cmd_vel_nav` (TwistStamped, yaw rate) เป็น `/cmd_vel_twist` (Twist, **มุมเลี้ยว**) ด้วย `δ = atan(ω·L/v)` — สำคัญที่สุดในแพ็กเกจนี้ |
| `scripts/isaac_add_depth_camera.py` | รันใน Isaac Sim Script Editor — สร้างกล้อง depth ใต้ `realsenseDepth` แล้วต่อ OmniGraph publish `/realsense_depth` + `/realsense_depth_camera_info` (asset เดิมมีแต่ Xform เปล่า ๆ ไม่มีกล้องและไม่มี graph) |
| `scripts/save_map.sh` | สั่ง `finish_trajectory` + `write_state` (.pbstream) แล้วเรียก `map_saver_cli` เขียน `.yaml`/`.pgm` ลง `maps/` ใน source |
| `scripts/save_vslam_map.sh` | เรียก `map_saver_cli` บน `/map` ของ RTAB-Map ลง `maps/` ใน source ต้องรันตอน `qcar2_vslam_mapping_launch.py` ยังเปิดอยู่ และ **จงใจไม่เรียก** `/rtabmap/backup` |
| `scripts/isaac_add_rgbd_camera.py` | รันใน Isaac Sim Script Editor — สร้าง Camera ใต้ `realsenseRGB` แล้วให้ render product เดียวป้อน `/realsense/color/image_raw` + `/realsense/depth/image_raw` + `/realsense/camera_info` depth จึง registered กับสีและ stamp ตรงกัน (idempotent) |
| `scripts/isaac_camera_streams.py` | preset เปิด/ปิด render product RTX render ทุกเฟรมไม่ว่ามีคน subscribe หรือไม่ ปล่อยไว้ครบ 4 ตัวคู่ RGB-D จะเหลือ 1.6 Hz — ตัวเดียวที่ได้ GPU คืนจริง |
| `scripts/isaac_add_csi_cameras.py` | รันใน Isaac Sim Script Editor — เปิด `active` ของ graph CSI 3 ตัวที่ asset ปิดไว้ แล้ว normalize topicName/frameSkipCount/enabled ของทั้ง 4 ตัว พร้อมบังคับ auto exposure เปิด เป็น USD attribute ล้วน ๆ ไม่ต้องใช้ omni.graph |
| `scripts/isaac_apply_and_save.py` | เอาสคริปต์ข้างบนไปรันกับ stage แบบ headless แล้วเซฟ ไม่ต้องเปิด GUI |
| `rviz/*.rviz` | ตั้ง `/map` เป็น Transient Local ไว้แล้ว ไม่งั้น RViz จะไม่เห็นแมพที่ publish แบบ latched |
| `maps/qcar2_map.yaml` + `.pgm` | แมพที่ `map_server` โหลดตอนเฟส 2 |
| `maps/qcar2_map.pbstream` | เซสชันของ Cartographer เอาไว้ต่อแมพเดิมหรือ export ใหม่ |
| `maps/qcar2_vslam_map.yaml` + `.pgm` | ภาพฉาย 2 มิติของแมพกล้อง ที่ `map_server` โหลดตอนเฟส 2 ของเส้นทางกล้อง |
| `maps/qcar2_vslam_cloud_cloud.ply` | cloud 3 มิติที่ export ไว้ เปิดดูได้โดยไม่ต้องรัน RTAB-Map (ซึ่งกินแรมหนัก) |
| `~/.ros/qcar2_vslam.db` | **ไม่ได้อยู่ใน repo** — pose graph + visual words ของ RTAB-Map ลบแล้วเส้นทางกล้องเฟส 2 ใช้ไม่ได้ ต้องขับเก็บใหม่ |
| `CMakeLists.txt` | install `launch/ config/ rviz/ behavior_trees/ maps/` และสคริปต์ทั้งหมด |
| `package.xml` | dependencies (nav2, cartographer_ros, rtabmap_slam/util/viz, vision_msgs, teleop_twist_keyboard, rviz2, depthimage_to_laserscan) — ultralytics ไม่ได้อยู่ในนี้เพราะเป็น pip ไม่ใช่ rosdep |
| `rviz/qcar2_yolo.rviz` | Image display บน `/csi/mosaic` เป็นหลัก ส่วนราย ๆ กล้องปิดไว้ (RViz dock เป็นแท็บ เห็นทีละอัน) |

---

## ทำไมค่าถึงตั้งแบบนี้

**`angular.z` ต้องแปลงหน่วยใน bridge** — Nav2 ส่ง `angular.z` เป็น **yaw rate (rad/s)**
แต่ Isaac Sim อ่านเป็น **มุมเลี้ยวล้อหน้า (rad)** จำกัดที่ ~0.5 rad วัดจริงบนซีนนี้ได้:

| สั่ง `angular.z` | รัศมีที่ได้ | `0.258/tan(สั่ง)` |
|---|---|---|
| 0.30 | 0.797 m | 0.834 m |
| 0.60 | 0.500 m | ชนลิมิตพวงมาลัย |
| 1.00 | 0.482 m | ชนลิมิตพวงมาลัย |

ถ้าส่งผ่านตรง ๆ รถจะเลี้ยวแรงกว่าที่ Nav2 สั่งหลายเท่า → เลยเส้นทาง → ถูกแก้ → เลยอีกฝั่ง → **วิ่งวน**
หลังแปลงแล้ว yaw rate ที่ได้จริงคลาดจากที่สั่งไม่เกิน 0.03 rad/s

**ขนาดรถวัดจาก TF จริง ไม่ได้เดา**

```
base_link -> hub_frontLeft  = ( 0.130,  0.056)
base_link -> wheel_rearLeft = (-0.128,  0.056)
=> wheelbase 0.258 m, ตัวถัง ~0.39 x 0.19 m
=> รัศมีวงเลี้ยวต่ำสุด = 0.258 / tan(0.5) ~= 0.47 m
```

จึงตั้ง `footprint` เป็น `[[0.21, 0.10], [0.21, -0.10], [-0.19, -0.10], [-0.19, 0.10]]`
และ `minimum_turning_radius` / `min_turning_r` = `0.5`

**`enable_stamped_cmd_vel: true` ทุกโหนด** — Nav2 1.3.12 บน Jazzy default เป็น `false` (Twist ธรรมดา)
แต่ bridge รับเป็น TwistStamped สองชนิดบน topic เดียวกันจะไม่ส่งข้อความถึงกันเลย
Nav2 วางแผนสวยงามแต่รถไม่ขยับ

**`yaw_goal_tolerance: 3.15`** = ไม่สนทิศตอนจอด รถหมุนอยู่กับที่ไม่ได้ ถ้าบังคับมุมมันจะวนรอบเป้าหมายไม่จบ
ทิศตอนเข้าเป้าให้ SmacPlannerHybrid จัดการ (มันวางแผนถึง goal *pose* อยู่แล้ว)

**`motion_model_for_search: "REEDS_SHEPP"`** ไม่ใช่ `DUBIN` — Dubins เดินหน้าอย่างเดียว
เป้าหมายอยู่ข้างหลัง 0.8 m จะกลายเป็นวนหลายเมตร และถ้าวงเลี้ยวไม่พอก็หาเส้นทางไม่เจอเลย
Reeds-Shepp ถอยหลังได้เหมือนรถกลับรถ 3 จังหวะ จึงต้องลด `PreferForwardCritic` เหลือ 1.5
(ถ้าไว้ 5.0 มันจะบล็อกช่วงถอยจนรถจอดนิ่ง) และใช้ `PathAngleCritic mode: 2`
กับ `PathAlignCritic use_path_orientations: true` ตามที่ Nav2 แนะนำสำหรับรถถอยหลังได้

**`CostCritic consider_footprint: true`** — ถ้าเป็น `false` MPPI เช็คชนเป็นวงกลมรัศมี inscribed
= 0.10 m ทั้งที่รถยาว 0.40 m (Nav2 เตือน `Inconsistent configuration in collision checking`)
ผลคือวางเส้นทางเฉี่ยวกำแพงแล้วค้าง ลด `batch_size` เหลือ 1000 ชดเชย control loop ยังได้ 20 Hz

ผลวัดจริง goal เดียวกัน จุดเริ่มเดียวกัน:

| | ก่อน | หลัง |
|---|---|---|
| ระยะที่วิ่ง | 9.07 m | 1.84 m |
| ระยะเส้นตรง | 1.92 m | 1.92 m |
| มุมที่หมุนรวม | 297° | 65° |
| ผล | timeout | SUCCEEDED 13 s |

**`local_costmap` ใช้ frame `odom` และไม่มี `static_layer`** — ถ้าใช้ frame `map` พร้อม static layer
local costmap จะกระตุกทุกครั้งที่ AMCL แก้ตำแหน่ง

**`inflation_radius: 0.30`** ไม่ใช่ 0.75 — เป่า 0.75 m รอบรถกว้าง 0.19 m ทำให้ประตูทุกบานดูวิ่งผ่านไม่ได้

**ห้ามใส่ list ว่างใน YAML** — `polygons: []` / `observation_sources: []` / `docks: []` ไม่มี type
ทำให้ `collision_monitor` กับ `docking_server` abort ตอนสตาร์ท
(`parameter_value_from failed ... No parameter value set`) ต้องใส่ค่าจริงแล้วปิดด้วย `enabled: False` แทน

**path ของแมพถูกยัดใน `qcar2_navigation_launch.py` ไม่ได้ส่งผ่าน `map:=` ของ nav2**
เพราะ argument นั้นมาเป็น params file อีกไฟล์ที่ scope `/**:` และ ROS 2 ให้ key ชื่อโหนดตรง ๆ
(`map_server:`) ชนะ wildcard เสมอไม่ว่าไฟล์ไหนมาทีหลัง — `yaml_filename` ว่างของเราจะชนะ
แล้ว `map_server` ขึ้นมาแบบไม่มีแมพ และต้องแทนที่ใน `OpaqueFunction` ด้วย
เพราะ `IncludeLaunchDescription` set launch_arguments ใน scope ลูก ทำให้
`LaunchConfiguration('map')` ที่ประเมินทีหลังอ่านค่าผิด

---

## แก้ปัญหา

**รถวิ่งวน** — เช็คก่อนว่ารถไม่ได้จอดค้างในเขต inflation รถที่เคยชนหรือวนเข้ากำแพงจะอยู่ในช่องที่
costmap ให้ค่า `253` (inscribed = รถเข้าไม่ได้) จากตรงนั้น planner จะออกเส้นทางเพี้ยนเป็นสิบเมตร
และ MPPI สั่ง ~0 m/s ตลอด ให้ teleop ถอยออกมาที่โล่งก่อนแล้วค่อยสั่ง goal ใหม่
ถ้าวนทั้ง ๆ ที่อยู่ในที่โล่ง ให้กลับไปตรวจการแปลงมุมเลี้ยวข้างบน

**รถไม่ขยับเลยทั้งที่ Nav2 บอกว่ากำลังวิ่ง** — เช็ค `ros2 topic info -v /cmd_vel_nav`
ต้องเป็น `TwistStamped` ทั้ง publisher และ subscriber ถ้ามีสองชนิดปนกันคือไม่มีข้อความส่งถึงกัน

**goal fail ทันทีใน ~13 ms** — TF ขาด ลอง `ros2 run tf2_ros tf2_echo map base_link`
ถ้าขึ้น `not part of the same tree` แปลว่า Cartographer หรือ AMCL ตาย มักเกิดจากกด Stop/Play ใน Isaac Sim

**RViz ไม่เห็นแมพ** — `/map` publish แบบ latched (transient local) ถ้า RViz ตั้ง Durability
เป็น Volatile จะไม่ได้รับ ไฟล์ `.rviz` ในนี้ตั้งไว้ถูกแล้ว
