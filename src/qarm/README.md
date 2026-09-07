# qarm

แพ็กเกจเดียวจบสำหรับ **Quanser QArm** บน ROS 2 Jazzy — interfaces, โหนด (แขนจริง + แขนจำลอง),
URDF/meshes และหน้าจอ RViz

ต้นทางคือตัวอย่างของ Quanser ใน `ROS 2 QArm Guide.pdf` แต่แก้ให้ทำงานบน Jazzy
(คู่มือเขียนสำหรับ Kilted) และเพิ่มโหมดจำลองเพื่อให้ทดสอบได้โดยไม่ต้องมีแขนจริง

## เริ่มใช้งาน

```bash
colcon build --packages-select qarm
source install/setup.bash
```

```bash
# เห็นแขนใน RViz (แขนจำลอง + action server + RViz)
ros2 launch qarm view_qarm.py

# สั่ง goal จากอีกเทอร์มินัล
ros2 action send_goal /move_qarm qarm/action/MoveQArm "{task_space_pose: [0.25,-0.40,0.25,0.0]}"

# ไม่ต้องการ RViz แค่รัน pipeline
ros2 launch qarm move_qarm.py use_sim:=true goal_pose:="[0.5,0.0,0.5,0.0]"

# แขนจริง (ตัด use_sim ออก / ใส่ false)
ros2 launch qarm move_qarm.py
ros2 launch qarm view_qarm.py use_sim:=false
```

`goal` คือ `[x, y, z, yaw]` ของปลายเครื่องมือ หน่วยเมตร/เรเดียน

## ต้องติดตั้งก่อน

1. **Quanser SDK** (ให้ `import quanser.hardware` ได้)
   ```bash
   sudo apt install -y --allow-downgrades quanser-sdk librealsense2=2.49.0 \
       librealsense2-dev- librealsense2-gl- librealsense2-utils-
   ```
   `libquanser-media1` ล็อก `librealsense2 = 2.49.0` แบบเป๊ะ ถ้าเครื่องมี 2.58.x จาก repo ของ Intel
   จะขึ้น "held broken packages" ต้องถอยเวอร์ชันและถอน `-dev/-gl/-utils` ใน transaction เดียวกัน
   (เครื่องหมาย `-` ท้ายชื่อ) apt แก้แยกทีละคำสั่งไม่ได้

2. **Quanser Academic Resources** (`pal` / `hal`) ตั้งใน `~/.bashrc`
   ```bash
   export QAL_DIR=$HOME/Documents/Quanser/Quanser_Academic_Resources-dev-windows
   export PYTHONPATH=$PYTHONPATH:$QAL_DIR/0_libraries/python
   ```

`move_qarm_server` ใช้แค่คณิตศาสตร์ IK/FK (`hal.products.qarm.QArmUtilities`) ซึ่งเป็น NumPy ล้วน
มันเปิด HIL ต่อเมื่อสร้าง `QArm()` เท่านั้น จึงทดสอบได้เต็มรูปแบบโดยไม่ต้องมีแขน

## โหนด

| โหนด | หน้าที่ |
|---|---|
| `qarm_hardware` | ไดรเวอร์จริง คุย HIL ผ่าน `pal.products.qarm.QArm` (default `hardware=1` = แขนจริง) |
| `qarm_sim` | แขนจำลอง หัวข้อเหมือน `qarm_hardware` เป๊ะ ไม่พึ่ง SDK ข้อต่อวิ่งเข้าหาคำสั่งด้วยความเร็วจำกัด |
| `move_qarm_server` | action `MoveQArm` รับ `[x,y,z,yaw]` → IK → ส่งมุมข้อต่อให้โหนดแขน |
| `move_qarm_client` | ตัวอย่าง client อ่านเป้าจากพารามิเตอร์ `goal_pose` |
| `rgbd` | กล้อง RealSense บนแขน → `qarm_camera/color`, `qarm_camera/depth` |

หัวข้อทั้งหมดเป็นชื่อสัมบูรณ์ `/qarm/...` ทุกโหนด เพื่อให้สลับ `qarm_sim` ↔ `qarm_hardware` ได้จริง

| หัวข้อ | ชนิด |
|---|---|
| `/qarm/joint_states` | `sensor_msgs/JointState` (4 ข้อต่อ + gripper) |
| `/qarm/diagnostics` | `qarm/msg/QArmDiagnostics` (กระแส, PWM, อุณหภูมิ) |
| `/qarm/joint_position_cmd` | `std_msgs/Float64MultiArray` (4 มุม) |
| `/qarm/gripper_cmd` | `std_msgs/Float64` |
| `/qarm/led_cmd` | `std_msgs/Float64MultiArray` (RGB) |

## พารามิเตอร์

| โหนด | พารามิเตอร์ | ค่าเริ่มต้น | ความหมาย |
|---|---|---|---|
| `move_qarm_server` | `position_tolerance` | 0.04 | ผลรวม error ตำแหน่ง+มุมที่ถือว่าถึงเป้า |
| | `goal_timeout` | 20.0 | วินาทีสูงสุดต่อ 1 goal |
| | `joint_state_timeout` | 5.0 | รอ joint state แรกนานสุด |
| `move_qarm_client` | `goal_pose` | `[0,0,0.5,0]` | `[x, y, z, yaw]` |
| | `goal_attempts` | 5 | ส่ง goal ซ้ำได้กี่ครั้ง |
| `qarm_sim` | `max_joint_speed` | 1.0 | rad/s |

## RMW ต้องตรงกันทุกเทอร์มินัล

`~/.bashrc` ตั้ง `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ฉะนั้นเทอร์มินัลปกติเป็น **CycloneDDS**
เชลล์ไหนไม่ได้ source `~/.bashrc` (สคริปต์, cron, เครื่องมืออัตโนมัติ) จะกลับไปใช้ FastDDS แล้วอาการหลอกมาก:

- `ros2 node list` / `ros2 action list` **ยังเห็นกันและกัน** เพราะ RTPS คุยกันได้บางส่วน
- แต่ type hash ไม่ตรง → goal ถูกถอดรหัสฝั่งรับเป็น `[0, 0, 0, 0]`
- response ส่งกลับไม่ถึง client ค้างเงียบ ไม่มี error สักบรรทัด

เช็คก่อนงง: `echo $RMW_IMPLEMENTATION` ต้องได้ `rmw_cyclonedds_cpp` เหมือนกันทุกหน้าต่าง

## ต่างจากคู่มือ Quanser ตรงไหน

คู่มือเขียนสำหรับ **Kilted** เวิร์กสเปซนี้เป็น **Jazzy** สองข้อนี้ทำให้โค้ดต้นฉบับรันไม่ได้เลย:

| | Kilted (ตามคู่มือ) | Jazzy (ที่ใช้จริง) |
|---|---|---|
| เริ่ม node | `with rclpy.init(args=args):` | `rclpy.init()` คืน `None` → ใช้ `init / spin / destroy_node / try_shutdown` |
| ส่งภาพ | `image_transport_py` | ไม่มีใน Jazzy → `rgbd` publish `sensor_msgs/Image` ตรง ๆ (เท่ากับ transport `raw`) |

`rgbd` **ไม่ใช้ `cv_bridge`** เพราะบนเครื่องนี้มัน segfault (Jazzy build กับ NumPy 1.x แต่ Isaac Sim
ดัน NumPy 2.x ขึ้นหน้า PYTHONPATH) จึงแปลงด้วย NumPy ตรง ๆ เหมือน `yolo_detector.py` ของ qcar2 —
อย่า "ทำให้ง่ายขึ้น" กลับไปใช้ cv_bridge

## จุดที่พังเงียบ ๆ ถ้าไปแก้

- **IK ของ Quanser คืน `[0,0,0,0]` เป็น sentinel เมื่อเอื้อมไม่ถึง** แต่ค่านี้ก็เป็นคำตอบที่ถูกของท่า home
  (`[0.45, 0, 0.49]`) ด้วย โค้ดที่เช็คแค่ "ศูนย์ทั้งสี่ = เกิน limit" จะปฏิเสธท่า home ทั้งที่เอื้อมถึง
  `move_qarm_server.reachable()` จึงยืนยันซ้ำด้วย FK
- **ลูปรอผลต้องเช็ค `is_cancel_requested` และมี `goal_timeout`** ไม่งั้น goal ที่เข้าเป้าไม่ได้จะค้างตลอดกาล
  และ cancel จะถูกตอบรับแต่แขนไม่หยุด
- **client ต้องยิง goal ซ้ำได้** ตอน launch ทุกโหนดขึ้นพร้อมกัน server อาจตอบรับ goal ก่อนจับคู่ endpoint
  ของ client เสร็จ → response หาย (`failed to send response (timeout)`) client ค้างเงียบ
  ตอนนี้รอด้วย `spin_until_future_complete` แล้วส่งใหม่ (`goal_attempts`)
  หมายเหตุ: `ros2 action send_goal` ไม่มีกลไกนี้ ถ้ายิงทันทีที่ stack เพิ่งขึ้นอาจไม่ได้ result กลับ รอสัก 10 วิ
- **server รอ joint state แรกแบบมีขอบเขต** (`joint_state_timeout`) แทนที่จะ abort ทันที ด้วยเหตุผลเดียวกัน
- **โมดูล Python ของโหนดชื่อ `qarm_nodes` ไม่ใช่ `qarm`** เพราะ `rosidl_generate_interfaces` สร้างโมดูล
  Python ชื่อ `qarm` ไปแล้ว (คือตัวที่ `from qarm.action import MoveQArm` เรียกใช้) ถ้าตั้งชื่อซ้ำจะแย่ง
  `__init__.py` กันเอง

## URDF

มาจาก [quanser/urdf_representations](https://github.com/quanser/urdf_representations) (BSD-3-Clause, ดู `LICENSE`)
แก้ไป 3 จุด บันทึกไว้ในหัวไฟล์ `urdf/qarm.urdf` แล้ว:

1. **ชื่อข้อต่อ** `YAW/SHOULDER/ELBOW/WRIST` → `base_joint/shoulder_joint/arm_joint/wrist_joint`
   ให้ตรงกับที่โหนด publish — **ถ้าไม่ตรง `robot_state_publisher` จะไม่บ่นเลยแต่หุ่นนิ่งสนิท**
2. **พาธ mesh** `package://QARM/` → `package://qarm/`
3. **เพิ่มเฟรม `tool_frame`** ห่างจากหน้าแปลน `END-EFFECTOR` ไป 0.162 ม. ตามแกน +z

ค่ามุมข้อต่อคือ phi ชุดเดียวกับที่แขนรายงาน ไม่ต้องแปลงหน่วยหรือกลับเครื่องหมาย
(ลิมิต `arm_joint` ใน URDF `[-1.658, 1.309]` rad = `[-95°, +75°]` ตรงกับ `_check_joint_limits` เป๊ะ)

### `tool_frame` คือจุดที่ควบคุมจริง

| เฟรม | ตรงกับอะไร |
|---|---|
| `base_link` → `END-EFFECTOR` | หน้าแปลนข้อมือ — สั้นกว่า goal 0.162 ม. เสมอ |
| `base_link` → `tool_frame` | **จุดที่ `MoveQArm` ควบคุม** ตรงกับ FK ที่ ~0.2 มม. |

ส่วนต่าง 0.2 มม. มาจาก URDF ใช้ `L_1 = 0.1397714` ขณะที่ `QArmUtilities` ใช้ `0.1400` — เป็นของ Quanser เอง

### ชื่อเฟรมชนกับ qcar2

URDF ใช้ `base_link` เป็นฐานแขน **ชื่อเดียวกับ `base_link` ของ QCar2** ที่ Isaac Sim publish
(`odom -> base_link`) ถ้าเปิด stack รถพร้อม RViz แขนใน ROS domain เดียวกัน `base_link` จะมีพ่อสองตัว
→ TF พังทั้งคู่ ถ้าต้องรันพร้อมกันให้ใส่ `frame_prefix:=qarm_` ที่ `robot_state_publisher`
แล้วเปลี่ยน Fixed Frame ใน RViz เป็น `qarm_world`

meshes รวม 18 MB (`base_link.STL` อย่างเดียว 11 MB) เป็น STL ดิบจาก Quanser
