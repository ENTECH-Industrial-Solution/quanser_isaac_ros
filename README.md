# quanser_isaac_ros

ROS 2 Jazzy workspace สำหรับขับหุ่นยนต์ Quanser อัตโนมัติใน NVIDIA Isaac Sim

Isaac Sim เป็นคนจำลองหุ่นยนต์และส่ง `/clock`, `/scan`, `/imu`, `/odom` และ TF
`odom -> base_link` ออกมา ฝั่ง ROS รับไปทำ SLAM / นำทาง / ขับตามเลน แล้วส่งคำสั่ง
ขับกลับเข้า drive graph ของ Isaac

> **`angular.z` ที่ส่งเข้า Isaac คือ *มุมเลี้ยวล้อหน้า* หน่วยเรเดียน ไม่ใช่ yaw rate**
> QCar2 เป็นรถ Ackermann `src/qcar2_isaac_nav2/src/twist_stamped_to_twist.py` เป็นตัว
> แปลงหน่วยด้วยโมเดลจักรยาน ไม่ใช่แค่เปลี่ยนชนิดข้อความ ถอดออกแล้วรถจะวิ่งวนเป็นวงกลม
> ทั้งที่ log ของ Nav2 ดูปกติทุกบรรทัด

## แพ็กเกจ

| แพ็กเกจ | สำหรับ |
|---|---|
| [`qcar2_isaac_nav2`](src/qcar2_isaac_nav2/) | **ตัวหลัก** — QCar2 (รถ Ackermann) ครบทุกเส้นทาง |
| `qbot_platform_issac_nav2` | QBot Platform (ขับเคลื่อนแบบต่างล้อ) SLAM + Nav2 |

## เลือกเส้นทางที่จะใช้

ทั้งสี่เส้นทางอยู่ในแพ็กเกจ `qcar2_isaac_nav2` และรันแยกกัน

| เส้นทาง | เซนเซอร์ | ทำอะไร |
|---|---|---|
| **นำทางด้วย lidar** | `/scan` | Cartographer เก็บแมพ → AMCL + Nav2 นำทางไป goal pose |
| **นำทางด้วยกล้อง** | RGB-D | RTAB-Map ทำ V-SLAM แทน lidar ทั้งหมด → Nav2 |
| **ขับตามเลน + หลบสิ่งกีดขวาง** | กล้อง CSI + lidar + depth | ไม่ใช้ Nav2 เลย ไม่มีแมพ ไม่มี goal — กล้องหาเส้นเลน แล้วหลบของที่ขวางทาง |
| **ตรวจจับวัตถุ** | กล้อง CSI 4 ตัว 360° | YOLO ออก detections + ภาพ mosaic เฉย ๆ ไม่แตะการขับ |

**เริ่มเร็วสุด — ขับตามเลนพร้อมหลบสิ่งกีดขวาง:**

```bash
ros2 launch qcar2_isaac_nav2 qcar2_lane_follow_launch.py
```

## Build

```bash
cd ~/entech_quanser_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select qcar2_isaac_nav2
source install/setup.bash
```

`config/`, `launch/`, `rviz/`, `behavior_trees/`, `maps/` ถูกติดตั้งผ่าน
`install(DIRECTORY ...)` — **แก้ไฟล์ YAML / launch / rviz / map แล้วต้อง build ใหม่ทุกครั้ง**
ไม่งั้นจะเหมือนแก้แล้วไม่มีอะไรเกิดขึ้น

ไม่มี unit test — ความถูกต้องของ workspace นี้คือ "รถไปถึงเป้าใน Isaac Sim ได้จริงไหม"
`colcon test` รันแค่ linter

## รายละเอียด

ทุกอย่างที่เหลือ — วิธีรันทีละขั้น, ค่าพารามิเตอร์แต่ละตัวมาจากการวัดอะไร,
กับดักที่ทำให้เสียเวลามาแล้ว — อยู่ใน **[`src/qcar2_isaac_nav2/README.md`](src/qcar2_isaac_nav2/README.md)**
ซึ่งมีสารบัญให้กดข้ามไปหัวข้อที่ต้องการได้

อ่านก่อนแก้ค่าใน `config/` — ค่าส่วนใหญ่ในนั้นมาจากการวัดจริง ไม่ใช่ค่า default
