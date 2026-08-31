"""YOLO object detection on the QCar2's 360 deg CSI camera ring.

    # 1. Once, in Isaac Sim with the sim STOPPED - build the four CSI graphs:
    #      scripts/isaac_add_csi_cameras.py
    #    then switch their render products on:
    #      scripts/isaac_camera_streams.py  with QCAR2_CAMERA_PRESET=csi
    #
    # 2. Press PLAY, then:
    ros2 launch qcar2_isaac_nav2 qcar2_yolo_launch.py

    # 3. Watch it work:
    ros2 topic echo /csi_front/detections
    ros2 topic hz /csi_front/annotated

This is a pure sensing add-on. It publishes detections and annotated images and
nothing else - no TF, no costmap layer, no cmd_vel - so it cannot disturb a
running Nav2 or V-SLAM stack, and it can be started and stopped underneath one.

GPU budget is the real constraint, not correctness. Four CSI render products
plus the RGB-D pair is five renders per frame on a laptop GPU that is also
running RViz, and render products cost their full price whether or not anything
subscribes (see isaac_camera_streams.py). The presets exist so the CSI ring and
the V-SLAM RGB-D pair can take turns:

    QCAR2_CAMERA_PRESET=csi     YOLO on all four cameras          (this file)
    QCAR2_CAMERA_PRESET=vslam   RGB-D mapping / localization      (V-SLAM route)
    QCAR2_CAMERA_PRESET=both    both at once, everything degraded

To run YOLO alongside V-SLAM on this hardware, use preset `csi_front` and
`cameras:=front` - two render products instead of five.

The four cameras are 97.6 deg horizontal FOV each, mounted on `base_link` at
z=0.109 m, facing front / back / left / right. 4 x 97.6 = 390 deg, so the ring
closes with a little overlap at the corners: an object at a corner is normally
seen by two cameras and gets two independent detections. Nothing here de-duplicates
them, because the two detections are in different camera frames - fusing them
means projecting into a common frame first, which needs depth the side cameras
do not have.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('qcar2_isaac_nav2')

    # A comma-separated string is the only shape that survives the command line
    # cleanly; the node wants a real list.
    cameras = [c.strip() for c in
               LaunchConfiguration('cameras').perform(context).split(',')
               if c.strip()]
    if not cameras:
        raise RuntimeError("cameras:= is empty - pass e.g. cameras:=front,back")

    valid = {'front', 'back', 'left', 'right'}
    unknown = set(cameras) - valid
    if unknown:
        raise RuntimeError(
            f"unknown camera(s) {sorted(unknown)} - choose from {sorted(valid)}")

    use_sim_time = LaunchConfiguration('use_sim_time')

    yolo_node = Node(
        package='qcar2_isaac_nav2',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'cameras': cameras,
            'model': LaunchConfiguration('model'),
            'device': LaunchConfiguration('device'),
            'confidence': LaunchConfiguration('confidence'),
            'iou': LaunchConfiguration('iou'),
            'imgsz': LaunchConfiguration('imgsz'),
            'half': LaunchConfiguration('half'),
            'publish_annotated': LaunchConfiguration('publish_annotated'),
            'max_detections': LaunchConfiguration('max_detections'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_yolo',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'qcar2_yolo.rviz')],
    )

    return [yolo_node, rviz_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cameras', default_value='front,back,left,right',
            description='Which CSI cameras to run YOLO on, comma separated. '
                        'Use cameras:=front with preset csi_front to run '
                        'alongside V-SLAM on one GPU.'),
        DeclareLaunchArgument(
            'model', default_value='yolo11n.pt',
            description='Ultralytics model. Downloaded to ~/.config/Ultralytics '
                        'on first use, so the first run needs a network. n=nano '
                        'is the right size here: four cameras share one GPU that '
                        'is also rendering the scene. yolo11s.pt is more accurate '
                        'and roughly halves the frame rate.'),
        DeclareLaunchArgument(
            'device', default_value='cuda:0',
            description='torch device. Falls back to cpu with a warning if CUDA '
                        'is unavailable.'),
        DeclareLaunchArgument(
            'confidence', default_value='0.35',
            description='Detection confidence threshold. The Isaac warehouse is '
                        'out of distribution for a COCO-trained model, so this is '
                        'lower than the 0.5 default - raise it if you get noise.'),
        DeclareLaunchArgument(
            'iou', default_value='0.45',
            description='NMS IoU threshold'),
        DeclareLaunchArgument(
            'imgsz', default_value='640',
            description='Inference size. Frames are 820x410 and get letterboxed '
                        'to this.'),
        DeclareLaunchArgument(
            'half', default_value='true',
            description='fp16 inference. Roughly 1.5x faster on this GPU; forced '
                        'off on cpu.'),
        DeclareLaunchArgument(
            'publish_annotated', default_value='true',
            description='Publish /csi_<pos>/annotated with boxes drawn on. Costs '
                        'a copy and an encode per frame - turn it off if you only '
                        'consume /detections.'),
        DeclareLaunchArgument(
            'max_detections', default_value='30',
            description='Cap on detections per frame'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the Isaac Sim /clock. Isaac Sim must be PLAYING.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Open RViz with the four annotated views in a 2x2 grid'),
        OpaqueFunction(function=launch_setup),
    ])
