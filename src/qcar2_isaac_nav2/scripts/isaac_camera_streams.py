"""Switch the QCar2's camera render products on and off, one preset at a time.

Run inside Isaac Sim (Script Editor) with the sim STOPPED, or headless:

    QCAR2_CAMERA_PRESET=csi \
    ~/isaacsim/_build/linux-x86_64/release/python.sh \
        scripts/isaac_apply_and_save.py <stage.usd> scripts/isaac_camera_streams.py

Pick the preset by editing PRESET below (Script Editor) or by setting
QCAR2_CAMERA_PRESET (headless).

Every ROS2 camera graph on this asset owns an IsaacCreateRenderProduct, and a
render product is rendered by RTX on every frame whether or not anything
subscribes to the topic it feeds. `frameSkipCount` does NOT help - it throttles
publishing, not rendering. The render product has to be switched off, which is
what this does, and it is the only thing here that actually reclaims GPU.

Measured on an RTX 5060 laptop sharing the GPU with RViz: with four 640x480
cameras live the RGB-D pair that RTAB-Map consumes was starved down to 1.6 Hz.
The full set on this asset is now eight render products (four CSI, three legacy
RealSense streams, one RGB-D pair), so running everything at once is not a real
option - hence presets rather than a single on/off.

Non-camera graphs (lidar, imu, tf, drive controller, clock) are never touched:
they are cheap and the stack depends on them.

Reversible: run it again with a different preset.
"""

import os

import omni.usd

# Which graphs to leave rendering. Everything else with a render product is
# switched off.
PRESETS = {
    # RGB-D V-SLAM only - qcar2_vslam_*_launch.py. One render product.
    "vslam": {
        "ros_qcar2_realsense_rgbd",
    },
    # The 360 deg CSI ring only - qcar2_yolo_launch.py. Four render products,
    # which is already a heavy scene on a laptop GPU.
    "csi": {
        "ros_qcar2_csi_front",
        "ros_qcar2_csi_back",
        "ros_qcar2_csi_left",
        "ros_qcar2_csi_right",
    },
    # Just the front CSI camera. The cheap way to develop the YOLO node while
    # V-SLAM keeps running - two render products instead of five.
    "csi_front": {
        "ros_qcar2_csi_front",
    },
    # Everything the two routes need at once: five render products. Expect the
    # RGB-D pair to drop well below the 10 Hz RTAB-Map wants, which degrades
    # mapping quality rather than failing outright - so if a map comes out
    # sparse after running like this, this is why.
    "both": {
        "ros_qcar2_realsense_rgbd",
        "ros_qcar2_csi_front",
        "ros_qcar2_csi_back",
        "ros_qcar2_csi_left",
        "ros_qcar2_csi_right",
    },
    # No cameras at all - lidar/AMCL navigation, or measuring what the cameras
    # actually cost by taking them away.
    "none": set(),
}

PRESET = os.environ.get("QCAR2_CAMERA_PRESET", "csi")


def main():
    if PRESET not in PRESETS:
        raise RuntimeError(
            f"Unknown preset {PRESET!r}. Choose one of: "
            f"{', '.join(sorted(PRESETS))}")
    keep = PRESETS[PRESET]

    stage = omni.usd.get_context().get_stage()

    found = []
    for prim in stage.Traverse():
        if prim.GetTypeName() != "OmniGraphNode":
            continue
        path = str(prim.GetPath())
        if "isaac_create_render_product" not in path:
            continue
        graph = path.split("/")[-2]
        enable = graph in keep
        attr = prim.GetAttribute("inputs:enabled")
        if not attr:
            raise RuntimeError(
                f"{path} has no inputs:enabled - this Isaac Sim build cannot "
                "switch render products off from USD")
        attr.Set(enable)
        found.append((graph, enable))

    if not found:
        raise RuntimeError("No IsaacCreateRenderProduct nodes on this stage")

    # A preset naming a graph that is not on the stage is almost always a
    # missing setup step rather than a typo - say which one, and what builds it.
    missing = keep - {graph for graph, _ in found}
    if missing:
        print(f"[qcar2] WARNING preset {PRESET!r} names graphs that do not "
              f"exist: {', '.join(sorted(missing))}")
        print("[qcar2]   CSI graphs  -> run scripts/isaac_add_csi_cameras.py")
        print("[qcar2]   RGB-D graph -> run scripts/isaac_add_rgbd_camera.py")

    for graph, enable in sorted(found):
        print(f"[qcar2] {'ON ' if enable else 'off'}  {graph}")
    print(f"[qcar2] preset {PRESET!r}: "
          f"{sum(1 for _, e in found if e)}/{len(found)} camera render products "
          "enabled")


main()
