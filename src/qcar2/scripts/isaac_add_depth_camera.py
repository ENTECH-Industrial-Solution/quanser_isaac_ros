"""Add a depth camera + ROS 2 publisher graph to the QCar2 stage in Isaac Sim.

Run this INSIDE Isaac Sim (Window > Script Editor, paste and Run), with the
simulation STOPPED. It is idempotent - running it twice rebuilds the graph.

Why this script exists at all: the shipped QCar2 asset has
`/qcar2/base_link/realsenseDepth` as a bare Xform. It is already a target of the
ROS2PublishTransformTree node, so the frame `realsenseDepth` shows up in TF, but
there is no Camera prim under it and no OmniGraph publishing anything. The RGB
side (`realsenseRGB/Realsense_RGB` + `ros2graphs/ros_qcar2_realsense_rgb`) is
fully wired, so this script mirrors that graph for depth.

After running, press PLAY and confirm:

    ros2 topic hz /realsense_depth
    ros2 topic echo /realsense_depth_camera_info --once

Then bring up navigation normally - qcar2_navigation_launch.py converts
/realsense_depth into /scan_depth and feeds it to the local costmap.
"""

import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdGeom

# The published topics. Kept in sync with qcar2_navigation_launch.py.
DEPTH_TOPIC = "realsense_depth"
CAMERA_INFO_TOPIC = "realsense_depth_camera_info"

# The TF frame Isaac already publishes for this prim (= the prim's name).
# It is an OPTICAL frame: +z forward, +x right, +y down.
FRAME_ID = "realsenseDepth"

WIDTH, HEIGHT = 640, 480

# Publish every (FRAME_SKIP + 1)-th rendered frame. A 640x480 depth render on
# every tick is the single most expensive thing you can add to this scene, and
# /scan from the lidar only runs at ~4 Hz anyway, so there is nothing to gain
# from a faster depth stream.
FRAME_SKIP = 5


def find_qcar2_root(stage):
    """Locate the robot prim.

    The standalone asset puts it at /qcar2, but the navigation scene references
    it under /World/odom/qcar2. Match on structure, not on a hard-coded path.
    """
    for prim in stage.Traverse():
        if prim.GetName() == "qcar2" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError("No prim named 'qcar2' with a 'base_link' child on this stage")


def clone_camera(stage, src_path, dst_path):
    """Create the depth Camera as a copy of the RealSense RGB camera.

    Same intrinsics on purpose: on real hardware both streams come off one D435,
    and depthimage_to_laserscan derives its angles from the published camera_info,
    so a made-up focal length would silently skew every scan angle.
    """
    src = stage.GetPrimAtPath(src_path)
    if not src.IsValid():
        raise RuntimeError(f"RGB reference camera not found at {src_path}")

    if stage.GetPrimAtPath(dst_path).IsValid():
        stage.RemovePrim(dst_path)

    cam = UsdGeom.Camera.Define(stage, dst_path)
    dst = cam.GetPrim()
    for attr in src.GetAttributes():
        name = attr.GetName()
        # omni:kit:* is viewport bookkeeping (camera lock, center of interest).
        if name.startswith("omni:kit:"):
            continue
        value = attr.Get()
        if value is None:
            continue
        dst.CreateAttribute(name, attr.GetTypeName(), attr.IsCustom()).Set(value)
    return dst


def build_graph(stage, graph_path, camera_path):
    if stage.GetPrimAtPath(graph_path).IsValid():
        stage.RemovePrim(graph_path)

    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": graph_path,
            "evaluator_name": "execution",
            "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_SIMULATION,
        },
        {
            keys.CREATE_NODES: [
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("ros2_context", "isaacsim.ros2.bridge.ROS2Context"),
                ("isaac_create_render_product",
                 "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ros2_camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ros2_camera_info_helper",
                 "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick",
                 "isaac_create_render_product.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut",
                 "ros2_camera_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut",
                 "ros2_camera_info_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath",
                 "ros2_camera_helper.inputs:renderProductPath"),
                ("isaac_create_render_product.outputs:renderProductPath",
                 "ros2_camera_info_helper.inputs:renderProductPath"),
                ("ros2_context.outputs:context",
                 "ros2_camera_helper.inputs:context"),
                ("ros2_context.outputs:context",
                 "ros2_camera_info_helper.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("isaac_create_render_product.inputs:width", WIDTH),
                ("isaac_create_render_product.inputs:height", HEIGHT),
                ("ros2_camera_helper.inputs:type", "depth"),
                ("ros2_camera_helper.inputs:topicName", DEPTH_TOPIC),
                ("ros2_camera_helper.inputs:frameId", FRAME_ID),
                ("ros2_camera_helper.inputs:frameSkipCount", FRAME_SKIP),
                ("ros2_camera_info_helper.inputs:topicName", CAMERA_INFO_TOPIC),
                ("ros2_camera_info_helper.inputs:frameId", FRAME_ID),
                ("ros2_camera_info_helper.inputs:frameSkipCount", FRAME_SKIP),
            ],
        },
    )

    # cameraPrim is a USD relationship, not a plain attribute - the same shape
    # the existing graphs use for chassisPrim / targetPrims.
    rp = stage.GetPrimAtPath(f"{graph_path}/isaac_create_render_product")
    rel = rp.GetRelationship("inputs:cameraPrim")
    if not rel:
        rel = rp.CreateRelationship("inputs:cameraPrim")
    rel.SetTargets([Sdf.Path(camera_path)])


def main():
    stage = omni.usd.get_context().get_stage()
    root = str(find_qcar2_root(stage).GetPath())

    rgb_camera = f"{root}/base_link/realsenseRGB/Realsense_RGB"
    depth_camera = f"{root}/base_link/realsenseDepth/Realsense_Depth"
    graph_path = f"{root}/ros2graphs/ros_qcar2_realsense_depth"

    clone_camera(stage, rgb_camera, depth_camera)
    build_graph(stage, graph_path, depth_camera)

    print(f"[qcar2] depth camera : {depth_camera}")
    print(f"[qcar2] graph        : {graph_path}")
    print(f"[qcar2] topics       : /{DEPTH_TOPIC}, /{CAMERA_INFO_TOPIC}")
    print("[qcar2] press PLAY, then: ros2 topic hz /" + DEPTH_TOPIC)


main()
