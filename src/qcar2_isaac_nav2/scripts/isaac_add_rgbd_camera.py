"""Add a registered RGB-D camera + ROS 2 publisher graph to the QCar2 stage.

Run this INSIDE Isaac Sim (Window > Script Editor, paste and Run) with the
simulation STOPPED, or headless via scripts/isaac_save_rgbd_camera.py. It is
idempotent - running it twice rebuilds the camera and the graph.

Why this exists when isaac_add_depth_camera.py already publishes depth:

RGB-D SLAM (RTAB-Map) needs the depth image to be *registered* to the colour
image - same optical centre, same intrinsics, same timestamp - because it looks
up a depth value at the pixel where it found a visual feature. The QCar2 asset
models a real D435: `realsenseRGB` sits at base_link y=+0.0340 and
`realsenseDepth` at y=-0.0030, so the two are 37 mm apart. At 1 m that is an
~18 px disparity, and every feature would get the depth of whatever is 18 px to
its side. The existing /realsense_depth stream is fine for
depthimage_to_laserscan (which only reads a row of ranges) but not for this.

The fix is what RealSense calls "align depth to color": render depth from the
colour camera's viewpoint. Here that is one extra Camera prim parented under
`realsenseRGB`, so it inherits that Xform exactly, plus a single render product
feeding three publishers. Colour and depth then come off the *same* render
product, which also means they share a timestamp - no approximate sync needed.

After running, press PLAY and confirm:

    ros2 topic hz /realsense/color/image_raw     # ~10 Hz
    ros2 topic hz /realsense/depth/image_raw     # same rate, same stamps
    ros2 topic echo /realsense/camera_info --once

Leaves /realsense_depth and /realsense_depth_camera_info (from
isaac_add_depth_camera.py) untouched - the Nav2 local costmap still uses them.
"""

import omni.graph.core as og
import omni.usd
from pxr import Sdf, UsdGeom

COLOR_TOPIC = "realsense/color/image_raw"
DEPTH_TOPIC = "realsense/depth/image_raw"
CAMERA_INFO_TOPIC = "realsense/camera_info"

# One frame for both streams: they are rendered from the same camera, so the
# single camera_info below describes both. It is an OPTICAL frame
# (+z forward, +x right, +y down), which is what RTAB-Map expects for
# `frame_id` on an RGB-D pair.
FRAME_ID = "realsenseRGB"

WIDTH, HEIGHT = 640, 480

# Publish every (FRAME_SKIP + 1)-th rendered frame -> 10 Hz at 60 fps. Two
# 640x480 renders per published frame is already the most expensive thing in
# this scene; RTAB-Map is happy at 10 Hz as long as the car is driven slowly.
FRAME_SKIP = 5


def find_qcar2_root(stage):
    """Locate the robot prim.

    Standalone asset: /qcar2. Navigation scene: /World/odom/qcar2. Match on
    structure, not on a hard-coded path.
    """
    for prim in stage.Traverse():
        if prim.GetName() == "qcar2" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError("No prim named 'qcar2' with a 'base_link' child on this stage")


def clone_camera(stage, src_path, dst_path):
    """Create the RGB-D camera as a copy of the RealSense RGB camera.

    Parented under the same `realsenseRGB` Xform as the original, so it needs no
    transform of its own: it is the colour camera, rendered a second time.
    Copying the attributes keeps the intrinsics identical, which is what makes
    one camera_info valid for both streams.
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
        # omni:kit:* is viewport bookkeeping (camera lock, centre of interest).
        if name.startswith("omni:kit:"):
            continue
        value = attr.Get()
        if value is None:
            continue
        dst.CreateAttribute(name, attr.GetTypeName(), attr.IsCustom()).Set(value)

    # Keep auto exposure on, explicitly.
    #
    # Isaac materialises per-camera RTX post-process overrides onto any camera a
    # render product is attached to, and the default it writes is
    # `omni:rtx:autoExposure:enabled = False`. The shipped Realsense_RGB prim
    # carries no such override at all, so it follows the stage setting and is
    # correctly exposed. A fresh clone silently gets the override switched off
    # and renders this brightly lit warehouse blown out to pure white.
    #
    # That failure is not obvious from anything downstream: the topic publishes
    # at the right rate, the depth stream is unaffected, and the map still
    # builds - it just has no colour and no visual features, so every loop
    # closure is rejected with "Not enough inliers 0/15" and it reads like a
    # texture-poor scene rather than a broken camera. Check with
    # `ros2 topic echo /realsense/color/image_raw` or just look at it in RViz.
    dst.CreateAttribute("omni:rtx:autoExposure:enabled",
                        Sdf.ValueTypeNames.Bool, True).Set(True)
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
                ("ros2_color_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ros2_depth_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("ros2_camera_info_helper",
                 "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.CONNECT: [
                ("on_playback_tick.outputs:tick",
                 "isaac_create_render_product.inputs:execIn"),
                # All three publishers hang off the SAME render product. That is
                # what guarantees the colour and depth images published in a
                # given frame carry the same stamp and the same viewpoint.
                ("isaac_create_render_product.outputs:execOut",
                 "ros2_color_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut",
                 "ros2_depth_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:execOut",
                 "ros2_camera_info_helper.inputs:execIn"),
                ("isaac_create_render_product.outputs:renderProductPath",
                 "ros2_color_helper.inputs:renderProductPath"),
                ("isaac_create_render_product.outputs:renderProductPath",
                 "ros2_depth_helper.inputs:renderProductPath"),
                ("isaac_create_render_product.outputs:renderProductPath",
                 "ros2_camera_info_helper.inputs:renderProductPath"),
                ("ros2_context.outputs:context", "ros2_color_helper.inputs:context"),
                ("ros2_context.outputs:context", "ros2_depth_helper.inputs:context"),
                ("ros2_context.outputs:context",
                 "ros2_camera_info_helper.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("isaac_create_render_product.inputs:width", WIDTH),
                ("isaac_create_render_product.inputs:height", HEIGHT),
                ("ros2_color_helper.inputs:type", "rgb"),
                ("ros2_color_helper.inputs:topicName", COLOR_TOPIC),
                ("ros2_color_helper.inputs:frameId", FRAME_ID),
                ("ros2_color_helper.inputs:frameSkipCount", FRAME_SKIP),
                ("ros2_depth_helper.inputs:type", "depth"),
                ("ros2_depth_helper.inputs:topicName", DEPTH_TOPIC),
                ("ros2_depth_helper.inputs:frameId", FRAME_ID),
                ("ros2_depth_helper.inputs:frameSkipCount", FRAME_SKIP),
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
    rgbd_camera = f"{root}/base_link/realsenseRGB/Realsense_RGBD"
    graph_path = f"{root}/ros2graphs/ros_qcar2_realsense_rgbd"

    clone_camera(stage, rgb_camera, rgbd_camera)
    build_graph(stage, graph_path, rgbd_camera)

    print(f"[qcar2] rgbd camera  : {rgbd_camera}")
    print(f"[qcar2] graph        : {graph_path}")
    print(f"[qcar2] topics       : /{COLOR_TOPIC}, /{DEPTH_TOPIC}, /{CAMERA_INFO_TOPIC}")
    print(f"[qcar2] frame        : {FRAME_ID} (both streams)")
    print("[qcar2] press PLAY, then: ros2 topic hz /" + DEPTH_TOPIC)


main()
