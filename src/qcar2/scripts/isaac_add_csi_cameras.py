"""Enable the QCar2's three deactivated CSI cameras - the 360 deg ring for YOLO.

Run this INSIDE Isaac Sim (Window > Script Editor, paste and Run) with the
simulation STOPPED, or headless:

    ~/isaacsim/_build/linux-x86_64/release/python.sh \
        scripts/isaac_apply_and_save.py <stage.usd> scripts/isaac_add_csi_cameras.py

Back the stage up first:

    cp qcar2_workspace.usd qcar2_workspace.usd.bak-precsi

What this actually does - and what it does NOT do
--------------------------------------------------
The asset ships FOUR complete, correctly wired CSI camera graphs. Three of them
(`ros_qcar2_csi_back`, `_left`, `_right`) are simply switched off with USD's
`active = False`, so they compose away to nothing: `ros2 topic list` shows no
topic, and even a stage traversal looking for OmniGraph prims does not list them,
because an inactive prim's children are never composed at all. That makes them
look absent rather than disabled, and the obvious response - building three new
graphs from scratch - would be layering duplicates on top of prims that already
exist in the referenced `qcar2.usd`.

So this script does not build anything. It activates what is already there and
patches three attributes on each of the four graphs. Everything it touches is a
plain USD attribute, which is why it needs no omni.graph runtime and can be
verified against a copy of the stage with nothing but pxr.

The shipped graphs each hold exactly:

    on_playback_tick             omni.graph.action.OnPlaybackTick
    ros2_context                 isaacsim.ros2.bridge.ROS2Context
    isaac_create_render_product  820x410, cameraPrim -> that CSI camera
    ros2_camera_helper           topicName = "csi_<pos>" (bare, no camera_info)

The three patches:

1. topicName  "csi_back" -> "csi_back/image_raw". The bare name is unconventional
   and leaves nowhere to hang a camera_info later. The four cameras are only
   useful together, so they are renamed together, csi_front included.
2. frameSkipCount = 5, i.e. ~10 Hz at 60 fps. The shipped graphs set it nowhere
   at all, so they publish on EVERY tick (~20 Hz) - csi_front was the most
   expensive publisher in the scene for that reason.
3. inputs:enabled = False on the render product, created if missing, so
   isaac_camera_streams.py can switch these on and off by preset. That script
   raises if the attribute is absent, and the three inactive graphs never had it.

No camera_info publisher is added. It would mean grafting a node into a graph
that comes from a referenced layer, which is real surgery, and nothing in this
pipeline needs it: YOLO is 2D, and an RViz *Image* display (unlike a *Camera*
display) does not read camera_info. Add one when something needs to project a
detection into 3D.

Idempotent - re-running it re-applies the same values.

To reverse: set `active = False` on the three graphs again, or restore the
backup. Note that `active` is authored into the stage's ROOT layer as an
override; the referenced qcar2.usd is never modified.

After running, press PLAY and confirm:

    ros2 topic hz /csi_front/image_raw    # ~10 Hz
    ros2 topic hz /csi_back/image_raw
    ros2 topic hz /csi_left/image_raw
    ros2 topic hz /csi_right/image_raw

If a topic is silent, its render product is off - that is the preset's job:

    isaac_camera_streams.py  with QCAR2_CAMERA_PRESET=csi

GPU BUDGET
----------
A render product is rendered by RTX every frame whether or not anything
subscribes, and frameSkipCount throttles PUBLISHING only, never rendering. Four
CSI render products cost four full renders per frame on top of everything else;
four cameras measured here starved the V-SLAM RGB-D pair to 1.6 Hz. The CSI ring
and the RGB-D pair are meant to take turns - see isaac_camera_streams.py.
"""

import omni.usd
from pxr import Sdf

# Publish every (FRAME_SKIP + 1)-th rendered frame -> ~10 Hz at 60 fps.
FRAME_SKIP = 5

CSI_POSITIONS = ("front", "back", "left", "right")


def find_qcar2_root(stage):
    """Locate the robot prim.

    Standalone asset: /qcar2. Navigation scene: /World/odom/qcar2. Match on
    structure, not on a hard-coded path - same rule as the other scripts here.

    Traverse() skips inactive prims, but base_link and the robot root are always
    active, so this is unaffected by the very deactivation this script fixes.
    """
    for prim in stage.Traverse():
        if prim.GetName() == "qcar2" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError("No prim named 'qcar2' with a 'base_link' child on this stage")


def force_auto_exposure(stage, camera_path):
    """Keep auto exposure on, explicitly.

    Isaac materialises per-camera RTX post-process overrides onto any camera a
    render product is attached to, and the default it writes is
    `omni:rtx:autoExposure:enabled = False`. Three of these cameras have never
    had a live render product, so enabling them is exactly the moment Isaac
    would write that override in - and a camera with auto exposure off renders
    this brightly lit warehouse blown out to pure white.

    That failure is quiet in a way that wastes a lot of time: the topic publishes
    at the right rate and every frame is valid and correctly shaped, it is just
    white. YOLO then reports zero detections on a pipeline that looks perfectly
    healthy, and the natural next move is to lower the confidence threshold.
    Look at the image first.
    """
    cam = stage.GetPrimAtPath(camera_path)
    if not cam.IsValid():
        raise RuntimeError(f"CSI camera not found: {camera_path}")
    cam.CreateAttribute("omni:rtx:autoExposure:enabled",
                        Sdf.ValueTypeNames.Bool, True).Set(True)


def enable_graph(stage, graph_path, position):
    """Activate one CSI graph and normalise its publisher settings."""
    graph = stage.GetPrimAtPath(graph_path)
    if not graph.IsValid():
        raise RuntimeError(
            f"{graph_path} does not exist. This asset is expected to ship all "
            "four CSI graphs; only their `active` flag differs.")

    was_active = graph.IsActive()
    # Authored as an override in the stage's root layer - qcar2.usd untouched.
    graph.SetActive(True)

    helper = stage.GetPrimAtPath(f"{graph_path}/ros2_camera_helper")
    rp = stage.GetPrimAtPath(f"{graph_path}/isaac_create_render_product")
    if not helper.IsValid() or not rp.IsValid():
        raise RuntimeError(
            f"{graph_path} is missing its camera helper or render product - "
            "this is not the graph layout this script expects")

    helper.CreateAttribute("inputs:topicName", Sdf.ValueTypeNames.String,
                           True).Set(f"csi_{position}/image_raw")
    helper.CreateAttribute("inputs:frameId", Sdf.ValueTypeNames.String,
                           True).Set(f"csi_{position}")
    helper.CreateAttribute("inputs:frameSkipCount", Sdf.ValueTypeNames.UInt,
                           True).Set(FRAME_SKIP)

    # Created if absent: the three inactive graphs never carried it, and
    # isaac_camera_streams.py raises on a render product without it.
    rp.CreateAttribute("inputs:enabled", Sdf.ValueTypeNames.Bool, True).Set(False)

    return was_active


def main():
    stage = omni.usd.get_context().get_stage()
    root = str(find_qcar2_root(stage).GetPath())

    for position in CSI_POSITIONS:
        camera_path = (f"{root}/base_link/csi_{position}/"
                       f"CSI_{position}")
        graph_path = f"{root}/ros2graphs/ros_qcar2_csi_{position}"

        force_auto_exposure(stage, camera_path)
        was_active = enable_graph(stage, graph_path, position)
        state = "already active" if was_active else "ACTIVATED"
        print(f"[qcar2] csi_{position:<6} {state:<14} -> /csi_{position}/image_raw")

    print(f"[qcar2] 4 CSI graphs active, frameSkipCount={FRAME_SKIP} "
          "(~10 Hz at 60 fps), 820x410")
    print("[qcar2] render products are OFF - switch them on with")
    print("[qcar2]   isaac_camera_streams.py, preset 'csi'")


main()
