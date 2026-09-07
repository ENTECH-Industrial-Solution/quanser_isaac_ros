"""Publish `odom -> base_footprint` from the QBot Platform stage instead of
`world -> base_link`, and stop two publishers fighting over base_link's parent.

Run this INSIDE Isaac Sim (Window > Script Editor, paste and Run) with the
simulation STOPPED, then press PLAY again. Back the stage up first:

    cp qbot_platform_workspace.usd qbot_platform_workspace.usd.bak-preodom

The problem
-----------
Cartographer aborts every scan with

    "odom" passed to lookupTransform argument target_frame does not exist

because the shipped stage never publishes an `odom` frame. What it publishes is:

    world          -> base_link        (ground truth)
    base_footprint -> base_link        (articulation expansion)
    base_link      -> base_footprint   (explicit target)
    base_link      -> sensors, wheels, casters

`/odom` exists as a *topic* (odomFrameId "odom", chassisFrameId "base_link") but
ROS2PublishOdometry emits a nav_msgs/Odometry message only - it never writes TF.
Nav2 and Cartographer both want the frame, not the message, so the whole stack
stalls on a topic that looks perfectly healthy in `ros2 topic echo`.

On top of that, `base_link` ends up with three parents at once (world,
base_footprint, and none-because-it-is-a-root). tf2 keeps one parent per child
and takes whichever arrived last, so lookups route through `world` or through
`base_footprint` at random, and `base_link <-> base_footprint` is a 2-cycle.
Fixing only the missing odom frame would leave that in place.

Two graph nodes cause all of it:

    ros2_publish_transform_tree      parentPrim = base_link
                                     targets    = 5 sensors, 2 wheels,
                                                  base_footprint   <- the cycle
    ros2_publish_transform_tree_01   parentPrim = <empty>          <- "world"
                                     targets    = base_link

An empty parentPrim is not "no parent" - Isaac falls back to the frame name
`world`. And because `base_footprint` carries PhysicsArticulationRootAPI,
targeting `base_link` makes Isaac expand the articulation from its real root
anyway, which is where `base_footprint -> base_link` comes from.

What this script changes
------------------------
Three relationships. No prim is created, no node is added or removed.

1. _01 parentPrim  <empty>    -> /World/odom
   The stage already carries an identity Xform named `odom` above the robot,
   built for exactly this and never wired up. Isaac names the parent frame after
   the prim, so this alone turns `world ->` into `odom ->`.
2. _01 targetPrims base_link  -> base_footprint
   Target the articulation root that Isaac was expanding to regardless. The
   published edge becomes `odom -> base_footprint` and base_link keeps the
   single parent the articulation gives it.
3. tree targetPrims           -> base_footprint removed
   The remaining half of the cycle. The other seven targets are untouched.

Result:

    odom -> base_footprint -> base_link -> {sensors, wheels, casters}

one parent per frame, and the same shape the QCar2 side of this workspace
assumes ("Isaac Sim always owns odom -> base_link").

left_wheel and right_wheel stay listed on both nodes, so they are published
twice at ~113 Hz each. Both copies name `base_link` as the parent, so this is
duplicate work rather than a conflict, and it is left alone - dropping targets
that are not broken is a separate decision.

Idempotent - re-running re-applies the same three values.

To reverse: clear _01's parentPrim, point it back at base_link, and add
base_footprint back to the other node's targets. Everything is authored as a
root-layer override; the referenced qbot_platform asset is never modified.

After running, press PLAY and confirm the frame exists and has one parent:

    ros2 run tf2_ros tf2_echo odom base_link      # resolves, no LOOKUP_ERROR
    ros2 topic echo /tf --once | grep -c world    # 0
"""

import omni.usd
from pxr import Usd

TF_NODE_TYPE = "isaacsim.ros2.bridge.ROS2PublishTransformTree"


def find_qbot_root(stage):
    """Locate the robot prim by structure, not by a hard-coded path.

    Same rule as the qcar2 scripts: the scene may nest the robot differently,
    but a prim named `qbot_platform` with a `base_link` child is unambiguous.
    """
    for prim in stage.Traverse():
        if prim.GetName() == "qbot_platform" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError(
        "No prim named 'qbot_platform' with a 'base_link' child on this stage")


def find_odom_prim(root):
    """The Xform the robot hangs under, which must be named `odom`.

    Isaac derives the published parent frame id from the prim NAME, so this is
    the one thing that cannot be substituted: an Xform named anything else would
    publish a frame Nav2 and Cartographer are not looking for.
    """
    odom = root.GetParent()
    if not odom.IsValid() or odom.GetName() != "odom":
        raise RuntimeError(
            f"Expected the robot's parent prim to be named 'odom', got "
            f"'{odom.GetPath()}'. Create an identity Xform named 'odom' above "
            f"{root.GetPath()} first, or edit ODOM lookup in this script.")
    return odom


def find_tf_nodes(root, base_link_path):
    """Split the two ROS2PublishTransformTree nodes by what they publish.

    Classified by parentPrim rather than by prim name: the node named `_01`
    happens to be the root publisher today, but the distinction that matters is
    which one hangs off base_link and which one supplies base_link's own parent.
    """
    sensors = root_pub = None
    for prim in Usd.PrimRange(root):
        attr = prim.GetAttribute("node:type")
        if not attr or attr.Get() != TF_NODE_TYPE:
            continue
        parent = prim.GetRelationship("inputs:parentPrim").GetTargets()
        if parent and str(parent[0]) == base_link_path:
            sensors = prim
        else:
            root_pub = prim

    if sensors is None or root_pub is None:
        raise RuntimeError(
            f"Expected two {TF_NODE_TYPE} nodes under {root.GetPath()} - one "
            f"parented to base_link and one supplying base_link's parent - "
            f"found sensors={sensors}, root={root_pub}")
    return sensors, root_pub


def set_targets(prim, relationship, paths):
    """Set a relationship and report whether it actually changed."""
    rel = prim.GetRelationship(relationship)
    before = [str(t) for t in rel.GetTargets()]
    after = [str(p) for p in paths]
    if before == after:
        return False
    rel.SetTargets(paths)
    return True


def main():
    stage = omni.usd.get_context().get_stage()
    root = find_qbot_root(stage)
    odom = find_odom_prim(root)

    base_link = root.GetChild("base_link").GetPath()
    footprint_prim = root.GetChild("base_footprint")
    if not footprint_prim.IsValid():
        raise RuntimeError(f"No base_footprint under {root.GetPath()}")
    base_footprint = footprint_prim.GetPath()

    sensors, root_pub = find_tf_nodes(root, str(base_link))

    changed = [
        set_targets(root_pub, "inputs:parentPrim", [odom.GetPath()]),
        set_targets(root_pub, "inputs:targetPrims", [base_footprint]),
        set_targets(sensors, "inputs:targetPrims",
                    [t for t in sensors.GetRelationship("inputs:targetPrims")
                     .GetTargets() if t != base_footprint]),
    ]

    print(f"[qbot] {root_pub.GetName()}: parentPrim  -> {odom.GetPath()}")
    print(f"[qbot] {root_pub.GetName()}: targetPrims -> {base_footprint}")
    print(f"[qbot] {sensors.GetName()}: targetPrims -> base_footprint removed "
          f"({len(sensors.GetRelationship('inputs:targetPrims').GetTargets())} left)")
    print(f"[qbot] {sum(changed)}/3 relationships changed"
          f"{' (already applied)' if not any(changed) else ''}")
    print("[qbot] TF is now  odom -> base_footprint -> base_link -> sensors")
    print("[qbot] press PLAY, then: ros2 run tf2_ros tf2_echo odom base_link")


main()
