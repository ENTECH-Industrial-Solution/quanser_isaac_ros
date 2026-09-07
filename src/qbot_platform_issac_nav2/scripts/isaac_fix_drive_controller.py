"""Make the QBot Platform drivable: its articulation controller is never executed.

Run this INSIDE Isaac Sim (Window > Script Editor) with the simulation STOPPED,
then File > Save and REOPEN the stage before pressing Play - OmniGraph builds its
runtime graph when the stage loads and never re-reads USD afterwards, so an edit
applied to a live stage reports success and changes nothing until the reload.

Paste this one line rather than the whole file, so there is no chance of pasting
the wrong one:

    exec(open('/home/earth157/entech_quanser_ros2_ws/src/qbot_platform_issac_nav2/scripts/isaac_fix_drive_controller.py').read())

Back the stage up first:

    cp qbot_platform_workspace.usd qbot_platform_workspace.usd.bak-predrive

The problem
-----------
Nothing drives the robot. Publishing straight to the topic Isaac subscribes to
moves it 0.000 m and leaves /odom's twist at exactly zero, for angular commands
as well as linear.

Everything an obvious search turns up is healthy. Both wheel joints are enabled
velocity drives (stiffness 0, damping 0.29, effectively unlimited maxForce).
`ros2 topic info /cmd_vel_twist` shows Isaac's subscriber connected. The Kit log
has no [Error] for this graph. The graph's data path is fully wired:

    on_playback_tick -> ros2_subscribe_twist ("cmd_vel_twist")
                     -> break_3_vector(angular).z ---+
                     -> break_3_vector_01(linear).x -+-> differential_controller
                                                        .outputs:velocityCommand
                                                     -> articulation_controller
                                                        [left_wheel_joint,
                                                         right_wheel_joint]

The break is in the EXECUTION path, which is a separate set of connections:

    differential_controller.inputs:execIn  <- on_playback_tick.outputs:tick
    ros2_subscribe_twist.inputs:execIn     <- on_playback_tick.outputs:tick
    articulation_controller.inputs:execIn  <- NONE          <-- here

IsaacArticulationController is an action node. It writes joint commands only
when it is executed, and nothing executes it. differential_controller ticks
every frame and computes velocityCommand quite correctly; the value is simply
never written to the articulation. No error, because an untriggered action node
is not an error - it is just a branch of the graph nobody asked to run.

The working QCar2 stage in this same workspace confirms both halves of the fix:
its two articulation controllers take execIn from `ackermann_controller.
outputs:execOut`, and their `inputs:targetPrim` is `/World/odom/qcar2` - the
robot's Xform, not the prim carrying PhysicsArticulationRootAPI. So the QBot's
targetPrim (`/World/odom/qbot_platform`, likewise an Xform) is NOT a bug and is
left alone.

DifferentialController has no outputs:execOut at all - only
outputs:velocityCommand - so the only exec source available is the playback
tick, which is what differential_controller itself already uses. Both nodes then
tick every frame and OmniGraph pulls velocityCommand across.

A second defect: the robot cannot be told to go faster
------------------------------------------------------
Once it drives, it saturates at ~0.155 m/s no matter what is commanded -
teleop_twist_keyboard's speed multiplier climbs to 3.36 and nothing changes.

DifferentialController converts a linear command into a wheel rate with
`omega = v / wheelRadius`, then clamps that to maxWheelSpeed. The shipped
numbers do not describe this robot:

    wheelRadius    0.035     actual ~0.050
    wheelDistance  0.19      actual  0.394   (wheels sit at +/-0.197 in y)
    maxWheelSpeed  3.14151

With wheelRadius 1.4x too small the computed omega is 1.4x too high, so the
clamp binds from v = 3.14151 * 0.035 = 0.110 m/s upward. Every command above
that produces the same clamped omega = pi, hence the same ground speed:
pi * 0.050 = 0.157 m/s, which is what the robot measures at (0.155).

The true radius is confirmed twice over - 0.155 / 3.14151 = 0.0493 from motion,
and the wheel prims sit at z = +0.052 above the robot's ground plane.

wheelDistance being half the real track width is the same class of error, and
it halves every turn: commanding 1.0 rad/s measured 0.556 (0.19 / 0.394 = 0.48).
Left alone it makes Nav2 under-steer every correction.

maxWheelSpeed is raised to maxLinearSpeed / wheelRadius = 1.0 / 0.05 = 20 rad/s
so that maxLinearSpeed, the parameter that is supposed to express the top speed,
is the thing that actually limits it. What the robot then reaches is a physics
question - the wheel drives are velocity drives with damping 0.29, which is
weak - so measure it rather than assuming 1.0 m/s.

What this script changes
------------------------
1. Connects articulation_controller.inputs:execIn to on_playback_tick.outputs:tick.
   This is what makes the robot move at all.
2. Writes the measured geometry onto differential_controller: wheelRadius 0.05,
   wheelDistance 0.394, maxWheelSpeed 20.0. Without this a command means
   something other than what it says.
3. Prunes input connections that name a prim which is not on the stage. The
   shipped stage has exactly one - differential_controller.inputs:angularVelocity
   keeps a second connection to a `multiply` node that was deleted without its
   wire being cleaned up, leaving a malformed input holding two connections. That
   is a real defect and worth repairing, but it was NOT what stopped the robot:
   pruning it alone changed nothing. The scan stays so a run also proves nothing
   else rotted the same way.

   An attribute whose connections are ALL dead is reported and left alone.
   Dropping them would silently substitute the node's default, and
   angularVelocity's authored default here is -56.9 rad/s.

Idempotent - a second run finds execIn already connected and nothing dangling.

To reverse: restore the backup. Everything is authored into the stage's ROOT
layer; the referenced qbot_platform asset is never modified.

After the reload, press PLAY and confirm the wheels answer:

    ros2 topic pub -r 10 /cmd_vel_twist geometry_msgs/msg/Twist "{linear: {x: 0.2}}"
    ros2 topic echo /odom --field twist.twist.linear     # x should be ~0.2, not 0
"""

import omni.usd
from pxr import Sdf, Usd

ROS2_NODES = "ros2_nodes"

# Measured from the stage: wheels at +/-0.197 in y, wheel centres 0.052 above
# the ground plane. maxWheelSpeed is derived so the clamp never binds before
# the declared maxLinearSpeed (1.0 m/s) does.
GEOMETRY = {
    "wheelRadius": 0.05,
    "wheelDistance": 0.394,
    "maxWheelSpeed": 20.0,
}
DRIVE_GRAPH = "ros_qbot_platform_drive_controller"


def find_qbot_root(stage):
    """Locate the robot prim by structure, not by a hard-coded path."""
    for prim in stage.Traverse():
        if prim.GetName() == "qbot_platform" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError(
        "No prim named 'qbot_platform' with a 'base_link' child on this stage")


def connect_exec(graph):
    """Trigger the articulation controller from the playback tick.

    Returns True if the connection was added, False if it was already there.
    """
    ctrl = graph.GetChild("articulation_controller")
    tick = graph.GetChild("on_playback_tick")
    if not ctrl.IsValid() or not tick.IsValid():
        raise RuntimeError(
            f"Expected articulation_controller and on_playback_tick under "
            f"{graph.GetPath()}, found {ctrl.IsValid()=} {tick.IsValid()=}")

    src = tick.GetPath().AppendProperty("outputs:tick")
    attr = ctrl.GetAttribute("inputs:execIn")
    if not attr:
        attr = ctrl.CreateAttribute("inputs:execIn", Sdf.ValueTypeNames.UInt, True)

    existing = attr.GetConnections()
    if src in existing:
        print(f"[qbot] articulation_controller.inputs:execIn already <- {src}")
        return False

    if existing:
        # Not expected on the shipped stage, and overwriting someone else's
        # trigger silently would be worse than stopping here.
        raise RuntimeError(
            f"articulation_controller.inputs:execIn already has connections "
            f"{[str(c) for c in existing]} - refusing to overwrite. Inspect the "
            f"graph by hand.")

    attr.SetConnections([src])
    print(f"[qbot] articulation_controller.inputs:execIn  <- {src}")
    return True


def calibrate(graph):
    """Replace the differential controller's guessed geometry with the measured.

    Returns the number of values changed.
    """
    ctrl = graph.GetChild("differential_controller")
    if not ctrl.IsValid():
        raise RuntimeError(f"No differential_controller under {graph.GetPath()}")

    changed = 0
    for key, want in GEOMETRY.items():
        attr = ctrl.GetAttribute(f"inputs:{key}")
        if not attr:
            raise RuntimeError(f"differential_controller has no inputs:{key}")
        have = attr.Get()
        if have is not None and abs(have - want) < 1e-6:
            print(f"[qbot] {key:<14} already {want}")
            continue
        attr.Set(want)
        print(f"[qbot] {key:<14} {have} -> {want}")
        changed += 1
    return changed


def prune_dangling(stage, scope):
    """Rewrite every input whose connections name a prim that is not on the stage.

    Returns (repaired, orphaned). `orphaned` are attributes left untouched
    because they had no live connection to fall back on.
    """
    repaired = orphaned = 0
    for prim in Usd.PrimRange(scope):
        for attr in prim.GetAttributes():
            if not attr.HasAuthoredConnections():
                continue
            conns = attr.GetConnections()
            live = [c for c in conns if stage.GetPrimAtPath(c.GetPrimPath())]
            if len(live) == len(conns):
                continue

            print(f"[qbot] {attr.GetPath()}")
            for c in conns:
                print(f"[qbot]     {'KEEP' if c in live else 'DROP'} {c}")

            if not live:
                print("[qbot]     ^ no live connection left - LEFT ALONE, this "
                      "input would fall back to its default")
                orphaned += 1
                continue

            attr.SetConnections(live)
            repaired += 1
    return repaired, orphaned


def main():
    stage = omni.usd.get_context().get_stage()
    root = find_qbot_root(stage)

    scope = root.GetChild(ROS2_NODES)
    if not scope.IsValid():
        raise RuntimeError(f"No '{ROS2_NODES}' scope under {root.GetPath()}")
    graph = scope.GetChild(DRIVE_GRAPH)
    if not graph.IsValid():
        raise RuntimeError(f"No '{DRIVE_GRAPH}' graph under {scope.GetPath()}")

    wired = connect_exec(graph)
    tuned = calibrate(graph)
    repaired, orphaned = prune_dangling(stage, scope)

    if not repaired and not orphaned:
        print("[qbot] no dangling connections found")
    else:
        print(f"[qbot] {repaired} attribute(s) repaired, {orphaned} left alone")

    if wired or tuned:
        print("[qbot] now: File > Save, REOPEN the stage, PLAY")
    else:
        print("[qbot] nothing to change (already applied)")


main()
