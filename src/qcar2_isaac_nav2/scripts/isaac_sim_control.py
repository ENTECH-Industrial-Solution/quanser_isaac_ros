"""Drive Isaac Sim's timeline and the QCar2's start pose from outside the GUI.

Run this INSIDE Isaac Sim (Window > Script Editor, paste and Run) once. It then
sits on the app's update loop watching a command file, so everything after that
is scriptable:

    echo '{"op": "reset"}' > /tmp/qcar2_sim_cmd.json
    cat /tmp/qcar2_sim_ack.json

Why a file and not a topic: Isaac's ROS 2 bridge is a C++ extension and does not
put `rclpy` on Kit's interpreter, so a script pasted here cannot open a ROS node
to be commanded over. A file is the one channel both sides always have, and the
polling costs a `os.path.exists` per frame.

    {"op": "play"}      start the timeline
    {"op": "pause"}     pause where it is
    {"op": "stop"}      stop and rewind - this is what resets physics AND /clock
    {"op": "reset"}     stop then play: the repeatable start for a test run
    {"op": "status"}    is it playing, what is the sim time, where is the car
    {"op": "pose", "xyz": [x, y, z], "yaw_deg": 0, "then_play": true}
                        put the car somewhere known. Only meaningful while
                        stopped, so this stops first; physics initialises from
                        USD on the next play.
    {"op": "quit"}      unsubscribe and let the script be re-pasted

STOPPING THE TIMELINE REWINDS /clock, which every ROS node using sim time reads
as time running backwards: TF buffers keep stale futures, timers do not fire,
and Cartographer aborts outright (see README). So restart the ROS side after any
`stop` or `reset` - that is a property of the simulator, not of this script.

The pose op writes the robot root's existing translate/orient ops rather than
replacing the op stack, so the stage keeps the structure the asset shipped with.
Nothing here saves the stage; close without saving and the session is unchanged.
"""

import json
import os
import traceback

import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, UsdGeom

CMD_FILE = "/tmp/qcar2_sim_cmd.json"
ACK_FILE = "/tmp/qcar2_sim_ack.json"


def find_qcar2_root(stage):
    """Locate the robot prim - /qcar2 standalone, /World/odom/qcar2 in a scene."""
    for prim in stage.Traverse():
        if prim.GetName() == "qcar2" and prim.GetChild("base_link").IsValid():
            return prim
    raise RuntimeError("No prim named 'qcar2' with a 'base_link' child on this stage")


def _ops(prim):
    """The prim's translate and orient ops, by name."""
    return {op.GetOpName(): op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()}


def get_pose(prim):
    ops = _ops(prim)
    t = ops.get("xformOp:translate")
    xyz = list(t.Get()) if t else [0.0, 0.0, 0.0]
    return [float(v) for v in xyz]


def set_pose(prim, xyz, yaw_deg):
    """Write translate and yaw onto the ops the asset already has.

    Replacing the op stack with a matrix would work too, but it rewrites part of
    the shipped asset's structure for the sake of one test run.
    """
    ops = _ops(prim)
    if xyz is not None:
        t = ops.get("xformOp:translate")
        if t is None:
            t = UsdGeom.Xformable(prim).AddTranslateOp()
        t.Set(Gf.Vec3d(*[float(v) for v in xyz]))
    if yaw_deg is not None:
        q = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg)).GetQuat()
        for name in ("xformOp:orient", "xformOp:rotateXYZ"):
            op = ops.get(name)
            if op is None:
                continue
            if name == "xformOp:orient":
                # Single vs double precision is authored per asset; match it or
                # USD refuses the value with a type error.
                current = op.Get()
                op.Set(Gf.Quatf(q) if isinstance(current, Gf.Quatf) else Gf.Quatd(q))
            else:
                op.Set(Gf.Vec3d(0.0, 0.0, float(yaw_deg)))
            return
        UsdGeom.Xformable(prim).AddOrientOp().Set(Gf.Quatd(q))


class Control:

    def __init__(self):
        self.timeline = omni.timeline.get_timeline_interface()
        self.sub = (omni.kit.app.get_app().get_update_event_stream()
                    .create_subscription_to_pop(self.on_update, name="qcar2_sim_control"))
        self.pending_play = 0
        print(f"[qcar2] sim control listening on {CMD_FILE}")

    def robot(self):
        return find_qcar2_root(omni.usd.get_context().get_stage())

    def status(self):
        return {"playing": bool(self.timeline.is_playing()),
                "time": float(self.timeline.get_current_time()),
                "pose": get_pose(self.robot())}

    def on_update(self, _event):
        # A play queued behind a stop needs a frame in between: stopping tears
        # down the physics scene, and playing in the same frame comes back with
        # the articulation half-initialised and the car twitching on the spot.
        if self.pending_play:
            self.pending_play -= 1
            if self.pending_play == 0:
                self.timeline.play()
            return
        if not os.path.exists(CMD_FILE):
            return
        try:
            with open(CMD_FILE) as fh:
                cmd = json.load(fh)
        except Exception as exc:                                  # noqa: BLE001
            self.ack({"ok": False, "msg": f"unreadable command: {exc}"})
            os.remove(CMD_FILE)
            return
        os.remove(CMD_FILE)
        try:
            self.ack(self.run(cmd))
        except Exception as exc:                                  # noqa: BLE001
            traceback.print_exc()
            self.ack({"ok": False, "op": cmd.get("op"), "msg": str(exc)})

    def run(self, cmd):
        op = cmd.get("op")
        if op == "play":
            self.timeline.play()
        elif op == "pause":
            self.timeline.pause()
        elif op == "stop":
            self.timeline.stop()
        elif op == "reset":
            self.timeline.stop()
            self.pending_play = 3
        elif op == "pose":
            self.timeline.stop()
            set_pose(self.robot(), cmd.get("xyz"), cmd.get("yaw_deg"))
            if cmd.get("then_play", True):
                self.pending_play = 3
        elif op == "status":
            pass
        elif op == "quit":
            self.sub.unsubscribe()
            return {"ok": True, "op": op, "msg": "unsubscribed"}
        else:
            return {"ok": False, "op": op, "msg": f"unknown op {op!r}"}
        out = {"ok": True, "op": op}
        out.update(self.status())
        return out

    def ack(self, payload):
        with open(ACK_FILE, "w") as fh:
            json.dump(payload, fh)


# Re-pasting replaces the previous instance rather than stacking a second
# subscription on the update stream.
try:
    _qcar2_control.sub.unsubscribe()                              # noqa: F821
except NameError:
    pass
_qcar2_control = Control()
