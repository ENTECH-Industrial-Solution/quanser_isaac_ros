"""Apply Script-Editor scripts to a USD stage headlessly and save it.

    ~/isaacsim/_build/linux-x86_64/release/python.sh \
        scripts/isaac_apply_and_save.py <stage.usd> <script.py> [more.py ...]

The scripts in this directory are written to be pasted into Isaac Sim's Script
Editor. This runs them without a GUI so the change can be scripted, reviewed and
repeated - useful when the same edit has to be re-applied after the Quanser
asset is updated.

Two things the GUI does for you that a headless SimulationApp does not:

1. `isaacsim.ros2.bridge` is not enabled. Without it `og.Controller.edit()`
   raises `OmniGraphError: Could not create node using unrecognized type
   'isaacsim.ros2.bridge.ROS2Context'`.
2. That extension needs a few app ticks to finish loading. Enabling it and
   opening a stage that already contains ROS 2 graphs in the same tick crashed
   omni.graph.core with `std::out_of_range: no null terminator at count`
   (Isaac Sim 6.0.1).

Hence: enable, tick, open, tick, apply, save - in that order.
"""

import sys

from isaacsim import SimulationApp

if len(sys.argv) < 3:
    print(__doc__)
    sys.exit(2)

STAGE, SCRIPTS = sys.argv[1], sys.argv[2:]

simulation_app = SimulationApp({"headless": True, "renderer": "RaytracedLighting"})

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402
import omni.usd  # noqa: E402

enable_extension("isaacsim.ros2.bridge")
for _ in range(10):
    simulation_app.update()

ctx = omni.usd.get_context()
print(f"[apply] opening {STAGE}", flush=True)
ctx.open_stage(STAGE)
for _ in range(60):
    simulation_app.update()

stage = ctx.get_stage()
root_layer = stage.GetRootLayer()
if root_layer.identifier != STAGE:
    print(f"[apply] WARNING root layer is {root_layer.identifier}", flush=True)

for path in SCRIPTS:
    print(f"[apply] running {path}", flush=True)
    exec(compile(open(path).read(), path, "exec"), {"__name__": "__main__"})

for _ in range(20):
    simulation_app.update()

print(f"[apply] save_stage -> {ctx.save_stage()}", flush=True)
simulation_app.update()
simulation_app.close()
