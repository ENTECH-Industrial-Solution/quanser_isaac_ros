#!/usr/bin/env bash
# Save the occupancy grid RTAB-Map has built into maps/<name>.yaml + <name>.pgm.
#
#   ros2 run qcar2 save_vslam_map.sh [map_name] [output_dir]
#
# Defaults to "qcar2_vslam_map" written into the SOURCE maps/ folder so it
# survives a colcon rebuild. Run this while qcar2_vslam_mapping_launch.py is
# still running - /map is only published by the live node.
#
# The .pgm is for Nav2's map_server, i.e. the AMCL route
# (qcar2_navigation_launch.py map:=...). The camera route
# (qcar2_vslam_navigation_launch.py) does NOT need it: RTAB-Map republishes the
# grid straight from its database, which also holds the visual words needed to
# relocalise. Keep the database - ~/.ros/qcar2_vslam.db by default - if you want
# that route to work.
set -uo pipefail

MAP_NAME="${1:-qcar2_vslam_map}"
DEFAULT_SRC_DIR="$HOME/entech_quanser_ros2_ws/src/qcar2/maps"
OUT_DIR="${2:-$DEFAULT_SRC_DIR}"

mkdir -p "$OUT_DIR"

if ! ros2 node list 2>/dev/null | grep -q '/rtabmap'; then
  echo "ERROR: /rtabmap is not running. Start qcar2_vslam_mapping_launch.py first." >&2
  exit 1
fi

# Do NOT call /rtabmap/backup here. Despite the name it does not just copy the
# database: it saves the memory, copies the .db, then RE-INITIALISES working
# memory. /map is assembled from Working Memory only, so the grid collapses to
# whatever the camera can see from where the car is standing, and this script
# would then cheerfully save that 5x5 m stub over your map. The database itself
# keeps every node - if that already happened, relaunch with
# qcar2_vslam_navigation_launch.py (Mem/InitWMWithAllNodes=true loads the whole
# graph back into WM) and run this script again against that.
echo "==> Writing $OUT_DIR/$MAP_NAME.yaml / .pgm from the /map topic"
cd "$OUT_DIR" || exit 1
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" -t /map --occ 0.65 --free 0.196

if [[ -f "$OUT_DIR/$MAP_NAME.yaml" && -f "$OUT_DIR/$MAP_NAME.pgm" ]]; then
  echo
  echo "Map saved:"
  ls -la "$OUT_DIR/$MAP_NAME".{yaml,pgm} 2>/dev/null
  echo
  echo "Next:  colcon build --packages-select qcar2" 
  echo "  camera localization : ros2 launch qcar2 qcar2_vslam_navigation_launch.py"
  echo "  lidar/AMCL          : ros2 launch qcar2 qcar2_navigation_launch.py \\"
  echo "                          map:=$OUT_DIR/$MAP_NAME.yaml"
else
  echo "ERROR: map was not written. Is /map being published? (ros2 topic hz /map)" >&2
  echo "       RTAB-Map only publishes /map once it has added a node or two -" >&2
  echo "       drive the car a metre first." >&2
  exit 1
fi
