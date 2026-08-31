#!/usr/bin/env bash
# Save the map Cartographer has built into maps/<name>.yaml + <name>.pgm
# (and a .pbstream so the session can be resumed / re-exported later).
#
#   ros2 run qcar2_isaac_nav2 save_map.sh [map_name] [output_dir]
#
# Defaults to "qcar2_map" written into the SOURCE maps/ folder so it survives
# a colcon rebuild. Run this while qcar2_mapping_launch.py is still running.
set -uo pipefail

MAP_NAME="${1:-qcar2_map}"
DEFAULT_SRC_DIR="$HOME/entech_quanser_ros2_ws/src/qcar2_isaac_nav2/maps"
OUT_DIR="${2:-$DEFAULT_SRC_DIR}"

mkdir -p "$OUT_DIR"

if ! ros2 node list 2>/dev/null | grep -q '/cartographer_node'; then
  echo "ERROR: /cartographer_node is not running. Start qcar2_mapping_launch.py first." >&2
  exit 1
fi

echo "==> Finishing trajectory 0 (Cartographer stops adding new data)"
ros2 service call /finish_trajectory \
  cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}" >/dev/null 2>&1 \
  || echo "    (trajectory already finished - continuing)"

echo "==> Writing $OUT_DIR/$MAP_NAME.pbstream"
ros2 service call /write_state \
  cartographer_ros_msgs/srv/WriteState \
  "{filename: '$OUT_DIR/$MAP_NAME.pbstream', include_unfinished_submaps: true}" \
  >/dev/null 2>&1 \
  || echo "    (pbstream export failed - the .pgm below is what Nav2 actually needs)"

echo "==> Writing $OUT_DIR/$MAP_NAME.yaml / .pgm from the /map topic"
cd "$OUT_DIR" || exit 1
ros2 run nav2_map_server map_saver_cli -f "$MAP_NAME" -t /map --occ 0.65 --free 0.196

if [[ -f "$OUT_DIR/$MAP_NAME.yaml" && -f "$OUT_DIR/$MAP_NAME.pgm" ]]; then
  echo
  echo "Map saved:"
  ls -la "$OUT_DIR/$MAP_NAME".{yaml,pgm} 2>/dev/null
  echo
  echo "Next:  colcon build --packages-select qcar2_isaac_nav2"
  echo "       ros2 launch qcar2_isaac_nav2 qcar2_navigation_launch.py map:=$OUT_DIR/$MAP_NAME.yaml"
else
  echo "ERROR: map was not written. Is /map being published? (ros2 topic hz /map)" >&2
  exit 1
fi
