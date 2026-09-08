Maps saved by `scripts/save_map.sh` land here (`qcar2_map.yaml` + `qcar2_map.pgm`,
plus a `.pbstream` of the Cartographer session).

This directory is installed into the package share dir, so
`qcar2_navigation_launch.py` finds `qcar2_map.yaml` by default after a rebuild.
