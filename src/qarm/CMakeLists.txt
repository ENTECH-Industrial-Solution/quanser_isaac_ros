cmake_minimum_required(VERSION 3.8)
project(qarm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QArmDiagnostics.msg"
  "action/MoveQArm.action"
  DEPENDENCIES
    std_msgs
    action_msgs
    sensor_msgs
)

# The node code lives in the python module `qarm_nodes`, NOT `qarm`:
# rosidl_generate_interfaces already installs a python package named `qarm`
# (that is what `from qarm.action import MoveQArm` imports), so a second python
# package of the same name would fight it for the same __init__.py.
ament_python_install_package(qarm_nodes)

foreach(node qarm_hardware qarm_sim move_qarm_server move_qarm_client rgbd)
  install(PROGRAMS qarm_nodes/${node}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${node}
  )
endforeach()

install(DIRECTORY launch urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
