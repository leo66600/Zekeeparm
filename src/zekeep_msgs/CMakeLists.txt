cmake_minimum_required(VERSION 3.8)
project(zekeep_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointMitCmd.msg"
  "msg/JointPosVelCmd.msg"
  "msg/JointMotorState.msg"
  "msg/ArmStatus.msg"
  "srv/SetZero.srv"
  "srv/MoveToPoseIK.srv"
  "srv/SetGripper.srv"
  "srv/GripperCommand.srv"
  "action/MoveToPose.action"
  "action/GripperGrasp.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
