cmake_minimum_required(VERSION 3.22)
project(piper_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FindObjects.action"
  "action/MoveArmToNamedPose.action"
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/RunStage.action"
  "action/RunTree.action"
  "action/ViewWorkspace.action"
  "msg/BlackboardEntry.msg"
  "msg/DetectedObject.msg"
  "msg/Grasp.msg"
  "msg/Segment.msg"
  "msg/TreeNodeStatus.msg"
  "msg/TreeStatus.msg"
  "srv/AllowContacts.srv"
  "srv/AttachPart.srv"
  "srv/ComputeGrasps.srv"
  "srv/ControlTree.srv"
  "srv/PlaceTarget.srv"
  "srv/SegmentObjects.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
