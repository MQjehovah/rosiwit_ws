cmake_minimum_required(VERSION 3.0.2)
project(xzrobot_ros_dependency)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs actionlib tf2_msgs visualization_msgs
)

add_message_files(FILES
  Area.msg
  CameraCalib.msg
  ChassisCtrl.msg
  ChassisInfo.msg
  CleanControl.msg
  CleanInfo.msg
  CovPath.msg
  Diagnostic.msg
  Environment.msg
  ImuCalib.msg
  LEDControl.msg
  LEDInfo.msg
  List.msg
  LocationInfo.msg
  MonitorInfo.msg
  MqttMessage.msg
  NaviCmdActionAction.msg
  NaviCmdActionActionFeedback.msg
  NaviCmdActionActionGoal.msg
  NaviCmdActionActionResult.msg
  NaviCmdActionFeedback.msg
  NaviCmdActionGoal.msg
  NaviCmdActionResult.msg
  NaviState.msg
  RobotCtrl.msg
  RobotInfo.msg
  SlamInfo.msg
  VirtualForm.msg
  VirtualObstacles.msg
  WallFollow.msg
  WorkStationCtrl.msg
  WorkStationInfo.msg
)

add_service_files(FILES
  CommonBoolService.srv
  CommonIntRequestService.srv
  CommonIntResponseService.srv
  CommonService.srv
  DeviceInfo.srv
  DeviceLogin.srv
  DeviceUser.srv
  GetChargingNavGoal.srv
  GetChargingPile.srv
  GetMap.srv
  MapSaver.srv
  MapServerInit.srv
  MapServerPro.srv
  NavStyle.srv
  NaviCmd.srv
  NaviCmd2.srv
  OccMapConverter.srv
  PathPlan.srv
  PgmConverter.srv
  Relocation.srv
  SaveNewMap.srv
  SetMap.srv
  SetVirtualObstacle.srv
  SlamModeSwitchService.srv
  StartMapOperation.srv
  StopAmcl.srv
  StopMapOperation.srv
  SwitchMap.srv
  SwitchMapCover.srv
  SystemVolume.srv
  TagDetect.srv
  UpdateMap.srv
  UpdateMapCover.srv
  UpdatePath.srv
  UpdateTask.srv
  WifiLocate.srv
  WifiSaveClear.srv
  WifiSetMinMatchingDegree.srv
  WifiUpdateDatabase.srv
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs actionlib tf2_msgs visualization_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs nav_msgs actionlib_msgs actionlib tf2_msgs visualization_msgs)
