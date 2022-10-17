cmake_minimum_required(VERSION 2.8.3)
project(dji_osdk_ros) ## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    nav_msgs
    nmea_msgs
    roscpp
    rospy
    std_msgs
    sensor_msgs
    message_filters # for message_filter
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules")

set(ENABLE_ADVANCED_SENSING 1)
if(ENABLE_ADVANCED_SENSING)
	MESSAGE(STATUS "Support Advanced Sensing")
	ADD_DEFINITIONS(-DADVANCED_SENSING)
ENDIF()

find_package(Eigen3 REQUIRED)
find_package(LibUSB REQUIRED)
find_package(FFMPEG REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate msg in the 'msg' folder
add_message_files(
  FILES
  FCTimeInUTC.msg
  JoystickParams.msg
  GPSUTC.msg
  Gimbal.msg
  Waypoint.msg
  WaypointList.msg
  WaypointV2AircraftControlActuatorFlying.msg
  WaypointV2AircraftControlActuatorRotateHeading.msg
  WaypointV2AircraftControlActuator.msg
  WaypointV2AssociateTrigger.msg
  WaypointV2CameraActuatorFocalLengthParam.msg
  WaypointV2CameraActuatorFocusParam.msg
  WaypointV2CameraActuator.msg
  WaypointV2Config.msg
  WaypointV2GimbalActuatorRotationParam.msg
  WaypointV2GimbalActuator.msg
  WaypointV2InitSetting.msg
  WaypointV2IntervalTrigger.msg
  WaypointV2ReachpointTrigger.msg
  WaypointV2SampleReachPointTrigger.msg
  WaypointV2TrajectoryTrigger.msg
  WaypointV2Action.msg
  WaypointV2.msg
  WaypointV2MissionEventPush.msg
  WaypointV2MissionStatePush.msg

  MobileData.msg
  PayloadData.msg
  MissionWaypointAction.msg
  MissionWaypoint.msg
  MissionWaypointTask.msg
  MissionHotpointTask.msg
  FlightAnomaly.msg
  VOPosition.msg
  BatteryState.msg
  BatteryWholeInfo.msg
  SmartBatteryState.msg
  SmartBatteryDynamicInfo.msg
  HMSPushInfo.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES

  GetDroneType.srv
  GetM300StereoParams.srv
  FlightTaskControl.srv
  SetJoystickMode.srv
  JoystickAction.srv
  ObtainControlAuthority.srv
  KillSwitch.srv
  EmergencyBrake.srv
  GimbalAction.srv
  CameraEV.srv
  CameraShutterSpeed.srv
  CameraAperture.srv
  CameraISO.srv
  CameraFocusPoint.srv
  CameraTapZoomPoint.srv
  CameraSetZoomPara.srv
  CameraZoomCtrl.srv
  CameraStartShootSinglePhoto.srv
  CameraStartShootBurstPhoto.srv
  CameraStartShootAEBPhoto.srv
  CameraStartShootIntervalPhoto.srv
  CameraStopShootPhoto.srv
  CameraRecordVideoAction.srv
  GetWholeBatteryInfo.srv
  GetSingleBatteryDynamicInfo.srv
  MFIO.srv
  SetGoHomeAltitude.srv
  GetGoHomeAltitude.srv
  SetCurrentAircraftLocAsHomePoint.srv
  SetHomePoint.srv
  SetupCameraH264.srv
  SetAvoidEnable.srv
  GetAvoidEnable.srv
  InitWaypointV2Setting.srv
  UploadWaypointV2Mission.srv
  UploadWaypointV2Action.srv
  DownloadWaypointV2Mission.srv
  StartWaypointV2Mission.srv
  StopWaypointV2Mission.srv
  PauseWaypointV2Mission.srv
  ResumeWaypointV2Mission.srv
  GenerateWaypointV2Action.srv
  SetGlobalCruisespeed.srv
  GetGlobalCruisespeed.srv
  GetHMSData.srv
  SubscribeWaypointV2Event.srv
  SubscribeWaypointV2State.srv

  Activation.srv
  CameraAction.srv
  DroneTaskControl.srv
  SDKControlAuthority.srv
  SetLocalPosRef.srv
  MFIOConfig.srv
  MFIOSetValue.srv
  DroneArmControl.srv
  MissionStatus.srv
  MissionWpAction.srv
  MissionHpAction.srv
  MissionWpUpload.srv
  MissionWpSetSpeed.srv
  MissionWpGetSpeed.srv
  MissionWpGetInfo.srv
  MissionHpUpload.srv
  MissionHpGetInfo.srv
  MissionHpUpdateYawRate.srv
  MissionHpUpdateRadius.srv
  MissionHpResetYaw.srv
  SendMobileData.srv
  SendPayloadData.srv
  SetHardSync.srv
  QueryDroneVersion.srv
  Stereo240pSubscription.srv
  StereoVGASubscription.srv
  StereoDepthSubscription.srv
  SetupCameraStream.srv
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   geometry_msgs
   nav_msgs
   std_msgs  # Or other packages containing msgs
   sensor_msgs
 )


MESSAGE(STATUS "USB INCLUDE: ${LIBUSB_1_INCLUDE_DIR}")
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES onboard-sdk-ros
  CATKIN_DEPENDS
    message_runtime
    geometry_msgs
    nav_msgs
    std_msgs
    nmea_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 #${DJIOSDK_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
 ${LIBUSB_1_INCLUDE_DIR}
 ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/onboard-sdk-ros.cpp
# )