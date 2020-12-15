#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <uav_ros_control/reference/CarrotReference.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>

#define CARROT_OFF "OFF"
#define CARROT_ON_LAND "CARROT_ON_LAND"
#define CARROT_ON_AIR "CARROT_ON_AIR"
#define POS_HOLD "HOLD"

uav_reference::CarrotReference::CarrotReference(ros::NodeHandle &nh)
  : m_handlerState(nh, "mavros/state"), uav_reference::JoyControlInput(nh)
{
  // Define Publishers
  _pubCarrotTrajectorySp =
    nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("carrot/trajectory", 1);
  _pubCarrotYawSp = nh.advertise<std_msgs::Float64>("carrot/yaw", 1);
  _pubUAVYaw = nh.advertise<std_msgs::Float64>("uav/yaw", 1);
  _pubCarrotPose = nh.advertise<geometry_msgs::PoseStamped>("carrot/pose", 1);
  _pubCarrotStatus = nh.advertise<std_msgs::String>("carrot/status", 1);

  // Define Subscribers
  _subOdom = nh.subscribe("odometry", 1, &uav_reference::CarrotReference::odomCb, this);
  _subPosHoldRef = nh.subscribe(
    "position_hold/trajectory", 1, &uav_reference::CarrotReference::positionRefCb, this);

  // Initialize position hold service
  _servicePoisitionHold = nh.advertiseService(
    "position_hold", &uav_reference::CarrotReference::posHoldServiceCb, this);
  _serviceTakeoff = nh.advertiseService(
    "takeoff", &uav_reference::CarrotReference::takeoffServiceCb, this);

  _serviceLand =
    nh.advertiseService("land", &uav_reference::CarrotReference::landServiceCb, this);

  _setModeToLandClient = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  _intResetClient = nh.serviceClient<std_srvs::Empty>("reset_integrator");
  initializeParameters();

  if (!_manualTakeoffEnabled) {
    ROS_WARN("CarrotReference - Automatic takeoff enabled, position hold disabled");
    _positionHold = false;
  }

  // Initialize references
  _carrotPoint.transforms = std::vector<geometry_msgs::Transform>(1);
  _carrotPoint.velocities = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.accelerations = std::vector<geometry_msgs::Twist>(1);
}

uav_reference::CarrotReference::~CarrotReference() {}

bool uav_reference::CarrotReference::posHoldServiceCb(std_srvs::Empty::Request &request,
  std_srvs::Empty::Response &response)
{
  if (!_carrotEnabled) {
    ROS_FATAL(
      "CarrotReference::posHoldServiceCb - unable to activate POSITION_HOLD, carrot not "
      "enabled");
    return true;
  }

  if (_carrotOnLand) {
    ROS_FATAL(
      "CarrotReference::posHoldServiceCb - unable to activate POSITION_HOLD, carrot on "
      "land");
    return true;
  }

  if (!_positionHold) {
    ROS_WARN("CarrotReference::posHoldServiceCb - Position hold enabled");
    _positionHold = true;
    resetCarrot();
  }

  return true;
}

bool uav_reference::CarrotReference::landServiceCb(std_srvs::SetBool::Request &request,
  std_srvs::SetBool::Response &response)
{
  const auto set_response = [&response](bool success) { response.success = success; };

  if (_manualTakeoffEnabled) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, in MANUAL_TAKEOFF.");
    set_response(false);
    return true;
  }

  if (m_handlerState.getData().mode != "GUIDED_NOGPS") {
    ROS_FATAL(
      "CarrotReference::landServiceCb - unable to land, not in GUIDED_NOGPS mode");
    set_response(false);
    return true;
  }

  if (!_carrotEnabled) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, CARROT disabled");
    set_response(false);
    return true;
  }

  if (_carrotOnLand || !_takeoffHappened) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to land, UAV on land");
    set_response(false);
    return true;
  }

  if (!request.data) {
    ROS_FATAL("CarrotReference::landServiceCb - request to not land recieved.");
    set_response(false);
    return true;
  }

  // Assume we want to land at this point
  mavros_msgs::SetMode::Request landMode_req;
  mavros_msgs::SetMode::Response landMode_resp;
  landMode_req.custom_mode = "LAND";

  if (!_setModeToLandClient.call(landMode_req, landMode_resp)) {
    ROS_FATAL("CarrotReference::landServiceCb - unable to set LAND mode");
    set_response(false);
    return true;
  }

  _positionHold = false;
  _carrotOnLand = true;
  ROS_INFO("Carrotreference::landServiceCb - LAND finished");
  set_response(true);
  return true;
}

bool uav_reference::CarrotReference::takeoffServiceCb(
  uav_ros_control_msgs::TakeOff::Request &request,
  uav_ros_control_msgs::TakeOff::Response &response)
{
  const auto set_response = [&response](bool success) {
    response.success = success;
    if (success) {
      response.message = "Takeoff sucessful";
    } else {
      response.message = "Takeoff unsucessful";
    }
  };

  if (_manualTakeoffEnabled) {
    ROS_FATAL("CarrotReference::takeoffServiceCb - unable to takeoff, in MANUAL_TAKEOFF");
    set_response(false);
    return true;
  }

  // Check if the UAV is armed
  if (!m_handlerState.getData().armed) {
    ROS_FATAL("CarrotReference::takeoffServiceCb - unable to takeoff, not ARMED.");
    set_response(false);
    return true;
  }

  // Check if carrot is enabled
  if (!_carrotEnabled) {
    ROS_FATAL(
      "CarrotReference::takeoffServiceCb - unable to takeoff, carrot not enabled");
    set_response(false);
    return true;
  }

  if (!_carrotOnLand) {
    ROS_FATAL(
      "CarrotReference::takeoffServiceCb - unable to takeoff, not in CARROT_ON_LAND "
      "mode");
    set_response(false);
    return true;
  }

  resetCarrot();
  _carrotPoint.transforms[0].translation.z = _uavPos[2] + request.rel_alt;

  _positionHold = true;
  _takeoffHappened = true;
  _carrotOnLand = false;
  ROS_INFO("CarrotReference::takeoffServiceCb - enable position hold");

  set_response(true);
  return true;
}

void uav_reference::CarrotReference::positionRefCb(
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &posMsg)
{
  if (_positionHold) {
    _carrotPoint = *posMsg;
    _carrotYaw = util::calculateYaw(posMsg->transforms[0].rotation.x,
      posMsg->transforms[0].rotation.y,
      posMsg->transforms[0].rotation.z,
      posMsg->transforms[0].rotation.w);
    ROS_WARN_THROTTLE(5.0, "CarrotReference - Trajectory reference received");
  } else {
    ROS_FATAL(
      "CarrotReference - Trajectory reference recieved, but position hold mode is "
      "disabled.");
  }
}

void uav_reference::CarrotReference::resetIntegrators()
{
  std_srvs::Empty::Request req;
  std_srvs::Empty::Response resp;
  if (!_intResetClient.call(req, resp)) {
    ROS_FATAL("CarrotReference - Unable to reset integrators");
    return;
  }

  ROS_INFO("Controller integrators reset.");
}

void uav_reference::CarrotReference::updateCarrot()
{
  // Disable Position hold if carrot inputs exist
  if (_positionHold && isJoyActive()) {
    ROS_WARN("Position hold disabled - resetting carrot position");
    resetCarrot();
    _positionHold = false;
  }

  // Update carrot unless in position hold
  if (!_positionHold && _carrotEnabled && !_carrotOnLand) {
    updateCarrotXY();
    updateCarrotZ();
    updateCarrotYaw();
  } else if (!_positionHold && !_carrotEnabled) {
    // Map odometry to carrot
    resetCarrot();
  }
}

void uav_reference::CarrotReference::resetCarrot()
{
  _carrotPoint.transforms = std::vector<geometry_msgs::Transform>(1);
  _carrotPoint.velocities = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.accelerations = std::vector<geometry_msgs::Twist>(1);
  _carrotPoint.transforms[0].translation.x = _uavPos[0];
  _carrotPoint.transforms[0].translation.y = _uavPos[1];
  _carrotPoint.transforms[0].translation.z = _uavPos[2];

  // Set carrot orientation
  _carrotYaw = _uavYaw;
  tf2::Quaternion q;
  q.setEulerZYX(_carrotYaw, 0, 0);
  _carrotPoint.transforms[0].rotation.x = q.getX();
  _carrotPoint.transforms[0].rotation.y = q.getY();
  _carrotPoint.transforms[0].rotation.z = q.getZ();
  _carrotPoint.transforms[0].rotation.w = q.getW();
}


void uav_reference::CarrotReference::odomCb(const nav_msgs::OdometryConstPtr &msg)
{
  double qx = msg->pose.pose.orientation.x;
  double qy = msg->pose.pose.orientation.y;
  double qz = msg->pose.pose.orientation.z;
  double qw = msg->pose.pose.orientation.w;

  // Extract UAV yaw
  _uavYaw = util::calculateYaw(qx, qy, qz, qw);

  // Publish UAV yaw message
  std_msgs::Float64 uavYawMsg;
  uavYawMsg.data = _uavYaw;
  _pubUAVYaw.publish(uavYawMsg);

  // Extract UAV position
  _uavPos[0] = msg->pose.pose.position.x;
  _uavPos[1] = msg->pose.pose.position.y;
  _uavPos[2] = msg->pose.pose.position.z;

  if (_firstPass) {
    _firstPass = false;
    resetCarrot();
  }
}

void uav_reference::CarrotReference::updateCarrotYaw()
{
  // Update Carrot yaw angle and wrap to PI
  _carrotYaw += getYawSpManual();
  _carrotYaw = util::wrapMinMax(_carrotYaw, -M_PI, M_PI);

  tf2::Quaternion q;
  q.setEulerZYX(_carrotYaw, 0, 0);
  _carrotPoint.transforms[0].rotation.x = q.getX();
  _carrotPoint.transforms[0].rotation.y = q.getY();
  _carrotPoint.transforms[0].rotation.z = q.getZ();
  _carrotPoint.transforms[0].rotation.w = q.getW();
}

void uav_reference::CarrotReference::updateCarrotXY()
{
  updateCarrotXY(getXOffsetManual(), getYOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotZ()
{
  updateCarrotZ(getZOffsetManual());
}

void uav_reference::CarrotReference::updateCarrotXY(double xOff, double yOff)
{
  // Adjust carrot position w.r.t. the global coordinate system
  _carrotPoint.transforms[0].translation.x += cos(-_uavYaw) * xOff + sin(-_uavYaw) * yOff;
  _carrotPoint.transforms[0].translation.y += cos(-_uavYaw) * yOff - sin(-_uavYaw) * xOff;
}

void uav_reference::CarrotReference::updateCarrotZ(double zOff)
{
  _carrotPoint.transforms[0].translation.z += zOff;
}

void uav_reference::CarrotReference::publishCarrotSetpoint()
{
  tf2::Quaternion q;
  q.setEulerZYX(_carrotYaw, 0, 0);

  // Publish PoseStamped carrot reference
  _pubCarrotTrajectorySp.publish(_carrotPoint);

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = _carrotPoint.transforms[0].translation.x;
  pose.pose.position.y = _carrotPoint.transforms[0].translation.y;
  pose.pose.position.z = _carrotPoint.transforms[0].translation.z;
  pose.pose.orientation.x = q.getX();
  pose.pose.orientation.y = q.getY();
  pose.pose.orientation.z = q.getZ();
  pose.pose.orientation.w = q.getW();
  _pubCarrotPose.publish(pose);

  // Publish referent yaw message
  std_msgs::Float64 yawRefMsg;
  yawRefMsg.data = _carrotYaw;
  _pubCarrotYawSp.publish(yawRefMsg);
}

void uav_reference::CarrotReference::initializeParameters()
{
  ROS_WARN("CarrotReference::initializeParameters()");

  ros::NodeHandle nhPrivate("~");
  bool initialized = nhPrivate.getParam("carrot_index", _carrotEnabledIndex)
                     && nhPrivate.getParam("carrot_enable", _carrotEnabledValue)
                     && nhPrivate.getParam("manual_takeoff", _manualTakeoffEnabled);

  ROS_INFO("CarrotReference::initializeParameters() - carrot button enable index is %d",
    _carrotEnabledIndex);
  ROS_INFO("CarrotReference::initializeParameters() - carrot enable value is %d",
    _carrotEnabledValue);

  ROS_WARN_COND(_manualTakeoffEnabled,
    "CarrotReference::initializeParameters() - manual takeoff enabled");
  ROS_WARN_COND(!_manualTakeoffEnabled,
    "CarrotReference::initializeParameters() - automatic takeoff enabled");

  if (!initialized) {
    ROS_FATAL(
      "CarrotReference::initializeParameters() -\
			Carrot enable index not properly initialized.");
    throw std::runtime_error("CarrotReference parameters not properly set.");
  }
}

void uav_reference::CarrotReference::updateCarrotStatus()
{
  // Check if we landed after takeoff happened in automatic takeoff mode
  if (!_manualTakeoffEnabled && _takeoffHappened && !m_handlerState.getData().armed) {
    ROS_WARN("CarrotReference::updateCarrotStatus - UAV disarmed, assume LAND happened");
    _takeoffHappened = false;
    if (_carrotEnabled) { _carrotOnLand = true; }
    resetCarrot();
  }

  // Detect enable button - rising edge
  if (getJoyButtons()[_carrotEnabledIndex] == _carrotEnabledValue && !_carrotEnabled) {
    if (_manualTakeoffEnabled || _takeoffHappened) {
      _carrotEnabled = true;
      _carrotOnLand = false;
      ROS_INFO("CarrotReference::updateCarrotStatus - CARROT_ON_AIR enabled.");
      resetIntegrators();
      resetCarrot();
    } else if (!_carrotOnLand) {
      ROS_INFO("CarrotReference::updateCarrotStatus - CARROT_ON_LAND enabled.");
      _carrotEnabled = true;
      _carrotOnLand = true;
      resetIntegrators();
      resetCarrot();
    }
  }

  // Detect enable button - falling edge
  if (getJoyButtons()[_carrotEnabledIndex] == (1 - _carrotEnabledValue)
      && _carrotEnabled) {
    _carrotEnabled = false;
    _positionHold = false;
    _carrotOnLand = false;
    resetIntegrators();
    ROS_INFO("CarrotRefernce::updateCarrotStatus - carrot disabled.\n");
  }

  // Publish carrot status.
  std_msgs::String status;
  if (_positionHold && _carrotEnabled) {
    status.data = POS_HOLD;
  } else if (!_positionHold && _carrotEnabled) {
    if (_carrotOnLand) {
      status.data = CARROT_ON_LAND;
    } else {
      status.data = CARROT_ON_AIR;
    }
  } else {
    status.data = CARROT_OFF;
  }

  _pubCarrotStatus.publish(status);
}

bool uav_reference::CarrotReference::isCarrotEnabled() { return _carrotEnabled; }

bool uav_reference::CarrotReference::isHoldEnabled() { return _positionHold; }

void uav_reference::runDefault(uav_reference::CarrotReference &carrotRefObj,
  ros::NodeHandle &nh)
{
  double rate = 50;
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    carrotRefObj.updateCarrotStatus();
    carrotRefObj.updateCarrot();
    carrotRefObj.publishCarrotSetpoint();
    loopRate.sleep();
  }
}
