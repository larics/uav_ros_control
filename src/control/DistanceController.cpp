#include <uav_ros_control/control/DistanceController.hpp>
#include <uav_ros_control/filters/NonlinearFilters.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <array>

#define DIST_PID_PARAMS "control/distance"
#define DISTVEL_PID_PARAMS "control/distance_vel"
#define DIST_SP_DEADZONE 0.01

dist_control::DistanceControl::DistanceControl(ros::NodeHandle &nh)
  : _distancePID{ new PID("Distance") }, _distanceVelPID{ new PID("Distance vel") },
    _distanceMeasured(-1), _distanceVelocityMeasured(0), _distVelSp(0), _distSp(-1),
    _distSpOffset(0), _inspectionRequestFailed(false), _planeYaw(0),
    _currState(DistanceControlState::MANUAL), uav_controller::CascadePID(nh)
{
  // Initialize Class parameters
  initializeParameters(nh);

  // Setup all subscribers
  _subDistance =
    nh.subscribe("plane/distance", 1, &dist_control::DistanceControl::distanceCb, this);
  _subDistanceVelocity = nh.subscribe(
    "plane/distance_vel", 1, &dist_control::DistanceControl::distanceVelCb, this);
  _subPlaneNormal =
    nh.subscribe("plane/normal", 1, &dist_control::DistanceControl::normalCb, this);
  _subDistSpOffset = nh.subscribe("plane/distance_sp/offset",
    1,
    &dist_control::DistanceControl::distanceSpOffsetCb,
    this);

  // Setup all publishers
  _pubControlState = nh.advertise<std_msgs::Int32>("plane/control_state", 1);
  _pubDistanceSp = nh.advertise<std_msgs::Float64>("plane/distance_sp", 1);
  _pubDistanceVelocitySp = nh.advertise<std_msgs::Float64>("plane/distance_vel_sp", 1);

  _serviceEnableInspection = nh.advertiseService(
    "plane/inspection_mode", &dist_control::DistanceControl::enableInspectionCb, this);

  // Setup dynamic reconfigure server
  uav_ros_control::DistanceControlParametersConfig distConfig;
  setDistReconfigureParams(distConfig);
  _distConfigServer.updateConfig(distConfig);
  _distParamCallback =
    boost::bind(&dist_control::DistanceControl::distParamCb, this, _1, _2);
  _distConfigServer.setCallback(_distParamCallback);
}

dist_control::DistanceControl::~DistanceControl()
{
  // TODO Auto-generated destructor stub
}

bool dist_control::DistanceControl::enableInspectionCb(
  std_srvs::SetBool::Request &request,
  std_srvs::SetBool::Response &response)
{
  // If user wants to disable inspection
  if (!request.data) {
    ROS_WARN("DistanceControl - deactivation request recieved");
    deactivateInspection();
    response.message = "Deactivation request successful";
    response.success = false;
    return true;
  }

  // User wants to activate inspection
  _inspectionEnabled = true;
  detectStateChange();

  // Report if state was successfully changed
  if (inInspectionState()) {
    ROS_INFO("DistanceControl - enabled");
    response.message = "Successfully entered inspection state";
    response.success = true;
  } else {
    ROS_FATAL("DistanceControl - unable to enter inspection");
    response.message = "Unable to enter inspection state";
    response.success = false;
    _inspectionEnabled = false;
  }

  return true;
}

void dist_control::DistanceControl::distanceCb(const std_msgs::Float64ConstPtr &message)
{
  _distanceMeasured = message->data;
}

void dist_control::DistanceControl::distanceVelCb(
  const std_msgs::Float64ConstPtr &message)
{
  _distanceVelocityMeasured = message->data;
}


void dist_control::DistanceControl::distanceSpOffsetCb(
  const std_msgs::Float64ConstPtr &message)
{
  _distSpOffset = message->data;
}

void dist_control::DistanceControl::normalCb(
  const geometry_msgs::PoseStampedConstPtr &message)
{
  _planeYaw = util::calculateYaw(message->pose.orientation.x,
    message->pose.orientation.y,
    message->pose.orientation.z,
    message->pose.orientation.w);

  // Check in which direction is plane normal facing
  double xComponent = cos(_planeYaw);
  if (xComponent < 0) _planeYaw += M_PI;

  _planeYaw = util::wrapMinMax(_planeYaw, -M_PI, M_PI);
}

void dist_control::DistanceControl::detectStateChange()
{
  // If we're in inspection mode and received distance is invalid
  // then deactivate inspection mode !
  if (inspectionFailed()) {
    ROS_FATAL("Inspection mode failed - invalid distance");
    deactivateInspection();
    _inspectionRequestFailed = true;
    return;
  }

  // Check if user wants to go to inspection mode
  if (inspectionRequested() && !_inspectionRequestFailed) {
    ROS_DEBUG("Inspection mode requested.");
    // Check if current distance is valid
    if (_distanceMeasured < 0) {
      ROS_FATAL("Unable to enter inspection mode.");
      _inspectionRequestFailed = true;
      return;
    }

    ROS_INFO(
      "Inspection activation successful-following distance %.2f", _distanceMeasured);
    _distSp = _distanceMeasured;
    _currState = DistanceControlState::INSPECTION;
    return;
  }

  // Check if user wants to go back to manual mode
  if (manualRequested()) {
    ROS_WARN("Manual mode entered.");
    _inspectionRequestFailed = false;
    deactivateInspection();
  }
}

void dist_control::DistanceControl::deactivateInspection()
{
  _currState = DistanceControlState::MANUAL;
  _inspectionEnabled = false;

  // Reset all PIDs
  _distancePID->resetIntegrator();
  _distanceVelPID->resetIntegrator();
  resetPositionPID();
  resetVelocityPID();
  ROS_WARN("Inspection mode deactivated successfully.");
}

void dist_control::DistanceControl::publishState()
{
  // Publish 0 for manual state and 1 for inspection state
  std_msgs::Int32 newMessage;
  newMessage.data = inInspectionState() ? 1 : 0;
  _pubControlState.publish(newMessage);
}

void dist_control::DistanceControl::publishDistVelSp()
{
  std_msgs::Float64 newMessage;
  newMessage.data = _distVelSp;
  _pubDistanceVelocitySp.publish(newMessage);
}

void dist_control::DistanceControl::calculateInspectionSetpoint(double dt)
{
  doDistanceControl(dt);
}

void dist_control::DistanceControl::doDistanceControl(double dt)
{
  calculateAttThrustSp(dt);

  // update distance setpoint
  _distSp -=
    nonlinear_filters::deadzone(_distSpOffset, -DIST_SP_DEADZONE, DIST_SP_DEADZONE);

  // Calculate pitch setpoint using measured distance
  _distVelSp = _distancePID->compute(_distSp, _distanceMeasured, dt);
  double pitch = -_distanceVelPID->compute(_distVelSp, _distanceVelocityMeasured, dt);
  double yaw = getCurrentYaw() - _planeYaw;// TODO: Minus ili plus

  overridePitchTarget(pitch);
  overrideYawTarget(yaw);
}

void dist_control::DistanceControl::publishDistSp()
{
  std_msgs::Float64 newMessage;
  newMessage.data = _distSp;
  _pubDistanceSp.publish(newMessage);
}

bool dist_control::DistanceControl::inInspectionState()
{
  return _currState == DistanceControlState::INSPECTION;
}

bool dist_control::DistanceControl::inspectionRequested()
{
  return inspectionEnabled() && _currState == DistanceControlState::MANUAL;
}

bool dist_control::DistanceControl::inspectionFailed()
{
  return _currState == DistanceControlState::INSPECTION && _distanceMeasured < 0;
}

bool dist_control::DistanceControl::manualRequested()
{
  return !inspectionEnabled()
         && (_currState == DistanceControlState::INSPECTION || _inspectionRequestFailed);
}

void dist_control::DistanceControl::initializeParameters(ros::NodeHandle &nh)
{
  ROS_WARN("DistanceControl::initializeParameters()");

  _distancePID->initializeParameters(nh, DIST_PID_PARAMS);
  _distanceVelPID->initializeParameters(nh, DISTVEL_PID_PARAMS);
}

void dist_control::DistanceControl::distParamCb(
  uav_ros_control::DistanceControlParametersConfig &configMsg,
  uint32_t level)
{
  ROS_WARN("DistanceControl::parametersCallback");

  _distancePID->set_kp(configMsg.k_p_dist);
  _distancePID->set_kd(configMsg.k_d_dist);
  _distancePID->set_ki(configMsg.k_i_dist);
  _distancePID->set_lim_high(configMsg.lim_high_dist);
  _distancePID->set_lim_low(configMsg.lim_low_dist);

  _distanceVelPID->set_kp(configMsg.k_p_vdist);
  _distanceVelPID->set_kd(configMsg.k_d_vdist);
  _distanceVelPID->set_ki(configMsg.k_i_vdist);
  _distanceVelPID->set_lim_high(configMsg.lim_high_vdist);
  _distanceVelPID->set_lim_low(configMsg.lim_low_vdist);
}

void dist_control::DistanceControl::setDistReconfigureParams(
  uav_ros_control::DistanceControlParametersConfig &config)
{
  ROS_WARN("DistanceControl::setReconfigureParameters");

  config.k_p_dist = _distancePID->get_kp();
  config.k_i_dist = _distancePID->get_ki();
  config.k_d_dist = _distancePID->get_kd();
  config.lim_low_dist = _distancePID->get_lim_low();
  config.lim_high_dist = _distancePID->get_lim_high();

  config.k_p_vdist = _distanceVelPID->get_kp();
  config.k_i_vdist = _distanceVelPID->get_ki();
  config.k_d_vdist = _distanceVelPID->get_kd();
  config.lim_low_vdist = _distanceVelPID->get_lim_low();
  config.lim_high_vdist = _distanceVelPID->get_lim_high();
}

bool dist_control::DistanceControl::inspectionEnabled() { return _inspectionEnabled; }

void dist_control::DistanceControl::publishDistanceInfo()
{
  publishState();
  publishDistSp();
  publishDistVelSp();
}

void dist_control::runDefault(dist_control::DistanceControl &dc, ros::NodeHandle &nh)
{
  // Setup loop rate
  double rate = 50;
  ros::Rate loopRate(rate);
  double dt = 1.0 / rate;
  ROS_INFO("dist_control::runDeafult() - Setting rate to %.2f", rate);

  // Start the main loop
  while (ros::ok()) {
    ros::spinOnce();

    dc.detectStateChange();

    // Do  Inspection
    if (dc.inInspectionState()) dc.calculateInspectionSetpoint(dt);

    // If not in inspection state go to attitude control
    else
      dc.calculateAttThrustSp(dt);

    // Publish attitude setpoint
    dc.publishAttitudeTarget(uav_controller::MASK_IGNORE_RP_RATE);

    // Publish other information
    dc.publishDistanceInfo();
    dc.publishEulerSp();

    loopRate.sleep();
  }
}
