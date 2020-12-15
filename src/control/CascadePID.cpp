#include <uav_ros_control/control/CascadePID.hpp>
#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <geometry_msgs/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

#define CARROT_OFF "OFF"
#define CARROT_ON_LAND "CARROT_ON_LAND"
#define CARROT_ON_AIR "CARROT_ON_AIR"
#define POS_HOLD "HOLD"

// Define all parameter paths here
#define PID_X_PARAM "control/pos_x"
#define PID_Y_PARAM "control/pos_y"
#define PID_Z_PARAM "control/pos_z"
#define PID_VX_PARAM "control/vel_x"
#define PID_VY_PARAM "control/vel_y"
#define PID_VZ_PARAM "control/vel_z"

#define FFGAIN_VEL_X_PARAM "control/ff_gain/velocity/x"
#define FFGAIN_VEL_Y_PARAM "control/ff_gain/velocity/y"
#define FFGAIN_VEL_Z_PARAM "control/ff_gain/velocity/z"

#define FFGAIN_ACC_X_PARAM "control/ff_gain/acceleration/x"
#define FFGAIN_ACC_Y_PARAM "control/ff_gain/acceleration/y"
#define FFGAIN_ACC_Z_PARAM "control/ff_gain/acceleration/z"

#define HOVER_PARAM "control/hover"
#define GRAVITY_ACCELERATION 9.8;

uav_controller::CascadePID::CascadePID(ros::NodeHandle &nh)
  : _posXPID{ new PID("Position - x") }, _posYPID{ new PID("Position - y") },
    _posZPID{ new PID("Position - z") }, _velXPID{ new PID("Velocity - x") },
    _velYPID{ new PID("Velocity - y") }, _velZPID{ new PID("Velocity - z") },
    uav_controller::ControlBase(nh)
{
  // Initialize class parameters
  initializeParameters(nh);
  _velRefPub = nh.advertise<geometry_msgs::Vector3>("carrot/velocity", 1);
  _velCurrPub = nh.advertise<geometry_msgs::Vector3>("uav/velocity", 1);
  _yawRefSub = nh.subscribe("carrot/yaw", 1, &uav_controller::CascadePID::yawRefCb, this);
  _carrotStateSub =
    nh.subscribe("carrot/status", 1, &uav_controller::CascadePID::carrotStatusCb, this);

  // Setup dynamic reconfigure server
  uav_ros_control::PositionControlParametersConfig posConfig;
  setPositionReconfigureParams(posConfig);
  _posConfigServer.updateConfig(posConfig);
  _posParamCallback =
    boost::bind(&uav_controller::CascadePID::positionParamsCb, this, _1, _2);
  _posConfigServer.setCallback(_posParamCallback);

  // Initialize position hold service
  _serviceResetIntegrators = nh.advertiseService(
    "reset_integrator", &uav_controller::CascadePID::intResetServiceCb, this);
}

bool uav_controller::CascadePID::intResetServiceCb(
  std_srvs::Empty::Request & /* unused */,
  std_srvs::Empty::Response & /* unused */)
{
  ROS_WARN("Resetting all PID controllers");
  resetPositionPID();
  resetVelocityPID();
  return true;
}

bool uav_controller::CascadePID::activationPermission()
{
  return _carrotStatus == CARROT_ON_AIR || _carrotStatus == POS_HOLD;
}

void uav_controller::CascadePID::carrotStatusCb(const std_msgs::StringConstPtr &msg)
{
  _carrotStatus = msg->data;
}

void uav_controller::CascadePID::yawRefCb(const std_msgs::Float64ConstPtr &msg)
{
  _yawRef = msg->data;
}

void uav_controller::CascadePID::positionParamsCb(
  uav_ros_control::PositionControlParametersConfig &configMsg,
  uint32_t /* unused */)
{
  ROS_WARN("CascadePID::parametersCallback");

  _posYPID->set_kp(configMsg.k_p_xy);
  _posYPID->set_kd(configMsg.k_d_xy);
  _posYPID->set_ki(configMsg.k_i_xy);
  _posYPID->set_lim_high(configMsg.lim_high_xy);
  _posYPID->set_lim_low(configMsg.lim_low_xy);

  _velYPID->set_kp(configMsg.k_p_vxy);
  _velYPID->set_kd(configMsg.k_d_vxy);
  _velYPID->set_ki(configMsg.k_i_vxy);
  _velYPID->set_lim_high(configMsg.lim_high_vxy);
  _velYPID->set_lim_low(configMsg.lim_low_vxy);

  _posXPID->set_kp(configMsg.k_p_xy);
  _posXPID->set_kd(configMsg.k_d_xy);
  _posXPID->set_ki(configMsg.k_i_xy);
  _posXPID->set_lim_high(configMsg.lim_high_xy);
  _posXPID->set_lim_low(configMsg.lim_low_xy);

  _velXPID->set_kp(configMsg.k_p_vxy);
  _velXPID->set_kd(configMsg.k_d_vxy);
  _velXPID->set_ki(configMsg.k_i_vxy);
  _velXPID->set_lim_high(configMsg.lim_high_vxy);
  _velXPID->set_lim_low(configMsg.lim_low_vxy);

  _posZPID->set_kp(configMsg.k_p_z);
  _posZPID->set_kd(configMsg.k_d_z);
  _posZPID->set_ki(configMsg.k_i_z);
  _posZPID->set_lim_high(configMsg.lim_high_z);
  _posZPID->set_lim_low(configMsg.lim_low_z);

  _velZPID->set_kp(configMsg.k_p_vz);
  _velZPID->set_kd(configMsg.k_d_vz);
  _velZPID->set_ki(configMsg.k_i_vz);
  _velZPID->set_lim_high(configMsg.lim_high_vz);
  _velZPID->set_lim_low(configMsg.lim_low_vz);

  _ffGainVelocityX = configMsg.ff_vel_x;
  _ffGainVelocityY = configMsg.ff_vel_y;
  _ffGainVelocityZ = configMsg.ff_vel_z;

  _ffGainAccelerationX = configMsg.ff_acc_x;
  _ffGainAccelerationY = configMsg.ff_acc_y;
  _ffGainAccelerationZ = configMsg.ff_acc_z;

  _hoverThrust = configMsg.hover;
}

void uav_controller::CascadePID::setPositionReconfigureParams(
  uav_ros_control::PositionControlParametersConfig &config)
{
  ROS_WARN("CascadePID::setReconfigureParameters");

  config.k_p_xy = _posYPID->get_kp();
  config.k_i_xy = _posYPID->get_ki();
  config.k_d_xy = _posYPID->get_kd();
  config.lim_low_xy = _posYPID->get_lim_low();
  config.lim_high_xy = _posYPID->get_lim_high();

  config.k_p_vxy = _velYPID->get_kp();
  config.k_i_vxy = _velYPID->get_ki();
  config.k_d_vxy = _velYPID->get_kd();
  config.lim_low_vxy = _velYPID->get_lim_low();
  config.lim_high_vxy = _velYPID->get_lim_high();

  config.k_p_z = _posZPID->get_kp();
  config.k_i_z = _posZPID->get_ki();
  config.k_d_z = _posZPID->get_kd();
  config.lim_low_z = _posZPID->get_lim_low();
  config.lim_high_z = _posZPID->get_lim_high();

  config.k_p_vz = _velZPID->get_kp();
  config.k_i_vz = _velZPID->get_ki();
  config.k_d_vz = _velZPID->get_kd();
  config.lim_low_vz = _velZPID->get_lim_low();
  config.lim_high_vz = _velZPID->get_lim_high();

  config.ff_vel_x = _ffGainVelocityX;
  config.ff_vel_y = _ffGainVelocityY;
  config.ff_vel_z = _ffGainVelocityZ;
  config.ff_acc_x = _ffGainAccelerationX;
  config.ff_acc_y = _ffGainAccelerationY;
  config.ff_acc_z = _ffGainAccelerationZ;

  config.hover = _hoverThrust;
}

void uav_controller::CascadePID::initializeParameters(ros::NodeHandle &nh)
{
  ROS_WARN("CascadePID::initializeParameters()");

  _posYPID->initializeParameters(nh, PID_Y_PARAM);
  _velYPID->initializeParameters(nh, PID_VY_PARAM);
  _posXPID->initializeParameters(nh, PID_X_PARAM);
  _velXPID->initializeParameters(nh, PID_VX_PARAM);
  _posZPID->initializeParameters(nh, PID_Z_PARAM);
  _velZPID->initializeParameters(nh, PID_VZ_PARAM);

  bool initialized = nh.getParam(HOVER_PARAM, _hoverThrust)
                     && nh.getParam(FFGAIN_VEL_X_PARAM, _ffGainVelocityX)
                     && nh.getParam(FFGAIN_VEL_Y_PARAM, _ffGainVelocityY)
                     && nh.getParam(FFGAIN_VEL_Z_PARAM, _ffGainVelocityZ)
                     && nh.getParam(FFGAIN_ACC_X_PARAM, _ffGainAccelerationX)
                     && nh.getParam(FFGAIN_ACC_Y_PARAM, _ffGainAccelerationY)
                     && nh.getParam(FFGAIN_ACC_Z_PARAM, _ffGainAccelerationZ);

  ROS_INFO("Hover thrust: %.2f", _hoverThrust);
  ROS_INFO("Feed-forward velocity gain: [%.2f, %.2f, %.2f]",
    _ffGainVelocityX,
    _ffGainVelocityY,
    _ffGainVelocityZ);
  ROS_INFO("Feed-forward acceleration gain: [%.2f, %.2f, %.2f]",
    _ffGainAccelerationX,
    _ffGainAccelerationY,
    _ffGainAccelerationZ);
  if (!initialized) {
    ROS_FATAL("CascadePID::initalizeParameters() - failed to initialize parameters");
    throw std::runtime_error("CascadedPID parameters not properly initialized.");
  }
}

void uav_controller::CascadePID::resetPositionPID()
{
  _posXPID->resetIntegrator();
  _posYPID->resetIntegrator();
  _posZPID->resetIntegrator();
}

void uav_controller::CascadePID::resetVelocityPID()
{
  _velXPID->resetIntegrator();
  _velYPID->resetIntegrator();
  _velZPID->resetIntegrator();
}

void uav_controller::CascadePID::calculateAttThrustSp(double dt)
{
  // Calculate first row of PID controllers
  double velocityRefX = _posXPID->compute(
    getCurrentReference().transforms[0].translation.x, getCurrPosition()[0], dt);
  double velocityRefY = _posYPID->compute(
    getCurrentReference().transforms[0].translation.y, getCurrPosition()[1], dt);
  double velocityRefZ = _posZPID->compute(
    getCurrentReference().transforms[0].translation.z, getCurrPosition()[2], dt);

  // Add velocity feed-forward gains
  velocityRefX += _ffGainVelocityX * getCurrentReference().velocities[0].linear.x;
  velocityRefY += _ffGainVelocityY * getCurrentReference().velocities[0].linear.y;
  velocityRefZ += _ffGainVelocityZ * getCurrentReference().velocities[0].linear.z;

  // Calculate second row of PID controllers
  double roll = -_velYPID->compute(velocityRefY, getCurrVelocity()[1], dt);
  double pitch = _velXPID->compute(velocityRefX, getCurrVelocity()[0], dt);
  double thrust = _velZPID->compute(velocityRefZ, getCurrVelocity()[2], dt);
  thrust += _hoverThrust;

  // Add acceleration feed-forward gains
  roll += _ffGainAccelerationX * getCurrentReference().accelerations[0].linear.x
          / GRAVITY_ACCELERATION;
  pitch += _ffGainAccelerationY * getCurrentReference().accelerations[0].linear.y
           / GRAVITY_ACCELERATION;
  _yawRef += _ffGainAccelerationZ * getCurrentReference().accelerations[0].linear.z
             / GRAVITY_ACCELERATION;

  // Decouple roll and pitch w.r.t. yaw
  setAttitudeSp(cos(getCurrentYaw()) * roll + sin(getCurrentYaw()) * pitch,
    cos(getCurrentYaw()) * pitch - sin(getCurrentYaw()) * roll,
    _yawRef);

  setThrustSp(thrust);

  geometry_msgs::Vector3 newMsg;
  newMsg.x = velocityRefX;
  newMsg.y = velocityRefY;
  newMsg.z = velocityRefZ;
  _velRefPub.publish(newMsg);

  geometry_msgs::Vector3 vel;
  vel.x = getCurrVelocity()[0];
  vel.y = getCurrVelocity()[1];
  vel.z = getCurrVelocity()[2];
  _velCurrPub.publish(vel);
}

void uav_controller::runDefault(uav_controller::CascadePID &cascadeObj,
  ros::NodeHandle & /* unused */)
{
  constexpr auto rate = 50;
  double dt = 1.0 / rate;
  ros::Rate loopRate(rate);

  while (ros::ok()) {
    ros::spinOnce();
    if (cascadeObj.activationPermission()) {
      cascadeObj.calculateAttThrustSp(dt);
      cascadeObj.publishAttitudeTarget(MASK_IGNORE_RPY_RATE);
    } else {
      ROS_FATAL_THROTTLE(2, "CascadePID::runDefault - controller inactive");
    }
    cascadeObj.publishEulerSp();
    loopRate.sleep();
  }
}