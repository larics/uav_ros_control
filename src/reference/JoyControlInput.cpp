#include <uav_ros_control/reference/JoyControlInput.hpp>
#include <uav_ros_control/filters/NonlinearFilters.hpp>
#include <math.h>

/** Minimum value for joy to be considered active. */
#define MIN_ACTIVE_VALUE 0.01

uav_reference::JoyControlInput::JoyControlInput(ros::NodeHandle &nh)
  : _controlIndices(new joy_struct::ControlIndices),
    _attitudeScales(new joy_struct::ScaleWeights),
    _positionScales(new joy_struct::ScaleWeights)
{
  // Initialize JoyMsg
  _joyMsg.axes = std::vector<float>(10, 0.0);
  _joyMsg.buttons = std::vector<int>(10, 0);

  // Initialize class parameters
  uav_reference::JoyControlInput::initializeParameters(nh);

  // Initialize Joy subscriber
  _subJoy = nh.subscribe("joy", 1, &uav_reference::JoyControlInput::joyCb, this);
}

uav_reference::JoyControlInput::~JoyControlInput() {}

void uav_reference::JoyControlInput::joyCb(const sensor_msgs::JoyConstPtr &message)
{
  _joyMsg = *message;
}

double uav_reference::JoyControlInput::getRollSpManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_LINEAR_Y] * _attitudeScales->LINEAR_Y;
}

double uav_reference::JoyControlInput::getPitchSpManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_LINEAR_X] * _attitudeScales->LINEAR_X;
}

double uav_reference::JoyControlInput::getYawSpManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_ANGULAR_YAW] * _attitudeScales->ANGULAR_Z;
}

double uav_reference::JoyControlInput::getThrustSpUnscaled()
{
  return (_joyMsg.axes[_controlIndices->AXIS_LINEAR_Z] + 1) / 2.0;
}

double uav_reference::JoyControlInput::getYOffsetManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_LINEAR_Y] * _positionScales->LINEAR_Y;
}

double uav_reference::JoyControlInput::getXOffsetManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_LINEAR_X] * _positionScales->LINEAR_X;
}

double uav_reference::JoyControlInput::getZOffsetManual()
{
  return _joyMsg.axes[_controlIndices->AXIS_LINEAR_Z] * _positionScales->LINEAR_Z;
}

double uav_reference::JoyControlInput::getYawScale()
{
  return _attitudeScales->ANGULAR_Z;
}

double uav_reference::JoyControlInput::getThrustScale()
{
  return _attitudeScales->LINEAR_Z;
}

bool uav_reference::JoyControlInput::isJoyActive()
{
  return abs(_joyMsg.axes[_controlIndices->AXIS_LINEAR_X]) > MIN_ACTIVE_VALUE
         || abs(_joyMsg.axes[_controlIndices->AXIS_LINEAR_Y]) > MIN_ACTIVE_VALUE
         || abs(_joyMsg.axes[_controlIndices->AXIS_LINEAR_Z]) > MIN_ACTIVE_VALUE
         || abs(_joyMsg.axes[_controlIndices->AXIS_ANGULAR_YAW]) > MIN_ACTIVE_VALUE;
}

void uav_reference::JoyControlInput::initializeParameters(ros::NodeHandle &nh)
{
  ROS_WARN("JoyControlInput::initializeParameters()");

  // Load all control inputs indices
  bool initialized =
    nh.getParam("joy/axis_linear/x", _controlIndices->AXIS_LINEAR_X)
    && nh.getParam("joy/axis_linear/y", _controlIndices->AXIS_LINEAR_Y)
    && nh.getParam("joy/axis_linear/z", _controlIndices->AXIS_LINEAR_Z)
    && nh.getParam("joy/axis_angular/yaw", _controlIndices->AXIS_ANGULAR_YAW);
  ROS_INFO_STREAM(*_controlIndices);
  if (!initialized) {
    ROS_FATAL(
      "JoyControlInput::initializeParameters() - \ 
			ControlIndices parameters are not properly set.");
    throw std::runtime_error("ControlIndices parameters not properly set.");
  }

  // Load all attitude scales
  initialized = nh.getParam("joy/scale_attitude/x", _attitudeScales->LINEAR_X)
                && nh.getParam("joy/scale_attitude/y", _attitudeScales->LINEAR_Y)
                && nh.getParam("joy/scale_attitude/z", _attitudeScales->LINEAR_Z)
                && nh.getParam("joy/scale_attitude/yaw", _attitudeScales->ANGULAR_Z);
  ROS_INFO_STREAM("Attitude " << *_attitudeScales);
  if (!initialized) {
    ROS_FATAL(
      "JoyControlInput::initializeParameters() - \
			Attitude weight parameters are not properly set.");
    throw std::runtime_error("Attitude weight parameters are not properly set.");
  }

  // Load all position scales
  _positionScales->ANGULAR_Z = 0;
  initialized = nh.getParam("joy/scale_position/x", _positionScales->LINEAR_X)
                && nh.getParam("joy/scale_position/y", _positionScales->LINEAR_Y)
                && nh.getParam("joy/scale_position/z", _positionScales->LINEAR_Z);
  ROS_INFO_STREAM("Position " << *_positionScales);
  if (!initialized) {
    ROS_FATAL(
      "JoyControlInput::initializeParameters() - \
			Position weight parameters are not properly set.");
    throw std::runtime_error("Position weight parameters are not properly set.");
  }
}

const std::vector<int32_t> uav_reference::JoyControlInput::getJoyButtons()
{
  return _joyMsg.buttons;
}

const std::vector<float> uav_reference::JoyControlInput::getJoyAxes()
{
  return _joyMsg.axes;
}