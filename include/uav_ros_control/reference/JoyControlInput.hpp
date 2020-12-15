#ifndef JOY_CONTROL_INPUT_H
#define JOY_CONTROL_INPUT_H

// Own includes
#include <uav_ros_control/reference/JoyStructure.hpp>

// ROS includes
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

namespace uav_reference {
/**
 * This class is used for getting control inputs from ROS Joy messages.
 * It will initialize all index and scaling parameters. It provides control
 * inputs for attitude and position control.
 */
class JoyControlInput
{

public:
  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   */
  JoyControlInput(ros::NodeHandle &);
  virtual ~JoyControlInput();

  /**
   * Return the value for current x-position setpoint offset.
   */
  double getXOffsetManual();

  /**
   * Return the value for current y-position setpoint offset.
   */
  double getYOffsetManual();

  /**
   * Return the value for current z-position setpoint offset.
   */
  double getZOffsetManual();

  /**
   * Return the value for current roll setpoint.
   */
  double getRollSpManual();

  /**
   * Return the value for current pitch setpoint.
   */
  double getPitchSpManual();

  /**
   * Return the value for current yaw setpoint.
   */
  double getYawSpManual();

  /**
   * Return the unscaled value for current thrust setpoint.
   */
  double getThrustSpUnscaled();

  /**
   * Returns scale value for thrust.
   */
  double getThrustScale();

  /**
   * Return scale value for yaw.
   */
  double getYawScale();

  /**
   * Return current values of joy buttons array
   */
  const std::vector<int32_t> getJoyButtons();

  /**
   * Return vector of float values from Joy message.
   */
  const std::vector<float> getJoyAxes();

  /**
   * Returns true if joy inputs are active, i.e. greater than zero.
   */
  bool isJoyActive();

private:
  /**
   * Joystick callback function.
   */
  void joyCb(const sensor_msgs::JoyConstPtr &message);

  /**
   * Do all the parameter initialization here.
   */
  void initializeParameters(ros::NodeHandle &nh);

  /** Current Joy message set in the /joy callback function. */
  sensor_msgs::Joy _joyMsg;

  /** Indices - Joy structure */
  std::unique_ptr<joy_struct::ControlIndices> _controlIndices;

  /** Scale weights - Joy structure */
  std::unique_ptr<joy_struct::ScaleWeights> _attitudeScales;

  /** Scale weights - Joy structure */
  std::unique_ptr<joy_struct::ScaleWeights> _positionScales;

  /** Declare joy subscriber. */
  ros::Subscriber _subJoy;
};

}// namespace uav_reference

#endif /** JOY_CONTROL_INPUT_H */