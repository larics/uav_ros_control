/*
 * DistanceControl.h
 *
 *  Created on: Apr 13, 2019
 *      Author: lmark
 */

#ifndef DISTANCE_CONTROL_H
#define DISTANCE_CONTROL_H

#include <uav_ros_control/control/CascadePID.hpp>
#include <uav_ros_control/reference/JoyStructure.hpp>
#include <uav_ros_control/DistanceControlParametersConfig.h>

// ROS Includes
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

#include <iostream>
#include <array>

namespace dist_control {
/** Name of dynamic reconfigure node. */
#define DIST_DYN_RECONF "distance_config"

enum DistanceControlState {
  /**
   * User controls the UAV manually using joystick commands.
   */
  MANUAL,

  /**
   * User controls the UAV while it performs inspection, maintaining
   * the distance from the callback function feedback loop.
   */
  INSPECTION
};

class DistanceControl : public uav_controller::CascadePID
{

public:
  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   *
   * @param nh - given ROS node handle
   */
  DistanceControl(ros::NodeHandle &nh);
  virtual ~DistanceControl();

  /**
   * Change the state back to manual if received distance is invalid.
   * Change the state to inspection mode if appropriate joystick command is
   * given.
   */
  void detectStateChange();

  /**
   * Calculate target attitude setpoint while in inspection mode.
   * During Inspection mode UAV is controlled using "Carrot commands" i.e.
   * position control.
   *
   * @param dt - Given discretization time.
   */
  void calculateInspectionSetpoint(double dt);

  /**
   * Return true if in inspection state, otherwise false.
   */
  bool inInspectionState();

  /**
   * Perform distance control. Set attitude setpoint according to the
   * current distance.
   */
  void doDistanceControl(double dt);

  /**
   * Callback function used for setting various parameters.
   */
  void distParamCb(uav_ros_control::DistanceControlParametersConfig &configMsg,
    uint32_t level);

  /**
   * Distance callback function.
   */
  void distanceCb(const std_msgs::Float64ConstPtr &message);

  /**
   * Distance setpoint offset callback
   */
  void distanceSpOffsetCb(const std_msgs::Float64ConstPtr &message);

  /**
   * Distance velocity callback function.
   */
  void distanceVelCb(const std_msgs::Float64ConstPtr &message);

  /**
   * Plane normal callback function.
   */
  void normalCb(const geometry_msgs::PoseStampedConstPtr &message);

  /**
   * Publish various distance control algorithm information.
   */
  void publishDistanceInfo();

private:
  /**
   * Publish current control state.
   */
  void publishState();

  /**
   * Publish distance setpoint as a std_msgs::Float64 message.
   */
  void publishDistSp();

  /**
   * Publish distance velocity setpoint as a Float64 ROS message.
   */
  void publishDistVelSp();

  /**
   * Publish distance to carrot.
   */
  void publishDistanceToCarrot();

  /**
   * Inspection enable service callback.
   */
  bool enableInspectionCb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response);

  /**
   * Set reconfigure parameters in the given config object.
   */
  void setDistReconfigureParams(uav_ros_control::DistanceControlParametersConfig &config);

  /**
   * Initialize parameters.
   */
  void initializeParameters(ros::NodeHandle &nh);

  /**
   * From the current Joy message determine if inspection if not.
   */
  bool inspectionEnabled();

  /**
   * Perform all necessary steps in order to deactivate inspection mode.
   */
  void deactivateInspection();

  /**
   * Returns true if inspection is requested, otherwise return false.
   */
  bool inspectionRequested();

  /**
   * Returns true if inspection failed, otherwise return false.
   */
  bool inspectionFailed();

  /**
   * Returns true if manual mode is requested, otherwise return false.
   */
  bool manualRequested();

  /** Current control state */
  DistanceControlState _currState;

  /** Distance PID controller */
  std::unique_ptr<PID> _distancePID;

  /** Distance velocity PID controller */
  std::unique_ptr<PID> _distanceVelPID;

  /** User enabled the inspection */
  bool _inspectionEnabled;

  /** True if inspection state was requested and denied, false otherwise. */
  bool _inspectionRequestFailed;

  /** Current distance measured value. Used both in sim and real mode. */
  double _distanceMeasured;

  /** Currently measured distance velocity. Used both in sim and real mode. */
  double _distanceVelocityMeasured;

  /** Distance setpoint value */
  double _distSp;

  /** Distance setpoint offset value*/
  double _distSpOffset;

  /** Distance velocity setpoint value. */
  double _distVelSp;

  /** Yaw of the plane normal with respect to UAV base frame. */
  double _planeYaw;

  /** Define all ROS subscribers. **/
  ros::Subscriber _subDistance;
  ros::Subscriber _subDistanceVelocity;
  ros::Subscriber _subPlaneNormal;
  ros::Subscriber _subDistSpOffset;

  /** Define all ROS publishers. **/
  ros::Publisher _pubControlState;
  ros::Publisher _pubDistanceSp;
  ros::Publisher _pubDistanceVelocitySp;
  ros::Publisher _pubCarrotDistance;

  /** Define all the services */
  ros::ServiceServer _serviceEnableInspection;

  /** Define Dynamic Reconfigure parameters **/
  boost::recursive_mutex _distConfigMutex;
  dynamic_reconfigure::Server<uav_ros_control::DistanceControlParametersConfig>
    _distConfigServer{ _distConfigMutex, ros::NodeHandle(DIST_DYN_RECONF) };
  dynamic_reconfigure::Server<
    uav_ros_control::DistanceControlParametersConfig>::CallbackType _distParamCallback;
};

/**
 * Run default Distance Control algorithm
 *
 * @param cc - Reference to CarrotControl object
 * @param nh - Given NodeHandle
 * @param simMode - true if simulation mode is enabled, otherwise false
 */
void runDefault(dist_control::DistanceControl &cc, ros::NodeHandle &nh);

}// namespace dist_control

#endif /* DISTANCE_CONTROL_H */