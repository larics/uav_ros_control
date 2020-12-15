#ifndef CONTROL_BASE_H
#define CONTROL_BASE_H

// ROS includes
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

// Cpp includes
#include <array>

namespace uav_controller {

/** Type mask used when publishing AttitudeTarget ROS messages. It will ignore
 * body roll, pitch and rate values. */
const int MASK_IGNORE_RPY_RATE = 7;

/** Type mask used when publishing AttitudeTarget ROS messages. It will ignore
 * body roll and pitch rate values. */
const int MASK_IGNORE_RP_RATE = 3;

/**
 * This class is used for defining UAV odometry subscribers.
 * It will also define attitude an thrust setpoint publisher.
 *
 * This class is meant to be used as a base class for some other control strategy.
 */
class ControlBase
{
public:
  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   *
   * @param nh - ROS node handle, used for initializing subscribers and publishers
   */
  explicit ControlBase(ros::NodeHandle &nh);
  virtual ~ControlBase() = default;
  

  /**
   * Set roll, pitch, yaw attitude setpoint.
   *
   * @param roll 	- Roll angle setpoint
   * @param pitch - Pitch angle setpoint
   * @param yaw 	- Yaw angle setpoint
   */
  void setAttitudeSp(double roll, double pitch, double yaw);

  /**
   * Set thrust setpoint.
   *
   * @param thrust - New thrust setpoint
   */
  void setThrustSp(double thrust);

  /**
   * Returns constant reference to the current position.
   */
  const std::array<double, 3> &getCurrPosition();

  /**
   * Returns constant reference to the current velocity.
   */
  const std::array<double, 3> &getCurrVelocity();

  /**
   * Return current UAV yaw angle.
   */
  double getCurrentYaw();

  /**
   * Publish attitude setpoiint.
   *
   * @typeMask - AttitudeTarget bitmask
   * @yawRate - Alse set yawRate if bitmask enables it
   */
  void publishAttitudeTarget(int typeMask, double yawRate = 0);

  /**
   * Publish setpoint euler angles as a Vector3 ROS message.
   */
  void publishEulerSp();

  /**
   * Get trajectory point reference.
   */
  const trajectory_msgs::MultiDOFJointTrajectoryPoint &getCurrentReference();

  void overridePitchTarget(double newPitch);
  void overrideRollTarget(double newRoll);
  void overrideYawTarget(double newYaw);

private:
  /**
   * Odometry callback function.
   */
  void odomCb(const nav_msgs::OdometryConstPtr &);

  /**
   * MSF odometry callback function
   */
  void msfOdomCb(const nav_msgs::OdometryConstPtr &);

  /**
   * Trajectory point callback function.
   */
  void trajPointCb(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &);

  /** Current UAV yaw */
  double _currentYaw = 0;

  /** Current local position vector. */
  std::array<double, 3> _currentPosition{ 0.0, 0.0, 0.0 };

  /** Current local velocity vector. */
  std::array<double, 3> _currentVelocity{ 0.0, 0.0, 0.0 };

  /** Attitude setpoint array. Attitude is considered in RPY. Thrust is in [0, 1] range.
   */
  std::array<double, 4> _attThrustSp{ 0.0, 0.0, 0.0, 0.0 };

  /** Current carrot reference */
  trajectory_msgs::MultiDOFJointTrajectoryPoint _currentReference;

  /** Declare all subscribers **/
  ros::Subscriber _subOdom;
  ros::Subscriber _subReference;

  /** Declare all publishers. **/
  ros::Publisher _pubEulerSetpoint;
  ros::Publisher _pubAttitudeSetpoint;
};

}// namespace uav_controller

#endif /* CONTROL_BASE_H */
