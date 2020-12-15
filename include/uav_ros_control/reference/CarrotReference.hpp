#ifndef CARROT_REFERENCE_H
#define CARROT_REFERENCE_H

#include <uav_ros_control/reference/JoyControlInput.hpp>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <uav_ros_control_msgs/TakeOff.h>
#include <mavros_msgs/State.h>
#include <uav_ros_control/filters/Util.hpp>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

namespace uav_reference {
/**
 * "Carrot-on-a-Stick" reference publisher class. Reference is published
 * with respect to the global coordinate system.
 */
class CarrotReference : public uav_reference::JoyControlInput
{
public:
  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   */
  CarrotReference(ros::NodeHandle &);
  virtual ~CarrotReference();

  /**
   * Update carrot position setpoint, with default joystick offset values.
   * Also update the yaw reference value.
   */
  void updateCarrot();

  /**
   * Publish carrot position setpoint as a Vector3 ROS message.
   */
  void publishCarrotSetpoint();

  /**
   * Check if carrot mode is entered. This method will reset carrot position
   * when entering carrot reference mode for the first time.
   */
  void updateCarrotStatus();

  /**
   * True if carrot is enabled otherwise false.
   */
  bool isCarrotEnabled();

  /**
   * True if position hold is enabled, otherwise false.
   */
  bool isHoldEnabled();

private:
  void resetIntegrators();

  /**
   * Initialize class parameters.
   */
  void initializeParameters();


  /**
   * Update x and y component of carrot position setpoint with the given value.
   *
   * @param - x carrot offset
   * @param - y carrot offset
   */
  void updateCarrotXY(double x, double y);

  /**
   * Update x component of carrot position setpoint with default joystick offset
   * value.
   */
  void updateCarrotXY();

  /**
   * Update z component of carrot position setpoint with the given value.
   *
   * @param - z carrot offset
   */
  void updateCarrotZ(double z);

  /**
   * Update z component of carrot position setpoint with default joystick offset
   * value.
   */
  void updateCarrotZ();

  /**
   * Update carrot Yaw component
   */
  void updateCarrotYaw();

  /**
   * Reset carrot trajectory point.
   */
  void resetCarrot();

  /**
   * Position hold service callback.OS
   */
  bool posHoldServiceCb(std_srvs::Empty::Request &request,
    std_srvs::Empty::Response &response);

  bool takeoffServiceCb(uav_ros_control_msgs::TakeOff::Request &request,
    uav_ros_control_msgs::TakeOff::Response &response);

  bool landServiceCb(std_srvs::SetBool::Request &request,
    std_srvs::SetBool::Response &response);

  /**
   * Callback function for Position reference. Works only during position hold mode.
   */
  void positionRefCb(const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &posMsg);

  /**
   * Odometry callback function. Used for extracting UAV yaw rotation.
   */
  void odomCb(const nav_msgs::OdometryConstPtr &);

  /** Carrot reference used for position hold. */
  trajectory_msgs::MultiDOFJointTrajectoryPoint _carrotPoint;

  /** UAV current position array. */
  std::array<double, 3> _uavPos{ 0.0, 0.0, 0.0 };

  /** Referent yaw angle. */
  double _carrotYaw = 0;

  /** Current UAV yaw angle */
  double _uavYaw = 0;

  /** True if carrot mode is enabled otherwise false */
  bool _carrotEnabled = false;

  /** True if position hold mode is enabled, otherwise false */
  bool _positionHold = true;

  /** Index used for enabling carrot mode */
  int _carrotEnabledIndex = -1;

  /** Carrot enable button value, 0 or 1 **/
  int _carrotEnabledValue = 1;

  /* First pass flag - set carrot to odometry */
  bool _firstPass = true;
  bool _manualTakeoffEnabled = true;
  bool _carrotOnLand = false, _takeoffHappened = false;

  /** Define all Publishers */
  ros::Publisher _pubCarrotTrajectorySp;
  ros::Publisher _pubCarrotYawSp;
  ros::Publisher _pubUAVYaw;
  ros::Publisher _pubCarrotPose;
  ros::Publisher _pubCarrotStatus;

  /** Define all Subscribers. */
  ros::Subscriber _subOdom;
  ros::Subscriber _subPosHoldRef;
  ros_util::TopicHandler<mavros_msgs::State> m_handlerState;

  /** Define all the services */
  ros::ServiceServer _servicePoisitionHold, _serviceTakeoff, _serviceLand;

  /* Reset integrator client */
  ros::ServiceClient _intResetClient, _setModeToLandClient;
};

/**
 * Run default Carrot Reference publishing node program.
 *
 * @param cc - Reference to CarrotReference object
 * @param nh - Given NodeHandle
 */
void runDefault(uav_reference::CarrotReference &cc, ros::NodeHandle &nh);
}// namespace uav_reference

#endif /** CARROT_REFERENCE_H */