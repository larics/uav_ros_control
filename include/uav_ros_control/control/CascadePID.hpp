#ifndef CASCADE_PID_H
#define CASCADE_PID_H

#include <uav_ros_control/control/ControlBase.hpp>
#include <uav_ros_control/control/PID.hpp>
#include <dynamic_reconfigure/server.h>
#include <uav_ros_control/PositionControlParametersConfig.h>

#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>

namespace uav_controller {
/** Name of dynamic reconfigure node. */
#define CASCADE_DYN_RECONF "cascade_config"

/**
 * PID cascade controller.
 */
class CascadePID : public ControlBase
{
public:
  /**
   * Default constructor. Used for reading ROS parameters and initalizing
   * private variables.
   */
  explicit CascadePID(ros::NodeHandle &nh);
  ~CascadePID() = default;

  /**
   * Calculate new attitude and thrust setpoint.
   *
   * @oaram dt - Discretization time
   */
  void calculateAttThrustSp(double dt);
  bool activationPermission();

  /**
   * Reset all position PIDs.
   */
  void resetPositionPID();

  /**
   * Reset all velociy PIDs.
   */
  void resetVelocityPID();

private:
  /**
   * Reset integrator service callback.OS
   */
  bool intResetServiceCb(std_srvs::Empty::Request &request,
    std_srvs::Empty::Response &response);

  /**
   * Yaw reference callback function.
   */
  void yawRefCb(const std_msgs::Float64ConstPtr &);
  void carrotStatusCb(const std_msgs::StringConstPtr &);

  /**
   * Do all the parameter initialization here.
   */
  void initializeParameters(ros::NodeHandle &nh);

  /**
   * Callback function used for setting various parameters.
   */
  void positionParamsCb(uav_ros_control::PositionControlParametersConfig &configMsg,
    uint32_t level);

  /**
   * Set reconfigure parameters in the given config object.
   */
  void setPositionReconfigureParams(
    uav_ros_control::PositionControlParametersConfig &config);

  /** PID controller for position along the y-axis.*/
  std::unique_ptr<PID> _posYPID;

  /** PID controller for velocity along the y-axis */
  std::unique_ptr<PID> _velYPID;

  /** PID controller for position along the x-axis.*/
  std::unique_ptr<PID> _posXPID;

  /** PID controller for velocity along the x-axis */
  std::unique_ptr<PID> _velXPID;

  /** PID controller for position along the z-axis.*/
  std::unique_ptr<PID> _posZPID;

  /** PID controller for velocity along the y-axis */
  std::unique_ptr<PID> _velZPID;

  /** Value from 0 to 1, hover thrust */
  double _hoverThrust = 0;

  /** Feed-forward gain for linear velocity reference. */
  double _ffGainVelocityX = 0, _ffGainVelocityY = 0, _ffGainVelocityZ = 0;

  /** Feed-forward gain for linear acceleration reference. */
  double _ffGainAccelerationX = 0, _ffGainAccelerationY = 0, _ffGainAccelerationZ = 0;

  /** Yaw reference */
  double _yawRef = 0;

  /** Define Dynamic Reconfigure parameters **/
  boost::recursive_mutex _posConfigMutex;
  dynamic_reconfigure::Server<uav_ros_control::PositionControlParametersConfig>
    _posConfigServer{ _posConfigMutex, ros::NodeHandle(CASCADE_DYN_RECONF) };
  dynamic_reconfigure::Server<
    uav_ros_control::PositionControlParametersConfig>::CallbackType _posParamCallback;

  /** Define all the services */
  ros::ServiceServer _serviceResetIntegrators;

  /** Velocity ref publisher */
  ros::Publisher _velRefPub;
  ros::Publisher _velCurrPub;

  /** Yaw reference subscriber. */
  ros::Subscriber _yawRefSub;
  ros::Subscriber _carrotStateSub;
  std::string _carrotStatus;
};

/**
 * Default position control program
 */
void runDefault(uav_controller::CascadePID &cc, ros::NodeHandle &nh);
}// namespace uav_controller

#endif /** CASCADE_PID_H */