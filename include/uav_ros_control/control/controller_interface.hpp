#ifndef CONTROLLER_INTERFACE_HPP
#define CONTROLLER_INTERFACE_HPP

#include <ros/ros.h>

#include <mavros_msgs/AttitudeTarget.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <nav_msgs/Odometry.h>

namespace uav_ros_control {

// parameters of the propulsion thrust curve
// T = A*sqrt(F) + B
// T is within [0, 1]
// F is in Newtons
struct MotorParams
{
  double A;
  double B;
};

class controller_interface
{
public:
  virtual ~controller_interface() = 0;

  /**
   * @brief This method initializes the controller. It is called once for every
   * controller. The runtime is not limited.
   *
   * @param parent_nh the node handle of the parent
   * @param name of the controller for distinguishing multiple running instances of the
   * same code
   * @param name_space the parameter namespace of the controller, can be used during
   * initialization of the private node handle
   * @param uav_mass the net mass of the UAV
   */
  virtual void initialize(const ros::NodeHandle &parent_nh,
    const std::string name,
    const double uav_mass) = 0;

  /**
   * @brief This method is called before the controller output will be required and used.
   * Should not take much time (within miliseconds).
   *
   * @param last_attitude_cmd the last command produced by the last active controller.
   * Should be used as an initial condition, e.g., for re-initializing integrators and
   * estimators.
   *
   * @return true if success
   */
  virtual bool activate(const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd) = 0;

  /**
   * @brief is called when this controller's output is no longer needed. However, it can
   * be activated later.
   */
  virtual void deactivate(void) = 0;

  /**
   * @brief The most important routine. It is called with every odometry update and it
   * should produce a new control command.
   *
   * @param uav_state the latest UAV state estimate
   * @param last_position_cmd the last controller's output command (may be useful)
   *
   * @return the new reference for the controllers
   */
  virtual const mavros_msgs::AttitudeTarget update(
    const nav_msgs::OdometryConstPtr &uav_state,
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd) = 0;
};

// A pure virtual destructor requires a function body.
controller_interface::~controller_interface(){};

}// namespace uav_ros_control

#endif /* CONTROLLER_INTERFACE_HPP */