#ifndef MODEL_PREDICTIVE_CONTROL_HPP
#define MODEL_PREDICTIVE_CONTROL_HPP

#include <uav_ros_control/control/controller_interface.hpp>
#include <uav_ros_control/control/cvx_wrapper.hpp>
namespace uav_ros_control {
class ModelPredictiveControl : public controller_interface
{
public:
  
  /**
   * @brief Construct a new Model Predictive Control object.
   * 
   */
  ModelPredictiveControl();

  /* Override all pure virtual methods */
  void initialize(ros::NodeHandle &parent_nh,
    const std::string name,
    const double uav_mass) override;
  
  bool activate(const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd) override;
  
  void deactivate() override;
  
  const mavros_msgs::AttitudeTarget update(const nav_msgs::OdometryConstPtr &uav_state,
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd)
    override;

private:
  /* A space for private variables and methods */

  // Solver ptrs
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_x;
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_y;
  std::unique_ptr<uav_ros_control::cvx_wrapper::CvxWrapper> m_solver_z;

  bool m_is_active = false;
};
}// namespace uav_ros_control

#endif /* MODEL_PREDICTIVE_CONTROL_HPP */