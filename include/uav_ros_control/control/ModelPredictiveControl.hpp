#ifndef MODEL_PREDICTIVE_CONTROL_HPP
#define MODEL_PREDICTIVE_CONTROL_HPP

#include <uav_ros_control/control/controller_interface.hpp>

namespace uav_ros_control {
class ModelPredictiveControl : public controller_interface
{
public:
  
  /* Override all pure virtual methods */
  void initialize(const ros::NodeHandle &parent_nh,
    const std::string name,
    const double uav_mass) override;
  
  bool activate(const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd) override;
  
  void deactivate() override;
  
  const mavros_msgs::AttitudeTarget update(const nav_msgs::OdometryConstPtr &uav_state,
    const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd)
    override;

private:
  /* A space for private variables and methods */
};
}// namespace uav_ros_control

#endif /* MODEL_PREDICTIVE_CONTROL_HPP */