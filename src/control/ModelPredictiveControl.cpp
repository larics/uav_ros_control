#include <uav_ros_control/control/ModelPredictiveControl.hpp>

uav_ros_control::ModelPredictiveControl::ModelPredictiveControl()
{
  // TODO:
}

void uav_ros_control::ModelPredictiveControl::initialize(const ros::NodeHandle &parent_nh,
  const std::string name,
  const double uav_mass)
{
  // TODO:
}

bool uav_ros_control::ModelPredictiveControl::activate(
  const mavros_msgs::AttitudeTargetConstPtr &last_attitude_cmd)
{
  ROS_INFO("ModelPredictiveControl::activate");
  // TODO:
  return false;
}

void uav_ros_control::ModelPredictiveControl::deactivate()
{
  ROS_INFO("ModelPredictiveControl::deactivate");
  // TODO:
}

const mavros_msgs::AttitudeTarget uav_ros_control::ModelPredictiveControl::update(
  const nav_msgs::OdometryConstPtr &uav_state,
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &last_position_cmd)
{
  ROS_INFO_THROTTLE(5.0, "ModelPredictiveControl::update");
  // TODO:
  return {};
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(uav_ros_control::ModelPredictiveControl,
  uav_ros_control::controller_interface);