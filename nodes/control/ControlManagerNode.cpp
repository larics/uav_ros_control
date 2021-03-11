#include <uav_ros_control/control/controller_interface.hpp>
#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

using controller_interface_t =
  pluginlib::ClassLoader<uav_ros_control::controller_interface>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_manager");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Load detector plugin
  auto controller_plugin_loader = std::make_unique<controller_interface_t>(
    "uav_ros_control", "uav_ros_control::controller_interface");
  auto controller =
    controller_plugin_loader->createUniqueInstance("ModelPredictiveControl");

  // TODO(lmarK): Parametrize these variables
  const auto controller_name = "my_first_mpc";
  const auto uav_mass = 10.0;
  controller->initialize(nh_private, controller_name, uav_mass);

  // Carrot status subscriber
  bool is_controller_active = false;
  auto carrot_status = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, [&](const std_msgs::StringConstPtr &msg) {
      // Case when controller needs to be activated
      if (!is_controller_active
          && (msg->data == "HOLD" || msg->data == "CARROT_ON_AIR")) {

        mavros_msgs::AttitudeTarget empty_command;
        empty_command.header.stamp = ros::Time::now();
        auto success = controller->activate(
          boost::make_shared<mavros_msgs::AttitudeTarget>(empty_command));
        is_controller_active = success;
        return;
      }

      // Case when controller is already activated and might need to be deactivated
      if (is_controller_active
          && !((msg->data == "HOLD" || msg->data == "CARROT_ON_AIR"))) {
        controller->deactivate();
        is_controller_active = false;
        return;
      }
    });

  // Trajectory point subscriber
  trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr traj_msg_ptr;
  auto traj_sub =
    nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>("uav/trajectory_point",
      1,
      [&](const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &msg) {
        traj_msg_ptr = msg;
      });

  // Odometry subscriber - this is where the magic happens
  auto att_target_pub =
    nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
    "odometry", 1, [&](const nav_msgs::OdometryConstPtr &msg) {
      if (is_controller_active) {
        const auto att_cmd = controller->update(msg, traj_msg_ptr);
        att_target_pub.publish(att_cmd);
      }
    });
  
  ros::spin();
  return 0;
}