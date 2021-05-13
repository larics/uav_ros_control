#include <uav_ros_control/control/controller_interface.hpp>
#include <pluginlib/class_loader.h>
#include <std_msgs/String.h>

using controller_interface_t =
  pluginlib::ClassLoader<uav_ros_control::controller_interface>;

int main(int argc, char** argv)
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
  const auto uav_mass        = 10.0;
  controller->initialize(nh_private, controller_name, uav_mass);

  // Carrot status subscriber
  bool is_controller_active = false;
  auto carrot_status        = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, [&](const std_msgs::StringConstPtr& msg) {
      // Case when controller needs to be activated
      if (!is_controller_active
          && (msg->data == "HOLD" || msg->data == "CARROT_ON_AIR")) {

        mavros_msgs::AttitudeTarget empty_command;
        empty_command.header.stamp = ros::Time::now();
        auto success               = controller->activate(
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
  auto traj_sub = nh.subscribe<trajectory_msgs::MultiDOFJointTrajectoryPoint>(
    "uav/trajectory_point",
    1,
    [&](const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr& msg) {
      traj_msg_ptr = msg;
    });

  // Odometry subscriber - this is where the magic happens
  auto att_target_pub =
    nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 1);
  auto odom_sub = nh.subscribe<nav_msgs::Odometry>(
    "odometry", 1, [&](const nav_msgs::OdometryConstPtr& msg) {
      if (!is_controller_active) {
        ROS_WARN_THROTTLE(5.0,
                          "[ControlManager] Odometry recieved but controller inactive.");
        return;
      }

      if (traj_msg_ptr) {
        const auto att_cmd = controller->update(msg, traj_msg_ptr);
        att_target_pub.publish(att_cmd);
        return;
      }

      // If trajectory point is not recieved set it to current odometry
      trajectory_msgs::MultiDOFJointTrajectoryPoint odom_traj_point;
      odom_traj_point.transforms =
        std::vector<geometry_msgs::Transform>{ geometry_msgs::Transform{} };
      odom_traj_point.velocities =
        std::vector<geometry_msgs::Twist>{ geometry_msgs::Twist{} };
      odom_traj_point.accelerations =
        std::vector<geometry_msgs::Twist>{ geometry_msgs::Twist{} };

      odom_traj_point.transforms.front().translation.x = msg->pose.pose.position.x;
      odom_traj_point.transforms.front().translation.y = msg->pose.pose.position.y;
      odom_traj_point.transforms.front().translation.z = msg->pose.pose.position.z;
      odom_traj_point.transforms.front().rotation      = msg->pose.pose.orientation;
      odom_traj_point.velocities.front()               = msg->twist.twist;

      ROS_WARN(
        "[ControlManager] Controller is active but trajecotry point is not recieved, "
        "this shouldn't happen! Setting trajectory point same as odometry.");

      auto odom_traj_ptr =
        boost::make_shared<const trajectory_msgs::MultiDOFJointTrajectoryPoint>(
          odom_traj_point);
      const auto att_cmd = controller->update(msg, odom_traj_ptr);
      att_target_pub.publish(att_cmd);
    });

  ros::spin();
  return 0;
}