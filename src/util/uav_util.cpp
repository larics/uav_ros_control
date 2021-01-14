#include <uav_ros_control/util/uav_util.hpp>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <uav_ros_msgs/TakeOff.h>

bool uav_util::automatic_takeoff(ros::NodeHandle &nh,
  double relative_altitude,
  bool enable_carrot)
{
  // Initialize mavros/set_mode client
  auto set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  if (!set_mode_client.exists()) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - " << set_mode_client.getService()
                                                       << " does not exist.");
    return false;
  }

  // Initialize mavros/arming client
  auto arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  if (!arming_client.exists()) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - " << arming_client.getService()
                                                       << " does not exist.");
    return false;
  }

  // Initialize takeoff client
  auto takeoff_client = nh.serviceClient<std_srvs::Empty>("takeoff");
  if (!takeoff_client.exists()) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - " << takeoff_client.getService()
                                                       << " does not exist.");
    return false;
  }

  // Initialize Joy publisher
  auto joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 1);

  // start automatic takeoff
  ROS_INFO("[uav_util::automatic_takeoff] - started");
  ros::Duration wait(1.0);

  // Swith to GUIDED mode
  mavros_msgs::SetMode guided_request;
  guided_request.request.custom_mode = "GUIDED";
  guided_request.request.base_mode = 0;
  auto call_success = set_mode_client.call(guided_request);
  if (!call_success || !guided_request.response.mode_sent) {
    ROS_WARN("[uav_util::automatic_takeoff] - unable to switch to GUIDED.");
    return false;
  }

  wait.sleep();

  if (enable_carrot) {
    // Enable CARROT_ON_LAND
    sensor_msgs::Joy enable_carrot_msg;
    enable_carrot_msg.buttons = std::vector<int>{ 0, 0, 0, 0, 0, 1 };
    joy_pub.publish(enable_carrot_msg);

    wait.sleep();
  }

  // Arm the UAV
  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;
  call_success = arming_client.call(arm_command);
  if (!call_success) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - call to "
                    << arming_client.getService() << " service failed.");
    return false;
  }

  wait.sleep();

  uav_ros_msgs::TakeOff takeoff_request;
  takeoff_request.request.rel_alt = 2;
  call_success = takeoff_client.call(takeoff_request);
  if (!call_success) {
    ROS_WARN("[uav_util::automatic_takeoff] - call to takeoff service unsuccessful.");
    return false;
  }
  if (!takeoff_request.response.success) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - takeoff failed with response: "
                    << takeoff_request.response);
    return false;
  }

  ROS_INFO("[uav_util::automatic_takeoff] - takeoff successful!.");
  // Shutdown all clients
  set_mode_client.shutdown();
  arming_client.shutdown();
  takeoff_client.shutdown();

  // Shutdown pub
  joy_pub.shutdown();

  return false;
}