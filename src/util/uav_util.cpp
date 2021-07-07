#include <uav_ros_control/util/uav_util.hpp>

#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <uav_ros_msgs/TakeOff.h>
#include <std_msgs/String.h>

bool uav_util::automatic_takeoff(ros::NodeHandle& nh,
                                 double           relative_altitude,
                                 bool             enable_carrot)
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
  auto takeoff_client = nh.serviceClient<uav_ros_msgs::TakeOff>("takeoff");
  if (!takeoff_client.exists()) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - " << takeoff_client.getService()
                                                       << " does not exist.");
    return false;
  }

  // Initialize Joy publisher
  auto joy_pub = nh.advertise<sensor_msgs::Joy>("joy", 1);

  // Initialize carrot status subscriber
  std::string carrot_status{ "n/a" };
  auto        carrot_status_sub = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, [&](const std_msgs::StringConstPtr& msg) {
      carrot_status = msg->data;
    });

  // Initialize mavros state subscriber
  bool is_armed         = false;
  auto mavros_state_sub = nh.subscribe<mavros_msgs::State>(
    "mavros/state", 1, [&](const mavros_msgs::StateConstPtr& msg) {
      is_armed = msg->armed;
    });

  const auto shutdown_connections = [&]() {
    // Shutdown all clients
    set_mode_client.shutdown();
    arming_client.shutdown();
    takeoff_client.shutdown();

    // Shutdown pub
    joy_pub.shutdown();

    // Shutdown sub
    carrot_status_sub.shutdown();
    mavros_state_sub.shutdown();
  };

  // start automatic takeoff
  ROS_INFO("[uav_util::automatic_takeoff] - started");
  ros::Duration wait(2.0);

  // Swith to GUIDED mode
  mavros_msgs::SetMode guided_request;
  guided_request.request.custom_mode = "GUIDED";
  guided_request.request.base_mode   = 0;
  auto call_success                  = set_mode_client.call(guided_request);
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

  int fail_counter = 0;


  // Wait loop for carrot reference activation
  while (ros::ok()) {
    ros::spinOnce();
    if (carrot_status == "CARROT_ON_LAND") { break; }
    fail_counter++;
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - failed to activate CarrotReference ["
                    << fail_counter << "/10]");
    if (fail_counter >= 10) {
      shutdown_connections();
      return false;
    }
    wait.sleep();
  }

  // Arm the UAV
  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;
  call_success              = arming_client.call(arm_command);
  if (!call_success) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - call to "
                    << arming_client.getService() << " service failed.");
    shutdown_connections();
    return false;
  }

  // Wait for armed


  fail_counter = 0;
  while (!is_armed && ros::ok()) {
    ros::spinOnce();
    fail_counter++;
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - failed to arm the UAV ["
                    << fail_counter << "/10]");
    if (fail_counter >= 10) {
      shutdown_connections();
      return false;
    }
    wait.sleep();
  }

  uav_ros_msgs::TakeOff takeoff_request;
  takeoff_request.request.rel_alt = relative_altitude;
  call_success                    = takeoff_client.call(takeoff_request);
  if (!call_success) {
    ROS_WARN("[uav_util::automatic_takeoff] - call to takeoff service unsuccessful.");
    shutdown_connections();
    return false;
  }
  if (!takeoff_request.response.success) {
    ROS_WARN_STREAM("[uav_util::automatic_takeoff] - takeoff failed with response: "
                    << takeoff_request.response);
    shutdown_connections();
    return false;
  }

  ROS_INFO("[uav_util::automatic_takeoff] - takeoff successful!.");
  shutdown_connections();
  return true;
}