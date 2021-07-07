#include <uav_ros_control/util/AutomaticTakeoff.hpp>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <sensor_msgs/Joy.h>
#include <uav_ros_msgs/TakeOff.h>

using namespace uav_util;

AutomaticTakeoff::AutomaticTakeoff(ros::NodeHandle& nh)
{
  m_set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  m_arming_client   = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  m_joy_publisher   = nh.advertise<sensor_msgs::Joy>("joy", 1);
  m_takeoff_client  = nh.serviceClient<uav_ros_msgs::TakeOff>("takeoff");
  m_odom_sub        = nh.subscribe<nav_msgs::Odometry>(
    "odometry", 1, &AutomaticTakeoff::odometry_callback, this);
  m_carrot_sub = nh.subscribe<std_msgs::String>(
    "carrot/status", 1, &AutomaticTakeoff::carrot_status_callback, this);
  m_mavros_state_sub = nh.subscribe<mavros_msgs::State>(
    "mavros/state", 1, &AutomaticTakeoff::mavros_state_sub, this);
}

void AutomaticTakeoff::odometry_callback(const nav_msgs::OdometryConstPtr& msg)
{
  m_odometry_recieved = true;
  m_curr_odom         = *msg;
}
void AutomaticTakeoff::carrot_status_callback(const std_msgs::StringConstPtr& msg)
{
  m_carrot_status = *msg;
}

void AutomaticTakeoff::mavros_state_sub(const mavros_msgs::StateConstPtr& msg)
{
  m_mavros_state = *msg;
}

std::tuple<bool, std::string> AutomaticTakeoff::tryTakeoff(double relative_altitude,
                                                           double timeout_duration,
                                                           bool   enable_carrot)
{
  ros::Time  start_time         = ros::Time::now();
  const auto check_elapsed_time = [&](const std::string& status) {
    if ((ros::Time::now() - start_time).toSec() > timeout_duration) {
      return std::make_tuple<bool, std::string>(
        false, "AutomaticTakeoff::tryTakeoff - timeout duration elapsed while " + status);
    }
    return std::make_tuple<bool, std::string>(true, "All is well.");
  };

  // Check for set_mode service
  if (!m_set_mode_client.exists()) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - " + m_set_mode_client.getService()
               + " does not exist." };
  }

  // Check for arming service
  if (!m_arming_client.exists()) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - " + m_arming_client.getService()
               + " does not exist." };
  }

  // Check for takeoff service
  if (!m_takeoff_client.exists()) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - " + m_takeoff_client.getService()
               + " does not exist. " };
  }

  if (!m_odometry_recieved) {
    return { false,
             "AutomaticTakeoff:tryTakeoff - odometry not recieved. "
               + m_odom_sub.getTopic() };
  }

  if (m_mavros_state.armed) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - UAV is already armed. Takeoff aborted." };
  }

  // Swith to GUIDED mode
  mavros_msgs::SetMode guided_request;
  guided_request.request.custom_mode = "GUIDED_NOGPS";
  guided_request.request.base_mode   = 0;
  auto call_success                  = m_set_mode_client.call(guided_request);
  if (!call_success || !guided_request.response.mode_sent) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - unable to switch to GUIDED_NOGPS mode." };
  }

  // Check if we're actually in GUIDED_NOGPS mode
  ros::Duration wait(0.01);
  while (ros::ok()) {
    wait.sleep();
    ros::spinOnce();

    if (m_mavros_state.mode == "GUIDED_NOGPS") { break; }
    auto status = check_elapsed_time("while waiting for GUIDED_NOGPS.");
    if (!std::get<0>(status)) { return status; }
  }

  // Activate CarrotReference
  if (enable_carrot) {
    sensor_msgs::Joy joy_msg;
    joy_msg.buttons = CARROT_ACTIVATION_VECTOR;
    m_joy_publisher.publish(joy_msg);
  }

  // Check if carrot is activated
  while (ros::ok()) {
    wait.sleep();
    ros::spinOnce();

    if (m_carrot_status.data == "CARROT_ON_LAND") { break; }
    auto status = check_elapsed_time("while waiting for carrot.");
    if (!std::get<0>(status)) { return status; }
  }

  // Arm the UAV
  mavros_msgs::CommandBool arm_command;
  arm_command.request.value = true;
  call_success              = m_arming_client.call(arm_command);
  if (!call_success) {
    return { false, "AutomaticTakeoff::tryTakeoff - call to arm service unsuccessful." };
  }

  // Check if the UAV is actually armed.
  while (ros::ok()) {
    wait.sleep();
    ros::spinOnce();

    if (m_mavros_state.armed) { break; }
    auto status = check_elapsed_time("while waiting for UAV arm.");
    if (!std::get<0>(status)) { return status; }
  }

  uav_ros_msgs::TakeOff takeoff_request;
  takeoff_request.request.rel_alt = relative_altitude;
  call_success                    = m_takeoff_client.call(takeoff_request);
  if (!call_success) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - call to takeoff service unsuccessful." };
  }
  if (!takeoff_request.response.success) {
    return { false,
             "AutomaticTakeoff::tryTakeoff - takeoff failed with response: "
               + takeoff_request.response.message };
  }

  // Check if the UAV is actually armed.
  while (ros::ok()) {
    wait.sleep();
    ros::spinOnce();

    if (abs(m_curr_odom.pose.pose.position.z - relative_altitude) < 0.5) { break; }
    auto status = check_elapsed_time("while waiting for UAV to takeoff.");
    if (!std::get<0>(status)) { return status; }
  }

  return { true,
           "AutomaticTakeoff::tryTakeoff - UAV takeoff successful. Currently at height "
             + std::to_string(m_curr_odom.pose.pose.position.z) };
}