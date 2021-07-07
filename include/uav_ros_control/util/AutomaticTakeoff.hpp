#ifndef AUTOMATIC_TAKEOFF_HPP
#define AUTOMATIC_TAKEOFF_HPP

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

namespace uav_util {

class AutomaticTakeoff
{
public:
  const std::vector<int> CARROT_ACTIVATION_VECTOR = { 0, 0, 0, 0, 0, 1 };

  /**
   * @brief Construct a new Automatic Takeoff object.
   *
   * @param nh A ROS Node handle
   */
  explicit AutomaticTakeoff(ros::NodeHandle& nh);

  /**
   * @brief Try to takeoff the UAV.
   *
   * @param relative_altitude Relative takeoff altitude.
   * @param timeout_duration Takeoff attempt duration in seconds.
   * @param enable_carrot Publish CARROT_ACTIVATION_VECTOR on the /joy topic.
   * @return std::tuple<bool, std::string> True if takeoff is successful, otherwise false.
   */
  std::tuple<bool, std::string> tryTakeoff(double relative_altitude,
                                           double timeout_duration,
                                           bool   enable_carrot = true);

private:
  nav_msgs::Odometry m_curr_odom;
  ros::Subscriber    m_odom_sub;
  bool               m_odometry_recieved = false;
  void               odometry_callback(const nav_msgs::OdometryConstPtr& msg);

  std_msgs::String m_carrot_status;
  ros::Subscriber  m_carrot_sub;
  void             carrot_status_callback(const std_msgs::StringConstPtr& msg);

  mavros_msgs::State m_mavros_state;
  ros::Subscriber    m_mavros_state_sub;
  void               mavros_state_sub(const mavros_msgs::StateConstPtr& msg);

  ros::Publisher     m_joy_publisher;
  ros::ServiceClient m_set_mode_client;
  ros::ServiceClient m_arming_client;
  ros::ServiceClient m_takeoff_client;
};

}// namespace uav_util

#endif /* AUTOMATIC_TAKEOFF_HPP */