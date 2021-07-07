#ifndef UAV_UTIL_HPP
#define UAV_UTIL_HPP

#include <ros/ros.h>

namespace uav_util {

/**
 * @brief A function that performs automatic takeoff if using the CarrotReferenceNode.
 *
 * @param nh
 * @param relative_altitude Relative takeoff altitude
 * @param enable_carrot Enable carrot mode through joy topic
 * @return true if takeoff successful
 * @return false if takeoff unsuccessful
 */
bool automatic_takeoff(
  ros::NodeHandle&   nh,
  double             relative_altitude,
  bool               enable_carrot  = true,
  const std::string& odometry_topic = "mavros/global_position/local");

}// namespace uav_util

#endif /* UAV_UTIL_HPP */