/*
 * ControlBase.cpp
 *
 *  Created on: Apr 11, 2019
 *      Author: lmark
 */
#include <uav_ros_control/control/ControlBase.hpp>
#include <uav_ros_control/filters/NonlinearFilters.hpp>

#include <mavros_msgs/AttitudeTarget.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>

// Cpp includes
#include <iostream>
#include <vector>
#include <array>
#include <cmath>

uav_controller::ControlBase::ControlBase(ros::NodeHandle &nh)
{
  ros::NodeHandle nhPrivate("~");
  bool msf_callback = false;
  bool initialized = nhPrivate.getParam("msf_callback", msf_callback);
  if (!initialized) {
    ROS_FATAL("ControlBase - parameters not loaded.");
    throw std::runtime_error("Parameters not loaded");
  }
  auto odomCallback = &uav_controller::ControlBase::odomCb;
  if (msf_callback) {
    odomCallback = &uav_controller::ControlBase::msfOdomCb;
  }

  // Initialize all subscribers
  _subOdom = nh.subscribe("odometry", 1, odomCallback, this);
  _subReference = nh.subscribe(
    "uav/trajectory_point", 1, &uav_controller::ControlBase::trajPointCb, this);

  // Initialized all publishers
  _pubAttitudeSetpoint =
    nh.advertise<mavros_msgs::AttitudeTarget>("uav/attitude_target", 1);
  _pubEulerSetpoint = nh.advertise<geometry_msgs::Vector3>("uav/euler_setpoint", 1);

  // Initialize references
  _currentReference.transforms = std::vector<geometry_msgs::Transform>(1);
  _currentReference.velocities = std::vector<geometry_msgs::Twist>(1);
  _currentReference.accelerations = std::vector<geometry_msgs::Twist>(1);
}

void uav_controller::ControlBase::odomCb(const nav_msgs::OdometryConstPtr &message)
{
  _currentPosition[0] = message->pose.pose.position.x;
  _currentPosition[1] = message->pose.pose.position.y;
  _currentPosition[2] = message->pose.pose.position.z;

  _currentVelocity[0] = message->twist.twist.linear.x;
  _currentVelocity[1] = message->twist.twist.linear.y;
  _currentVelocity[2] = -message->twist.twist.linear.z;

  _currentYaw = util::calculateYaw(message->pose.pose.orientation.x,
    message->pose.pose.orientation.y,
    message->pose.pose.orientation.z,
    message->pose.pose.orientation.w);
}

void uav_controller::ControlBase::msfOdomCb(const nav_msgs::OdometryConstPtr &message)
{
  _currentPosition[0] = message->pose.pose.position.x;
  _currentPosition[1] = message->pose.pose.position.y;
  _currentPosition[2] = message->pose.pose.position.z;

  _currentVelocity[0] = cos(-_currentYaw) * message->twist.twist.linear.x
                        + sin(-_currentYaw) * message->twist.twist.linear.y;
  _currentVelocity[1] = cos(-_currentYaw) * message->twist.twist.linear.y
                        - sin(-_currentYaw) * message->twist.twist.linear.x;
  _currentVelocity[2] = message->twist.twist.linear.z;

  _currentYaw = util::calculateYaw(message->pose.pose.orientation.x,
    message->pose.pose.orientation.y,
    message->pose.pose.orientation.z,
    message->pose.pose.orientation.w);
}

void uav_controller::ControlBase::trajPointCb(
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &msg)
{
  if (msg->transforms.empty() || msg->velocities.empty() || msg->accelerations.empty()) {
    ROS_FATAL("ControlBase::trajPointCb - Trajectory point incomplete.");
    return;
  }

  _currentReference.transforms[0] = msg->transforms[0];
  _currentReference.velocities[0] = msg->velocities[0];
  _currentReference.accelerations[0] = msg->accelerations[0];
}

const std::array<double, 3> &uav_controller::ControlBase::getCurrPosition()
{
  return _currentPosition;
}

const std::array<double, 3> &uav_controller::ControlBase::getCurrVelocity()
{
  return _currentVelocity;
}

void uav_controller::ControlBase::publishAttitudeTarget(int typeMask, double yawRate)
{
  tf2::Quaternion q;
  q.setRPY(_attThrustSp[0], _attThrustSp[1], _attThrustSp[2]);
  
  mavros_msgs::AttitudeTarget newMessage;
  newMessage.header.stamp = ros::Time::now();
  newMessage.type_mask = typeMask;
  newMessage.body_rate.z = yawRate;
  newMessage.orientation.x = q.getX();
  newMessage.orientation.y = q.getY();
  newMessage.orientation.z = q.getZ();
  newMessage.orientation.w = q.getW();
  newMessage.thrust = _attThrustSp[3];
  _pubAttitudeSetpoint.publish(newMessage);
}

void uav_controller::ControlBase::setThrustSp(const double thrust)
{
  _attThrustSp[3] = thrust;
}

void uav_controller::ControlBase::setAttitudeSp(const double roll,
  const double pitch,
  const double yaw)
{
  _attThrustSp[0] = roll;
  _attThrustSp[1] = pitch;
  _attThrustSp[2] = yaw;
}

void uav_controller::ControlBase::overrideRollTarget(const double roll)
{
  _attThrustSp[0] = roll;
}

void uav_controller::ControlBase::overridePitchTarget(const double pitch)
{
  _attThrustSp[1] = pitch;
}

void uav_controller::ControlBase::overrideYawTarget(const double yaw)
{
  _attThrustSp[2] = yaw;
}

void uav_controller::ControlBase::publishEulerSp()
{
  geometry_msgs::Vector3 newMessage;
  newMessage.x = _attThrustSp[0];
  newMessage.y = _attThrustSp[1];
  newMessage.z = _attThrustSp[2];
  _pubEulerSetpoint.publish(newMessage);
}

const trajectory_msgs::MultiDOFJointTrajectoryPoint &
  uav_controller::ControlBase::getCurrentReference()
{
  return _currentReference;
}

double uav_controller::ControlBase::getCurrentYaw() { return _currentYaw; }
