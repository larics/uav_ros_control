#include "uav_ros_control/control/FlightInit.hpp"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flight_init_node");
  ros::NodeHandle nh;

  std::shared_ptr<flight_init::FlightInit> fiObj{ new flight_init::FlightInit(nh) };
  fiObj->run();
}
