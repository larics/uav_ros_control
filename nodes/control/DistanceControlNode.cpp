#include <uav_ros_control/control/DistanceController.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_control_node");
  ros::NodeHandle nh;

  // Initialize distance control object
  std::shared_ptr<dist_control::DistanceControl> distCtlRefObj{
    new dist_control::DistanceControl(nh)
  };

  dist_control::runDefault(*distCtlRefObj, nh);
}