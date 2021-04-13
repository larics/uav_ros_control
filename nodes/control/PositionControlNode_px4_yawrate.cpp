#include <uav_ros_control/control/CascadePID.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "position_control_node");
  ros::NodeHandle nh;

  // Initialize distance control object
  std::shared_ptr<uav_controller::CascadePID> carrotRefObj{
    new uav_controller::CascadePID(nh)
  };

  uav_controller::runDefault_yawrate_px4(*carrotRefObj, nh);
}