#include <uav_ros_control/reference/CarrotReference.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "carrot_reference_node");
  ros::NodeHandle nh;

  // Initialize distance control object
  std::shared_ptr<uav_reference::CarrotReference> carrotRefObj{
    new uav_reference::CarrotReference(nh)
  };

  uav_reference::runDefault(*carrotRefObj, nh);
}