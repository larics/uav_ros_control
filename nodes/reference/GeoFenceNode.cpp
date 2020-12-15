#include <uav_ros_control/reference/GeoFence.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "uav_geofence_node");
  ros::NodeHandle nh;
  std::string ns = ros::this_node::getNamespace();

  std::string filename;
  nh.getParam(ns + "/gps_constraints_filename", filename);

  std::shared_ptr<uav_reference::GeoFence> geoFenceObj{ new uav_reference::GeoFence(
    nh, filename) };

  uav_reference::runDefault(*geoFenceObj, nh);
}