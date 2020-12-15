#include <uav_ros_control/reference/VisualServo.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "visual_servo_idle");
  ros::NodeHandle nh;

  std::shared_ptr<uav_reference::VisualServo> visualServoObj{
    new uav_reference::VisualServo(nh)
  };
  uav_reference::runIdle(*visualServoObj, nh);
}
