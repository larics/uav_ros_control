# UAV ROS Control 

| Ubuntu 18.04  | Ubuntu 20.04|
|---------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------|
 [![Melodic](https://github.com/lmark1/uav_ros_control/workflows/Melodic/badge.svg)](https://github.com/lmark1/uav_ros_control/actions) | [![Noetic](https://github.com/lmark1/uav_ros_control/workflows/Noetic/badge.svg)](https://github.com/lmark1/uav_ros_control/actions) |

## Summary

The main control package of the [uav_ros_stack](https://github.com/lmark1/uav_ros_stack).  

It contains UAV control algorithms suitable for use with Ardupilot or PX4 platforms. 

## Node description

Both nodes are started with default configurations as follows:
```bash
export UAV_NAMESPACE=red; roslaunch uav_ros_control pid_carrot.launch
```

Argument **control_type** denotes the control program used:
* pid_cascade_node - Ardupilot compatible control using roll-pitch-yaw-thrust commands
* pid_cascade_node_yawrate - Ardupilot compatible control using roll-pitch-yawrate-thrust commands
* pid_cascade_node_px4 - PX4 compatible control using roll-pitch-yaw-thrust commands
* pid_cascade_node_px4_yawrate - PX4 compatible control using roll-pitch-yawrate-thrust commands

### **PositionControlNode** 
An implementation of a cascade PID UAV control scheme  
* Subscribed topic **odometry** 
  * ```nav_msgs::Odometry```
  * current UAV estimated odometry
* Subscribed topic **uav/trajectory_point** 
  * ```trajectory_msgs::MultiDOFJointTrajectoryPoint``` 
  * current UAV referent trajectory point
* Published topic **uav/attitude_target**
  * ```mavros_msgs::AttitudeTarget```
  * calculated UAV attitude and thrust target

### **CarrotReferenceNode**
Used for publishing trajectory setpoints either through RC control or published topic  
* Parameter **manual_takeoff**
  * Enables automatic takeoff if set to *true*
* Parameters **carrot_index** and **carrot_enable**
  * Specify joy button that enables RC UAV control
* Service **takeoff**
  * ```uav_ros_msgs::Takeoff```
  * UAV takes off to the desired height
* Service **land**
  * ```std_srvs::SetBool```
  * UAV lands at the current position
* Service **position_hold**
  * ```std_srvs::Empty```
  * When called UAV listens to commands from topic **position_hold/trajectory**
  * If **joy** topic receives any inputs UAV exits position hold and listens to RC inputs
* Subscribed topic **joy**
  * ```sensor_msgs::Joy```
  * Controls the UAV if button at **carrot_index** is pressed
* Subscribed topic **odometry** 
  * ```nav_msgs::Odometry```
  * current UAV estimated odometry
* Subscribed topic **position_hold/trajectory** 
  * ```trajectory_msgs::MultiDOFJointTrajectoryPoint``` 
  * External UAV referent trajectory points
  * Relevant when UAV is in position hold
* Published topic **carrot/trajectory** 
  * ```trajectory_msgs::MultiDOFJointTrajectoryPoint``` 
  * current UAV referent trajectory point