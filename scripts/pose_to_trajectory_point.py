#!/usr/bin/env python

__author__ = 'aivanovic'

import rospy
from geometry_msgs.msg import Pose, Transform, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

class PoseToTrajectoryPoint:

    def __init__(self):
        self.uav_trajectory_point_pub = rospy.Publisher('position_hold/trajectory', 
            MultiDOFJointTrajectoryPoint, queue_size=1)

        rospy.Subscriber('position_hold/pose_ref', Pose, self.poseCallback, queue_size=1)

    def run(self):
        rospy.spin()

    def poseCallback(self, msg):
        multi = MultiDOFJointTrajectoryPoint()

        transform = Transform()
        transform.translation.x = msg.position.x
        transform.translation.y = msg.position.y
        transform.translation.z = msg.position.z
        transform.rotation.z = msg.orientation.z
        transform.rotation.w = msg.orientation.w

        vel = Twist()
        acc = Twist()

        multi.transforms.append(transform)
        multi.velocities.append(vel)
        multi.accelerations.append(acc)

        self.uav_trajectory_point_pub.publish(multi)

if __name__ == '__main__':

    rospy.init_node('pose_to_trajectory_point')
    pose_to_trajectory = PoseToTrajectoryPoint()
    pose_to_trajectory.run()

