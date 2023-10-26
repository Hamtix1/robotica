#!/usr/bin/env python

# ROS libraries
import rospy
from geometry_msgs.msg import Pose2D, Twist
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

import numpy
import math
from math import sin, cos, pi

# Pre-defined global variables

_truth_pose = Pose2D()
_cmd_vel = Twist()

##############################################################
##############################################################
# Practical session P1.4
# The position and orientation of the robot is given in the
# variable _trhuth_pose
# Your task is to design the path planning algorithm that
# drives the robot to the waypoints signaled in the map, and
# in the shortest possible time.
# To do so, you must drive the robot by checking its position and orientation, and
# send velocity commands accordingly.
# you must set the output _cmd_vel with the proper velocity values.


# You can create 2D pose structures such as
# _waypoint_1 = Pose2D()
# _and assign value to its x and y coordinates, and theta orientation:
# _waypoint_1.x = 1.0 # (m)
# _waypoint_1.y = 1.0 # (m)
# _waypoint_1.theta = 1.0 # (rad)[-pi,pi],
# If you require these or other class of global variables, add them here
# and declare them in the function process_data()



# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # _truth_pose.x , for the x coordinate of the position of the robot
    # _truth_pose.y , for the y coordinate of the position of the robot
    # _truth_pose.theta, for the orientation of the robot
    # The position and orientation of the robot is computed with
    # respect to the center of the field, in the position (0,0).
    global _truth_pose, _cmd_vel




    # Outputs: replace the value 0.0 with your output
    # linear velocity, [-3.0,3.0] (+-1.5 m/s)
    _cmd_vel.linear.x = 0.0
    # angular velocity, [-6.0,6.0] (+-3.0 rad/s)
    _cmd_vel.angular.z = 0.0

##############################################################
##############################################################

# Define callbacks and functions to process incoming messages
def gazebo_callback(msg):
    global _truth_pose, _model_index, _model_found

    if _model_found == False:
        _model_index = msg.name.index('robot_b')
        _model_found = True

    roll = pitch = yaw = 0.0
    orientation_q = msg.pose[_model_index].orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    _truth_pose.x = msg.pose[_model_index].position.x
    _truth_pose.y = msg.pose[_model_index].position.y
    _truth_pose.theta = yaw


# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_b_pathplanning_node')

    # Subscribe to the topics and associate the correspondign callback functions
    sub_pose = rospy.Subscriber('/gazebo/model_states/', ModelStates, gazebo_callback)

    # Publish messages
    pub_vel = rospy.Publisher('/robot_b/cmd_vel/', Twist, queue_size = 10)
    pub_pose = rospy.Publisher('/robot_b/mapping/truth_pose', Pose2D, queue_size = 1)

    rate=rospy.Rate(30)

    while not rospy.is_shutdown():

        process_data()
        pub_odom.publish(_cmd_vel)
        pub_pose.publish(_truth_pose)
        rate.sleep()
