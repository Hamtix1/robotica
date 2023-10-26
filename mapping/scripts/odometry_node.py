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

_odometry = Pose2D()
_truth_pose = Pose2D()
_cmd_vel = Twist()
_last_time = rospy.Time(0)

##############################################################
##############################################################
# Practical session P1.2
# The velocity commands sent to the robot are stored in the
# variable _cmd_vel
# Your task is to estimate the position and orientation of
# the robot with regard to its initial position in (0,0)
# You can estimate the position and orientation by integrating
# the velocity commands sent to the robot.
# Once you have estimated the position and orientation,
# you must set the output _odometry with the proper values.
# You are given with variables that get the current time, and
# store the time measured in the last iteration: current_time and
# _last_time, respectively.

# If you require new global variables, add them here
# and declare them in the function process_data()



# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # _cmd_vel.linear.x , for the linear velocity of the robot
    # _cmd_vel.angular.z , for the angular velocity of the robot
    # current_time stores the the current time
    # _odometry is not cleared, then, it stores the last values you set on the variable
    global _odometry, _cmd_vel, _last_time

    # curren_time stores the time of the current iteration, do not modify
    current_time = rospy.Time.now()
    # _last_time stores the time of the iteration before the current iteration, do not modify
    if _last_time.to_sec() == 0:
        _last_time = current_time
    # You can get the time in seconds by using the following operator
    # curren_time.to_sec()
    # _last_time.to_sec()




    # Outputs: replace the value 0.0 with your output
    # x coordinate with respect to the initial position of the robot, that is (0,0) (m)
    _odometry.x = 0.0
    # y coordinate with respect to the initial position of the robot, that is (0,0) (m)
    _odometry.y = 0.0
    # theta angle with respect to the initial orientation of the robot of the robot, that is (0.0) (rad)
    # theta must be normalized between [-pi, pi],
    _odometry.theta = 0.0
    # Saving current_time in _last_time for next iteration, do not modify
    _last_time = current_time

##############################################################
##############################################################

# Define callbacks and functions to process incoming messages
def gazebo_callback(msg):
    global _truth_pose, _model_index, _model_found

    if _model_found == False:
        _model_index = msg.name.index('robot')
        _model_found = True

    roll = pitch = yaw = 0.0
    orientation_q = msg.pose[_model_index].orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

    _truth_pose.x = msg.pose[_model_index].position.x
    _truth_pose.y = msg.pose[_model_index].position.y
    _truth_pose.theta = yaw

def cmd_vel_callback(msg):
    global _cmd_vel
    _cmd_vel = msg
    process_data()

# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_odometry_node')

    # Subscribe to the topics and associate the correspondign callback functions
    sub_vel = rospy.Subscriber('/robot/cmd_vel/', Twist, cmd_vel_callback)
    sub_pose = rospy.Subscriber('/gazebo/model_states/', ModelStates, gazebo_callback)

    # Publish messages
    pub_odom = rospy.Publisher('/robot/mapping/estimated_odometry', Pose2D, queue_size = 1)
    pub_pose = rospy.Publisher('/robot/mapping/truth_pose', Pose2D, queue_size = 1)

    rate=rospy.Rate(60)

    while not rospy.is_shutdown():

        pub_odom.publish(_odometry)
        pub_pose.publish(_truth_pose)
        rate.sleep()
