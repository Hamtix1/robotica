#!/usr/bin/env python3

# ROS libraries
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Float64

import numpy

# Pre-defined global variables

_qwe = Vector3()
_asd = Vector3()
_pose = Vector3()
_cmd_vel = Twist()
_cam_pan = Float64()
_cam_tilt = Float64()
linear = 0
angular = 0
cameraPan = 0
##############################################################
##############################################################
# Practical session P1.1
# The information retrived from the interface is stored in the
# variables _qwe, _asd, _pose
# Your task is produce the proper velocity output for the robot
# and the set the position of the camera.
# The velocity must be set in the variable _cmd_vel,
# the pan of the camera in the variable _cam_pan and the
# tilt of the camera in the variable _cam_tilt.

# If you require new global variables, add them here
# and declare them in the function process_data()


# Insert your code inside the function process_data
def process_data():
    # You can acces the value of the following variables:
    # If pressed key (q) _qwe.x = 1.0 ; (w) _qwe.y = 1.0; (e) _qwe.z = 1.0. Otherwise, they all equal to 0.0
    # If pressed key (a) _asd.x = 1.0 ; (s) _asd.y = 1.0; (d) _asd.z = 1.0. Otherwise, they all equal to 0.0
    # If click in the interface, _pose.x = vertical coordinate ;  _pose.y = horizontal coordinate. Otherwise, they all equal to -1.0
    # If click out of the interface, or the mouse leaves the interface, _pose.x = _pose.y = -1.0

    global _qwe, _asd, _pose, _cmd_vel, linear, angular, cameraPan
    maxLinear = 3
    maxAngular = 6
    sizeVentana = 300 #Tamaño de la interface cuadrada
    maxCoord=sizeVentana/2
    #Movimiento del robot por interface padMouse
    # print(_pose.y) 
    if _pose.x != -1 and _pose.y != -1:              
        xCursor = -(_pose.y/maxCoord-1)
        yCursor = -(_pose.x/maxCoord-1)
        linear = yCursor*maxLinear
        angular = xCursor*maxAngular
    else:
        linear = 0
        angular = 0
        


    #Movimiento de la cámara por el teclado
    if _qwe.x:
        cameraPan = -0.785
    elif _qwe.y:
        cameraPan = 0
    elif _qwe.z:
        cameraPan = 0.785
    
    #Control de magnitud de velocidad linear y angular por teclado
    if _asd.x:
        if linear >= 0:
            linear = linear - 0.1*maxLinear
            if linear < 0:
                linear = 0
        if angular >= 0:
            angular = angular - 0.1*maxAngular
            if angular < 0:
                angular = 0
        if linear < 0:
            linear = linear + 0.1*maxLinear
            if linear > 0:
                linear = 0
        if angular < 0:
            angular = angular + 0.1*maxAngular
            if angular > 0:
                angular = 0
    
    if _asd.z:
        if linear != 0:
            if linear >= 0:
                linear = linear + 0.1*maxLinear
                if linear > maxLinear:
                    linear = maxLinear
            if angular >= 0:
                angular = angular + 0.1*maxAngular
                if angular > maxAngular:
                    angular = maxAngular
            if linear < 0:
                linear = linear - 0.1*maxLinear
                if linear < -maxLinear:
                    linear = -maxLinear
            if angular < 0:
                angular = angular - 0.1*maxAngular
                if angular < -maxAngular:
                    angular = -maxAngular
    if _asd.y:
        linear = 0
        angular = 0

    

    # Outputs: replace the value 0.0 with your output
    # linear velocity, [-3.0,3.0] (+-1.5 m/s)
    _cmd_vel.linear.x = linear
    # angular velocity, [-6.0,6.0] (+-3.0 rad/s)
    _cmd_vel.angular.z = angular
    # camera pan, [-1.57,1.57] (rad)
    _cam_pan.data = cameraPan
    # camera tilt, [-1.57,1.57] (rad)
    _cam_tilt.data = 0.218

    # Cleaning keyboard input data after being processed, if you want
    # to keep it, you must store it in a global variable.
    # Other variables are not cleaded here.
    clean_keyboard_input()

##############################################################
##############################################################

# Cleans data after being processed
def clean_keyboard_input():
    global _qwe, _asd
    _qwe.x = _qwe.y = _qwe.z = 0
    _asd.x = _asd.y = _asd.z = 0

# Define callbacks and functions to process incoming messages
def qwe_callback(msg):
    global _qwe
    _qwe = msg

def asd_callback(msg):
    global _asd
    _asd = msg

def pose_callback(msg):
    global _pose
    _pose = msg

# Main

if __name__ == '__main__':

    # Add here the name of the ROS node. In ROS, node names must be unique.
    rospy.init_node('robot_teleoperation_node')

    # Subscribe to the topics and associate the corresponding callback functions
    sub_qwe = rospy.Subscriber('/robot/teleoperation/key_qwe/', Vector3, qwe_callback)
    sub_asd = rospy.Subscriber('/robot/teleoperation/key_asd/', Vector3, asd_callback)
    sub_pose = rospy.Subscriber('/robot/teleoperation/mouse_pose/', Vector3, pose_callback)

    # Publish messages
    pub_vel = rospy.Publisher('/robot/cmd_vel/', Twist, queue_size=10)
    pub_pan = rospy.Publisher('/robot/joint_pan_position_controller/command', Float64, queue_size = 1)
    pub_tilt = rospy.Publisher('/robot/joint_tilt_position_controller/command', Float64, queue_size = 1)

    rate=rospy.Rate(30)

    while not rospy.is_shutdown():

        process_data()
        pub_vel.publish(_cmd_vel)
        pub_pan.publish(_cam_pan)
        pub_tilt.publish(_cam_tilt)
        rate.sleep()
