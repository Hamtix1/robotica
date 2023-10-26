#!/usr/bin/env python

try:
    import tkinter
except ImportError:
    import Tkinter as tkinter

import rospy
from geometry_msgs.msg import Vector3

import numpy

class InterfaceNode():
    def __init__(self):
        # Retrieve params:
        self._frequency = rospy.get_param('~frequency', 15.0)

        # Create publishers:
        self._pub_cmd_pose = rospy.Publisher('/robot_b/teleoperation/mouse_pose/', Vector3, queue_size=10)
        self._pub_cmd_qwe = rospy.Publisher('/robot_b/teleoperation/key_qwe/', Vector3, queue_size=10)
        self._pub_cmd_asd = rospy.Publisher('/robot_b/teleoperation/key_asd/', Vector3, queue_size=10)

        # Initialize mouse position (x, y) to None (unknown); it's initialized
        # when the mouse button is pressed on the _start callback that handles
        # that event:
        self._x_pose = -1.0
        self._y_pose = -1.0
        self._key_press = None

        # Initialize key inputs

        # Create window:
        self._root = tkinter.Tk()
        self._root.title('Input Robot B')

        # Make window non-resizable:
        self._root.resizable(0, 0)

        # Create canvas:
        self._canvas = tkinter.Canvas(self._root, bg='white', width=300, height=300)

        # Bind event handlers:
        self._canvas.bind('<Button-1>', self._start)
        self._canvas.bind('<ButtonRelease-1>', self._release)
        self._canvas.bind('<B1-Motion>', self._mouse_pose)
        self._canvas.bind('<Key>', self._key)

        self._canvas.pack()
        self._canvas.focus_set()

        # If frequency is positive, use synchronous publishing mode:
        if self._frequency > 0.0:
            # Create timer for the given frequency to publish the twist:
            period = rospy.Duration(1.0 / self._frequency)

            self._timer = rospy.Timer(period, self._publish)

        # Start window event manager main loop:
        self._root.mainloop()

    def __del__(self):
        if self._frequency > 0.0:
            self._timer.shutdown()

        self._root.quit()

    def _start(self, event):
        self._x_pose = event.x
        self._y_pose = event.y

    def _release(self, event):
        self._x_pose = -1.0
        self._y_pose = -1.0

    def _key(self, event):
        self._key_press = event.char

    def _mouse_pose(self, event):
        if event.x >= 0 and event.x <= 300 and event.y >= 0 and event.y <= 300:
            self._x_pose = event.x
            self._y_pose = event.y
        else:
            self._x_pose = -1.0
            self._y_pose = -1.0

    def _send_data(self):
        if self._key_press in {"q", "w", "e"}:
            qwe = Vector3()
            if self._key_press == "q":
                qwe.x = 1.0
            if self._key_press == "w":
                qwe.y = 1.0
            if self._key_press == "e":
                qwe.z = 1.0
            self._pub_cmd_qwe.publish(qwe)
            self._key_press = None
        if self._key_press in {"a", "s", "d"}:
            asd = Vector3()
            if self._key_press == "a":
                asd.x = 1.0
            if self._key_press == "s":
                asd.y = 1.0
            if self._key_press == "d":
                asd.z = 1.0
            self._pub_cmd_asd.publish(asd)
            self._key_press = None
        pose = Vector3()
        pose.y = self._x_pose
        pose.x = self._y_pose
        self._pub_cmd_pose.publish(pose)

    def _publish(self, event):
        self._send_data()

def main():
    rospy.init_node('robot_b_interface_node')

    InterfaceNode()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
