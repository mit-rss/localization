#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_tools import *

class Drive:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic", "/scan")
    DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic", "/vesc/ackermann_cmd_mux/input/navigation")
    SIDE = rospy.get_param("wall_follower/side", -1.) # either left or right (-1 or 1)
    VELOCITY = rospy.get_param("wall_follower/velocity", 1)

    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        #self.line_pub = rospy.Publisher(self.WALL_TOPIC, Marker, queue_size=10)
        self.carpub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)

    def forward_drive(self): # nice to have I guess
        car = AckermannDriveStamped()
        car.header.stamp = rospy.Time.now()
        car.header.frame_id = "base_link"
        car.drive.speed = self.VELOCITY
        car.drive.steering_angle = 0
        self.carpub.publish(car)
    



if __name__ == "__main__":
    rospy.init_node('drive_straight')
    drive_class = Drive()
    drive_straight = drive_class.forward_drive()
    rate = rospy.Rate(20)
    """ while not rospy.is_shutdown():
        wall_follower.stop_drive()
        rate.sleep() """
    rospy.spin()
