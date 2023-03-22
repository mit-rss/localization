#!/usr/bin/env python2

import rospy
import numpy as np
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

class ParticleFilter:
    def initialize_particles(data):
        pose = data.pose
        covariance = np.array(data.covariance).reshape((6,6))

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        mean = [pose.position.x, pose.position.y, yaw]
        relevant_covariance_idx = [0, 1, 5] #[x], [y], z, roll, pitch, [yaw=theta]
        relevant_covariance = covariance[np.ix_(relevant_covariance_idx, relevant_covariance_idx)] #sub covariance matrix with only x, y, theta
        sample_particles = np.random.multivariate_normal(mean, relevant_covariance, self.num_particles)
        return sample_particles
    
    def take_average(data):
        """ 
        takes numpy array of shape N x 3 in form [x, y, theta] from odometry, and returns a 1 x 3 array of form
        [x_average, y_average, theta_average]
        
        """
        x_odom = data[:, 0]  # x values
        y_odom = data[:, 1]  # y values
        angle_odom = data[:, 2]  # angle values

        x_average = np.average(x_odom)
        y_average = np.average(y_odom)
        angle_average = np.arctan2(np.sum(np.sin(angle_odom)), np.sum(np.cos(angle_odom)))

        average = np.array([[x_average, y_average, angle_average]])

        return average

   
    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")
        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          YOUR_LIDAR_CALLBACK, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          YOUR_ODOM_CALLBACK, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                initialize_particles,                        
                queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
