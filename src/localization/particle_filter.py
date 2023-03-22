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
                                          self.on_get_odometry, 
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.on_get_odometry, 
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                self.initialize_particles,                        
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
        self.particles = np.zeros((self.num_particles, 3))
        self.particle_indices = np.arange(0, self.num_particles)

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

    def initialize_particles(self, data):
        pose = data.pose
        covariance = np.array(data.covariance).reshape((6,6))

        roll, pitch, yaw = euler_from_quaternion(pose.orientation)
        mean = [pose.position.x, pose.position.y, yaw]
        relevant_covariance_idx = [0, 1, 5] #[x], [y], z, roll, pitch, [yaw=theta]
        relevant_covariance = covariance[np.ix_(relevant_covariance_idx, relevant_covariance_idx)] #sub covariance matrix with only x, y, theta
        sample_particles = np.random.multivariate_normal(mean, relevant_covariance, self.num_particles)
        return sample_particles
        
    def on_get_odometry(self, odometry_data):
        # Get dX
        twist = odometry_data.twist.twist
        twist_dx = twist.linear.x
        twist_dy = twist.linear.y
        twist_dtheta = twist.angular.z
        dX = np.array([twist_dx, twist_dy, twist_dtheta])

        # Update Motion Model
        self.particles = self.motion_model.evaluate(self.particles, dX)

        # Update Pose Estimate
        self.update_pose()

    def on_get_lidar(self, lidar_data):
        # Update Sensor Model
        particle_weights = self.sensor_model.evaluate(self.particles, lidar_data)

        # Resample Particles
        selection = np.random.choice(self.particle_indices, self.num_particles, p=particle_weights)
        self.particles = self.particles[selection]

        # Update Pose Estimate
        self.update_pose()

    def update_pose(self):
        particle_x = self.particles[:, 0]  # x values
        particle_y = self.particles[:, 1]  # y values
        particle_theta = self.particles[:, 2]  # angle values

        average_x = np.average(particle_x)
        average_y = np.average(particle_y)
        average_theta = np.arctan2(np.sum(np.sin(particle_theta)), np.sum(np.cos(particle_theta)))

        estimated_position = np.array([[average_x, average_y, average_theta]])


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
