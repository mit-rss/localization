#!/usr/bin/env python2

import rospy
import tf2_ros
import numpy as np
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion, TransformStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import threading

class ParticleFilter:   
    def __init__(self):
        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()
        self.particles = np.zeros((self.num_particles, 3))
        self.particle_indices = np.arange(0, self.num_particles)
        self.lock = threading.Lock()
        self.estimated_pose = [0,0,0]

        # Initialize publishers
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.particle_visualizer = rospy.Publisher("/particles", PoseArray, queue_size = 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


        # Initialize subscribers
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.on_get_lidar, 
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.on_get_odometry, 
                                          queue_size=1)
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                self.initialize_particles,                        
                queue_size=1)

    def initialize_particles(self, data):
        pose = data.pose.pose
        covariance = np.array(data.pose.covariance).reshape((6,6))

        orient = pose.orientation
        quat_tuple = (orient.x, orient.y, orient.z, orient.w)
        roll, pitch, yaw = euler_from_quaternion(quat_tuple)
        mean = [pose.position.x, pose.position.y, yaw]
        relevant_covariance_idx = [0, 1, 5] #[x], [y], z, roll, pitch, [yaw=theta]
        relevant_covariance = covariance[np.ix_(relevant_covariance_idx, relevant_covariance_idx)] #sub covariance matrix with only x, y, theta
        init_particles = np.random.multivariate_normal(mean, relevant_covariance, self.num_particles)
        self.particles = init_particles

    def on_get_odometry(self, odometry_data):
        # Get dX
        twist = odometry_data.twist.twist
        twist_dx = twist.linear.x
        twist_dy = twist.linear.y
        twist_dtheta = twist.angular.z
        dX = np.array([twist_dx, twist_dy, twist_dtheta])

        # Update Motion Model
        self.lock.acquire()
        self.particles = self.motion_model.evaluate(self.particles, dX)

        # Update Pose Estimate
        self.update_pose()
        self.lock.release()

    def on_get_lidar(self, lidar_data):
        lidar_data = np.array(lidar_data.ranges)
        # Downsample LIDAR data
        if len(lidar_data) > self.num_beams_per_particle:
            lidar_data = lidar_data[np.linspace(0, len(lidar_data) - 1, self.num_beams_per_particle, endpoint=True, dtype="int")]

        # Update Sensor Model
        self.lock.acquire()
        particle_weights = self.sensor_model.evaluate(self.particles, lidar_data)

        # Resample Particles
        selection = np.random.choice(self.particle_indices, self.num_particles, p=particle_weights)
        self.particles = self.particles[selection]

        # Update Pose Estimate
        self.update_pose()
        self.lock.release()

    def update_pose(self):
        # Retrieve Particle Locations
        particle_x = self.particles[:, 0]  # x values
        particle_y = self.particles[:, 1]  # y values
        particle_theta = self.particles[:, 2]  # angle values

        # Estimate Robot Position through Averaging
        average_x = np.average(particle_x)
        average_y = np.average(particle_y)
        average_theta = np.arctan2(np.sum(np.sin(particle_theta)), np.sum(np.cos(particle_theta)))
        self.estimated_pose = (average_x, average_y, average_theta)

        # Broadcast Estimated Robot Transform
        transform_msg = TransformStamped()
        time = rospy.Time.now()
        transform_msg.header.stamp = rospy.Time.now()
        transform_msg.header.frame_id = "map"
        transform_msg.child_frame_id = self.particle_filter_frame
        transform_msg.transform.translation = Point(average_x, average_y, 0)
        transform_msg.transform.rotation = Quaternion(*quaternion_from_euler(0,0,average_theta))
        self.tf_broadcaster.sendTransform(transform_msg)

        # Visualize Odometry if Listening
        if self.odom_pub.get_num_connections() > 0:
            estimated_odom = Odometry()
            estimated_odom.header.stamp = time
            estimated_odom.header.frame_id = "map"
            estimated_odom.child_frame_id = self.particle_filter_frame
            estimated_odom.pose.pose.position = transform_msg.transform.translation
            estimated_odom.pose.pose.orientation = transform_msg.transform.rotation

            # Publish Odometry Message
            self.odom_pub.publish(estimated_odom)
            
        # Visualize New Particle Cloud if Listening
        if self.particle_visualizer.get_num_connections() > 0:
            pose_array = PoseArray()
            pose_array.header.stamp = time
            pose_array.header.frame_id = "map"
            poses = []

            for i in range(self.num_particles):
                pose = Pose()
                pose.position = Point(self.particles[i, 0], self.particles[i, 1], 0)
                pose.orientation = Quaternion(*quaternion_from_euler(0,0,self.particles[i, 2]))
                poses.append(pose)
            
            pose_array.poses = poses
            self.particle_visualizer.publish(pose_array)



if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
