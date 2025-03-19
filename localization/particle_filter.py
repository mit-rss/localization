from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np 

from rclpy.node import Node
import rclpy

assert rclpy


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        self.get_logger().info("=============+READY+=============")

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

        # initializing number of particles, particles, + their weights 
        self.declare_parameter('num_particles', 100) 
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value

        self.particles = np.zeros((self.num_particles, 3)) # (x, y, theta)  
        self.weights = np.ones(self.num_particles)/self.num_particles 

    def pose_callback(self, pose): 
        """
        use rviz /initial_pose for initializing the particles and a guess of where the robot's location is 
        with a random spread of particles around a clicked point or pose. 
        """

        # getting x, y, theta from the pose 
        x = pose.pose.pose.position.x 
        y = pose.pose.pose.position.y
        theta = np.arctan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) * 2 # calcuting yaw angle 
        
        # intialize particles around this with gaussian 
        self.particles[:, 0] = x + np.random.normal(0, 0.5, self.num_particles)
        self.particles[:, 1] = y + np.random.normal(0, 0.5, self.num_particles)
        self.particles[:, 2] = theta + np.random.normal(0, 0.5, self.num_particles)

        self.weights.fill(1 / self.num_particles) # weights set uniformly across all particles for initialization 

    def odom_callback(self, odometry): 
        """
        process odometry data to pass into motion model
        only rely on twist and not pose 
        """

        # get odometry data 
        dx = odometry.twist.twist.linear.x
        dy = odometry.twist.twist.linear.y
        dtheta = odometry.twist.twist.angular.z  # yaw once again
        odometry_data = np.array([dx, dy, dtheta])

        # evaluate through motion model and update particles
        self.particles = self.motion_model.evaluate(self.particles, odometry_data)

    def laser_callback(self, scan): 
        """
        process scan data to pass into sensor model 
        and resample particles based on sensor model probabilities, numpy.random.choice can be useful 
        """
        
        scan_ranges = np.array(scan.ranges)

        # get probabilities for each particle by passing scans into the sensor model and update weights 
        self.weights = self.sensor_model.evaluate(self.particles, scan_ranges)
        self.weights += 1e-100 # to prevent dividing by 0 
        self.weights /= np.sum(self.weights) # normalize all the weights 

        # resample particles 
        self.particles = self.particles[np.random.choice(range(self.num_particles), self.num_particles, self.weights)]

        # publish msg
        # determine "Average" particle pose and publish 
        # publishes estimated pose as a transformation between the /map frame and a frame for the expected car's base link 
        # --> publish to /base_link_pf for simulator 
        # --> publish to /base_link for real car 

        # weighted means for x and y, circular mean for theta 
        mean_x = np.sum(self.particles[:, 0] * self.weights)
        mean_y = np.sum(self.particles[:, 1] * self.weights)
        mean_theta = np.arctan2(np.sum(np.sin(self.particles[:, 2])), np.sum(np.cos(self.particles[:,2]))) 

        # publish estimated pose 
        msg = Odometry() 
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = mean_x
        msg.pose.pose.position.y = mean_y 
        msg.pose.pose.orientation.z = np.sin(mean_theta / 2)
        msg.pose.pose.orientation.w = np.cos(mean_theta / 2)

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
