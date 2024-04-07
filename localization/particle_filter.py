from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion, quaternion_from_euler
from threading import Lock
from time import time

import numpy as np

from rclpy.node import Node
import rclpy

assert rclpy


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value

        # Particles initialization constants
        self.declare_parameter('num_particles', 1000)
        self.declare_parameter('particle_spread', 1.0)

        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.particle_spread = self.get_parameter('particle_spread').get_parameter_value().double_value
        self.particles = np.zeros((self.num_particles, 3))

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

        self.laser_sub = self.create_subscription(LaserScan, scan_topic, self.laser_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.pose_callback, 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)

        # Visualization
        self.viz_pub = self.create_publisher(PoseArray, "/particles", 1)
        self.viz_timer = self.create_timer(1/20, self.visualize_particles)

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        # Synchronization primitive
        self.lock = Lock()

        self.last_odom_time = None

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
    
    def laser_callback(self, scan: LaserScan):
        """
        From the instructions:
        Whenever you get sensor data use the sensor model to compute the particle probabilities.
        Then resample the particles based on these probabilities.

        Anytime the particles are update (either via the motion or sensor model), determine the
        "average" (term used loosely) particle pose and publish that transform.
        """
        with self.lock:
            probabilities = self.sensor_model.evaluate(self.particles, np.array(scan.ranges))
            # self.get_logger().info(str(np.sum(probabilities)))
            probabilities **= 0.4
            probabilities /= np.sum(probabilities)
            # probabilities = np.exp(probabilities - np.max(probabilities))
            # probabilities /= np.sum(probabilities, axis=0)
            idx = np.random.choice(self.num_particles, self.num_particles, p=probabilities)
            self.get_logger().info(f"{probabilities.shape}, {np.mean(idx)}")
            self.particles = self.particles[idx, :]
            self.publish_transform()

    def odom_callback(self, odom: Odometry):
        """
        From the instructions:
        Whenever you get odometry data use the motion model to update the particle positions.

        Anytime the particles are update (either via the motion or sensor model), determine the
        "average" (term used loosely) particle pose and publish that transform.
        """
        # x, y, theta = ParticleFilter.msg_to_pose(odom.pose.pose)

        # try:
        #     dx = x - self.last_x
        #     dy = y - self.last_y
        #     dtheta = np.arctan2(np.sin(theta) - np.sin(self.last_theta), np.cos(theta) - np.cos(self.last_theta))
        # except AttributeError:
        #     dx, dy, dtheta = 0, 0, 0
        # self.last_x = x
        # self.last_y = y
        # self.last_theta = theta

        # self.get_logger().info(f"d: {dx}, {dy}, {dtheta}")
        # linear = odom.twist.twist.linear
        # dx, dy = linear.x, linear.y
        # dtheta = odom.twist.twist.angular.z
        # self.get_logger().info(str(odom.header.stamp))
        # if self.last_odom_time is None:
        #     self.last_odom_time = time()
        #     dt = 1
        # else:
        #     now = time()
        #     dt = (now - self.last_odom_time)
        #     self.last_odom_time = now
        # dx *= dt
        # dy *= dt
        # dtheta *= dt
        now = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        try:
            dt = now - self.last_odom_stamp
        except AttributeError:
            dt = 0
        self.last_odom_stamp = now

        velocity = odom.twist.twist.linear
        dx, dy = velocity.x * dt, velocity.y * dt
        dtheta = odom.twist.twist.angular.z * dt

        with self.lock:
            self.particles = self.motion_model.evaluate(self.particles, np.array([dx, dy, dtheta]))
            self.publish_transform()

    def publish_transform(self):
        """
        NOTE: This function must be called with an ownership of `self.lock` to prevent race conditions

        From the instructions:
        Anytime the particles are update (either via the motion or sensor model), determine the
        "average" (term used loosely) particle pose and publish that transform.
        """
        x = np.average(self.particles[:, 0]).item()
        y = np.average(self.particles[:, 1]).item()

        theta = self.particles[:, 2]
        theta = np.arctan2(np.average(np.sin(theta)), np.average(np.cos(theta))).item()

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link_pf"
        
        odom.pose.pose = ParticleFilter.pose_to_msg(x, y, theta)

        self.odom_pub.publish(odom)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        From the instruction:
        You will also consider how you want to initialize your particles. We recommend that you
        should be able to use some of the interactive topics in rviz to set an initialize "guess"
        of the robot's location with a random spread of particles around a clicked point or pose.
        Localization without this type of initialization (aka the global localization or the
        kidnapped robot problem) is very hard.
        """
        x, y, theta = ParticleFilter.msg_to_pose(msg.pose.pose)

        with self.lock:
            self.particles = (np.random.random(self.particles.shape) - 0.5) * self.particle_spread
            self.particles += np.array([x, y, 0])

            self.particles[:, 2] = theta + (np.random.random(self.num_particles) - 0.5) * 0.1

    def visualize_particles(self):
        """
        Display the current state of the particles in RViz
        """
        with self.lock:
            msg = PoseArray()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.poses.extend(ParticleFilter.pose_to_msg(x, y, t) for [x, y, t] in self.particles)
        
        self.viz_pub.publish(msg)

    @staticmethod
    def pose_to_msg(x, y, theta):
        msg = Pose()

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = 0.0
        
        quaternion = quaternion_from_euler(0.0, 0.0, theta)
        msg.orientation.x = quaternion[0]
        msg.orientation.y = quaternion[1]
        msg.orientation.z = quaternion[2]
        msg.orientation.w = quaternion[3]

        return msg

    @staticmethod
    def msg_to_pose(msg: Pose):
        pos, ori = msg.position, msg.orientation

        x, y = pos.x, pos.y
        theta = euler_from_quaternion((ori.x, ori.y, ori.z, ori.w))[-1]

        return x, y, theta


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ParticleFilter())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
