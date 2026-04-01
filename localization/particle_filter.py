import numpy as np

from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, PoseArray, Pose
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

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
        self.declare_parameter('num_particles', 200)

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.num_particles = self.get_parameter("num_particles").get_parameter_value().integer_value

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
        self.particle_pub = self.create_publisher(PoseArray, "/pf/particles", 1)

        # TF broadcaster to publish the map -> particle_filter_frame transform
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        # Particles: Nx3 array of [x, y, theta], start at origin
        self.particles = np.zeros((self.num_particles, 3))

        # Track last odometry timestamp so we can compute dt
        self.last_odom_time = None

        # Don't run the filter until the user sets an initial pose
        self.initialized = False

        self.get_logger().info("=============+READY+=============")

    def pose_callback(self, pose_msg):
        """
        Initialize particles around a pose estimate clicked in RViz.
        Spreads particles in a Gaussian cloud around the given x, y, theta.
        """
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y

        # Extract yaw from the quaternion (only z and w matter for 2D)
        q = pose_msg.pose.pose.orientation
        theta = 2.0 * np.arctan2(q.z, q.w)

        # Spread particles around the initial guess with small noise
        self.particles[:, 0] = x 
        self.particles[:, 1] = y 
        self.particles[:, 2] = theta 
        self.initialized = True
        self.get_logger().info(f"Particles initialized at ({x:.2f}, {y:.2f}, {theta:.2f})")

    def odom_callback(self, odom_msg):
        """
        On each odometry message, use the motion model to move all particles
        forward by the measured displacement, then publish an updated pose.
        """
        if not self.initialized:
            return

        # Compute dt since the last odometry message
        current_time = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        if self.last_odom_time is None:
            self.last_odom_time = current_time
            return

        dt = current_time - self.last_odom_time
        self.last_odom_time = current_time

        if dt <= 0:
            return

        # Integrate twist over dt to get a body-frame displacement [dx, dy, dtheta]
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        omega = odom_msg.twist.twist.angular.z
        odometry = np.array([vx * dt, vy * dt, omega * dt])

        self.particles = self.motion_model.evaluate(self.particles, odometry)
        self.publish_pose_estimate()

    def laser_callback(self, scan_msg):
        """
        On each laser scan, use the sensor model to weight particles by how
        well they explain the observation, then resample.
        """
        if not self.initialized:
            return
        if not self.sensor_model.map_set:
            return

        # Downsample the full lidar scan to num_beams_per_particle evenly spaced beams
        ranges = np.array(scan_msg.ranges)
        num_beams = self.sensor_model.num_beams_per_particle
        indices = np.linspace(0, len(ranges) - 1, num_beams, dtype=int)
        observation = ranges[indices]

        # Get a probability weight for each particle
        weights = self.sensor_model.evaluate(self.particles, observation)
        if weights is None:
            return

        # Normalize weights so they sum to 1
        total = np.sum(weights)
        if total == 0:
            weights = np.ones(self.num_particles) / self.num_particles
        else:
            weights = weights / total

        # Resample: particles with higher weight get picked more often
        sampled_indices = np.random.choice(self.num_particles, size=self.num_particles, p=weights)
        self.particles = self.particles[sampled_indices]

        # Add a tiny bit of noise after resampling to keep the set diverse
      

        self.publish_pose_estimate()

    def get_average_pose(self):
        """
        Compute the mean particle pose.
        Uses circular mean for theta to correctly handle angle wraparound.
        """
        avg_x = np.mean(self.particles[:, 0])
        avg_y = np.mean(self.particles[:, 1])

        # Circular mean: average the unit vectors, then take atan2
        avg_theta = np.arctan2(
            np.mean(np.sin(self.particles[:, 2])),
            np.mean(np.cos(self.particles[:, 2]))
        )

        return avg_x, avg_y, avg_theta

    def publish_pose_estimate(self):
        """
        Publish the current best pose estimate as an Odometry message and
        broadcast the corresponding map -> particle_filter_frame TF transform.
        """
        x, y, theta = self.get_average_pose()
        now = self.get_clock().now().to_msg()

        # Publish Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = "map"
        odom_msg.child_frame_id = self.particle_filter_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation.z = np.sin(theta / 2.0)
        odom_msg.pose.pose.orientation.w = np.cos(theta / 2.0)
        self.odom_pub.publish(odom_msg)

        # Publish the full particle cloud as a PoseArray
        pose_array = PoseArray()
        pose_array.header.stamp = now
        pose_array.header.frame_id = "map"
        for p in self.particles:
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            pose.orientation.z = np.sin(p[2] / 2.0)
            pose.orientation.w = np.cos(p[2] / 2.0)
            pose_array.poses.append(pose)
        self.particle_pub.publish(pose_array)

        # Broadcast TF transform so RViz can visualize the car on the map
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = "map"
        tf_msg.child_frame_id = self.particle_filter_frame
        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = np.sin(theta / 2.0)
        tf_msg.transform.rotation.w = np.cos(theta / 2.0)
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()

