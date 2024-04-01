from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

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
    
    def laser_callback(self, scan: LaserScan):
        """
        From the instructions:
        Whenever you get sensor data use the sensor model to compute the particle probabilities.
        Then resample the particles based on these probabilities.

        Anytime the particles are update (either via the motion or sensor model), determine the
        "average" (term used loosely) particle pose and publish that transform.
        """
        pass

    def odom_callback(self, odom: Odometry):
        """
        From the instructions:
        Whenever you get odometry data use the motion model to update the particle positions.

        Anytime the particles are update (either via the motion or sensor model), determine the
        "average" (term used loosely) particle pose and publish that transform.
        """
        pass

    def pose_callback(self, pose: PoseWithCovarianceStamped):
        """
        From the instruction:
        You will also consider how you want to initialize your particles. We recommend that you
        should be able to use some of the interactive topics in rviz to set an initialize "guess"
        of the robot's location with a random spread of particles around a clicked point or pose.
        Localization without this type of initialization (aka the global localization or the
        kidnapped robot problem) is very hard.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(ParticleFilter())
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
