from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from rclpy.node import Node
import rclpy

assert rclpy
import numpy as np


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

        self.particles = np.zeros((100, 3))  # Example: 100 particles
        self.last_pose = None



    def pose_callback(self, msg):
        '''
        Handle initialization of particles based on an initial pose estimate.
        '''
        pose = msg.pose.pose  # Extract the Pose from the PoseWithCovarianceStamped message
        x, y, theta = self.pose_to_xyt(pose)
        # Initialize particles around this pose
        self.particles = np.random.normal([x, y, theta], [0.1, 0.1, 0.01], (len(self.particles), 3)) # arbitrarily chosen variances



    def odom_callback(self, msg):
        '''
        Whenever you get odometry data use the motion model to update the particle positions
        '''
        current_pose = msg.pose.pose  # Extract the current pose
        current_xyt = self.pose_to_xyt(current_pose)

        if self.last_pose is not None:
            # Compute differences
            dx = current_xyt[0] - self.last_pose[0]
            dy = current_xyt[1] - self.last_pose[1]
            dtheta = current_xyt[2] - self.last_pose[2]

            # Normalize dtheta to be between -pi and pi
            dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

            # Update particles with the motion model
            self.particles = self.motion_model.evaluate(self.particles, [dx, dy, dtheta])

        # Update the last pose
        self.last_pose = current_xyt
        self.calculate_new_frame_transform()



    def laser_callback(self, msg):
        '''
        Whenever you get sensor data use the sensor model to compute the particle probabilities. 
        Then resample the particles based on these probabilities

        args:
            observation (Lidar) : Correct scan produced by the robot

        '''
        weights = self.sensor_model.evaluate(self.convert_to_xyt(self.particles), msg)
        chosen_indices = np.random.choice(np.arange(len(weights)), size=len(weights), replace=True, p=weights)
        self.particles = self.particles[chosen_indices]
        self.calculate_new_frame_transform()



    def calculate_new_frame_transform(self):
        '''
        Anytime the particles are update (either via the motion or sensor model), determine 
        the "average" (term used loosely) particle pose and publish that transform.
        '''
        pass



    def pose_to_xyt(self, pose):
        '''
        Convert particles between representations of position/orientation
         
        args:
            array of Pose (ROS data structure that contains position as (x,y,z)
            and orientation as (x,y,z,w))
        returns:
            array of [x,y,t] structure where x = position along x, y = position
            along y, and t = theta along xy-plane
        '''
        x = pose.position.x
        y = pose.position.y
        
        # calculating yaw from quaternion (theta)
        t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
        t4 = +1.0 - 2.0 * (pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z)
        theta = np.arctan2(t3, t4)
        # done with theta calculation

        return np.array([x, y, theta])
    



def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
