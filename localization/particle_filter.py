from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Pose, PoseArray, Quaternion, TransformStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import tf_transformations
import tf2_ros
from ackermann_msgs.msg import AckermannDriveStamped

from rclpy.node import Node
import rclpy

assert rclpy
import numpy as np
from threading import Lock


class ParticleFilter(Node):

    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.declare_parameter('num_beams_per_particle', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value
        self.num_beams_per_particle = self.get_parameter('num_beams_per_particle').get_parameter_value().integer_value

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

        self.particles_pub = self.create_publisher(PoseArray, "/particles", 10)

        self.transform_pub = tf2_ros.TransformBroadcaster(self)

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, "/drive", 10)

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

        self.particles = np.zeros((200, 3))  # Example: 100 particles
        self.last_pose = None

        self.prev_time = self.get_clock().now()

        self.thread_lock = Lock()



    def pose_callback(self, msg):
        '''
        Handle initialization of particles based on an initial pose estimate.
        '''
        pose = msg.pose.pose  # Extract the Pose from the PoseWithCovarianceStamped message
        x, y, theta = self.pose_to_xyt(pose)
        # Initialize particles around this pose
        self.particles = np.random.normal([x, y, theta], [1, 1, 0.1], (len(self.particles), 3)) # arbitrarily chosen variances
        self.get_logger().info("Pose Init")



    def odom_callback(self, msg):
        '''
        Whenever you get odometry data use the motion model to update the particle positions
        '''
        # current_pose = msg.pose.pose  # Extract the current pose
        # current_xyt = self.pose_to_xyt(current_pose)

        # if self.last_pose is not None:
        #     # Compute differences
        #     dx = current_xyt[0] - self.last_pose[0]
        #     dy = current_xyt[1] - self.last_pose[1]
        #     dtheta = current_xyt[2] - self.last_pose[2]

        #     # Normalize dtheta to be between -pi and pi
        #     dtheta = (dtheta + np.pi) % (2 * np.pi) - np.pi

        #     # Update particles with the motion model
        #     self.particles = self.motion_model.evaluate(self.particles, [dx, dy, dtheta])

        # # Update the last pose
        # self.last_pose = current_xyt

        current_time = self.get_clock().now()

        twist = msg.twist.twist
        dx = twist.linear.x
        dy = twist.linear.y
        dtheta = twist.angular.z

        delta = np.array([dx, dy, dtheta])
        delta *= float((current_time-self.prev_time).nanoseconds/(10**9))
        self.prev_time = current_time

        self.thread_lock.acquire()

        self.particles = self.motion_model.evaluate(self.particles, delta)
        self.calculate_new_frame_transform()

        self.thread_lock.release()


    def laser_callback(self, msg):
        '''
        Whenever you get sensor data use the sensor model to compute the particle probabilities. 
        Then resample the particles based on these probabilities

        args:
            observation (Lidar) : Correct scan produced by the robot

        '''
        scan = np.array(msg.ranges)

        # downsampling
        if len(scan) > self.num_beams_per_particle:
            indices = np.round(np.linspace(0, len(scan) - 1, self.num_beams_per_particle, endpoint=True)).astype(int)
            scan = scan[indices]

        self.thread_lock.acquire()
        weights = self.sensor_model.evaluate(self.particles, scan)
        if weights is None:
            self.thread_lock.release()
            return
        weights /= np.sum(weights)

        

        chosen_indices = np.random.choice(np.arange(len(weights)), size=len(weights), replace=True, p=weights)
        self.particles = self.particles[chosen_indices]
        self.calculate_new_frame_transform()
        self.thread_lock.release()



    def calculate_new_frame_transform(self):
        '''
        Anytime the particles are update (either via the motion or sensor model), determine 
        the "average" (term used loosely) particle pose and publish that transform.
        '''
        # Calculate the circular mean for theta
        def circular_mean(angles):
            cos_sum = np.sum(np.cos(angles))
            sin_sum = np.sum(np.sin(angles))
            mean_angle = np.arctan2(sin_sum, cos_sum)
            return mean_angle

        # Calculate the mean pose
        def calculate_mean_pose(particle_data):
            # Calculate the mean for x and y dimensions
            mean_x = np.mean(particle_data[:, 0])
            mean_y = np.mean(particle_data[:, 1])
            
            # Calculate the circular mean for theta (angle)
            mean_theta = circular_mean(particle_data[:, 2])
            
            # Convert mean_theta to [0, 2π] range
            mean_theta = (mean_theta + 2 * np.pi) % (2 * np.pi)
            
            return np.array([mean_x, mean_y, mean_theta])

        now = self.get_clock().now().to_msg()


        # Calculate the mean pose
        mean_pose = calculate_mean_pose(self.particles)
        poseObj = self.xyt_to_pose(mean_pose)

        transform_msg = TransformStamped()
        transform_msg.header = Header(frame_id='/map', stamp=now)
        transform_msg.child_frame_id = self.particle_filter_frame

        transform_msg.transform.translation.x = poseObj.position.x
        transform_msg.transform.translation.y = poseObj.position.y
        transform_msg.transform.translation.y = poseObj.position.y

        transform_msg.transform.rotation.x = poseObj.orientation.x
        transform_msg.transform.rotation.y = poseObj.orientation.y
        transform_msg.transform.rotation.z = poseObj.orientation.z
        transform_msg.transform.rotation.w = poseObj.orientation.w
        
        self.transform_pub.sendTransform(transform_msg)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = 2.0
        self.drive_publisher.publish(drive_msg)

        if self.odom_pub.get_subscription_count() > 0:
            odomMsg = Odometry()
            odomMsg.header = Header(frame_id='/map', stamp=now)
            odomMsg.child_frame_id = self.particle_filter_frame
            odomMsg.pose.pose = poseObj
            self.odom_pub.publish(odomMsg)

        if self.particles_pub.get_subscription_count() > 0:
            particles = []
            for particle in self.particles:
                particles.append(self.xyt_to_pose(particle))
            particlesMsg = PoseArray()
            particlesMsg.header = Header(frame_id='/map', stamp=now)
            particlesMsg.poses = particles
            
            self.particles_pub.publish(particlesMsg)







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
    
    def xyt_to_pose(self, xyt):
        '''
        Convert position/orientation representation to ROS Pose data structure
        
        Args:
            xyt: array of [x, y, theta] representing position along x, position
                along y, and theta along xy-plane
                
        Returns:
            ROS Pose data structure containing position as (x, y, z) and orientation
            as (x, y, z, w)
        '''
        # Extract x, y, and theta from the input array
        x, y, theta = xyt
        
        # Create a Point object for position (x, y, z)
        position = Point()
        position.x = x
        position.y = y
        position.z = 0.0
        
        # Calculate quaternion orientation from yaw (theta)
        quaternion = Quaternion()
        x, y, z, w = tf_transformations.quaternion_from_euler(0, 0, theta)
        quaternion.x = x
        quaternion.y = y
        quaternion.z = z
        quaternion.w = w
        
        # Create a Pose object and set its position and orientation
        pose = Pose()
        pose.position = position
        pose.orientation = quaternion
        return pose
    



def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()
