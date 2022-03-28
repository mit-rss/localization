#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from scipy.stats import circmean


class ParticleFilter:
    def __init__(self):
        # Get parameters
        self.particle_filter_frame = rospy.get_param("~particle_filter_frame")
        self.num_particles = rospy.get_param("~num_particles")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.deterministic = rospy.get_param("~deterministic")

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
                                          lidar_callback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          odom_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          pose_init_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.cloud_pub  = rospy.Publisher("/pf/pose/cloud", PoseArray, queue_size = 1)
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        self.odom_prev_time = rospy.get_time()

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

        # Jank as hell thread safety
        self.lidar_lock = False
        self.odom_lock = False

    
    def lidar_callback(self, data):
        
        # an instance of the odom function is already running, wait for it to finish
        while self.odom_lock:
            rospy.sleep(0.001)

        # an instance of the lidar function is already running, don't evaluate this lidar message
        if self.lidar_lock:
            return

        # claim the lidar lock. No other processes will be created until this lock is released
        self.lidar_lock = True

        try:
            # downsample lidar data to num_beams_per_particle 
            thetas = np.linspace(-self.scan_field_of_view/2., self.scan_field_of_view/2., self.num_beams_per_particle)
            # assumes angle_min = -angle_max
            theta_indices = (thetas * data.angle_increment / data.angle_max).astype(int)
            observation = data.ranges[theta_indices]
            probabilities = self.sensor_model.evaluate(self.particles, observation)
            
            # resample point cloud
            particle_indices = np.random.choice(self.num_particles, p=probabilities, size=num_particles)
            self.particles = self.particles[particle_indices]

        except Exception as e:
            rospy.logwarn("lidar_callback threw an exception")
            rospy.logwarn(e)
        finally:
            # release the lidar_lock, so other lidar_callback processes can be created
            self.lidar_lock = False

    def odom_callback(self, data):
        # someone else is using the particle array right now, don't touch that data
        if self.lidar_lock or self.odom_lock:
            return
        
        self.odom_lock = True
        try:
            # find delta time
            time = rospy.get_time()
            dt = time - self.odom_prev_time
            self.odom_prev_time = time

            # get odom
            odom = np.array([data.twist.linear.x, data.twist.linear.y, data.twist.angular.z]) * dt
            # update the point cloud
            particles = self.motion_model.evaluate(particles, odom)

            # TODO: add noise somewhere

            # publish results
            self.publish_poses()
        except Exception as e:
            rospy.logwarn("odom_callback threw an exception")
            rospy.logwarn(e)
        finally:
            self.odom_lock = False

    def pose_init_callback(self, data):
        # Pull position from data
        pose = data.pose.pose
        theta = tf.transformations.euler_from_quaternion(pose.orientation)[2]
        center_particle = [pose.position.x, pose.position.y, theta]

        # Pull covariance from data
        covariance = data.pose.covariance
        c_x = covariance[0]
        c_y = covariance[1]
        c_theta = covariance[5]
        
        # Combine to set particle array
        self.particles = np.product( np.random.normal([3, self.num_particles]), [c_x, c_y, c_theta] ) + center_particle

    def publish_poses(self):
        avg_x = np.mean(self.position(0))
        avg_y = np.mean(self.position(1))
        avg_theta = scipy.stats.circmean(theta)
        # TODO: publish the average point
        msg = Odometry()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = ""
        msg.child_frame_id = ""
        msg.pose.pose.position = [avg_x, avg_y, 0]
        msg.pose.pose.orientation = tf.transformations.euler2quaternion([0,0,avg_theta])
        self.odom_pub.publish(msg)

        # TODO: also publish transform?

        br = tf.TransformBroadcaster()
        br.sendTransform(avg_x, avg_y, 0),
  12                      tf.transformations.quaternion_from_euler(0, 0, avg_theta),
  13                      rospy.Time.now(),
  14                      data,
  15                      "map")

        # TODO: publish all the points
        msg = PoseArray()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = ""
        msg.child_frame_id = ""
        msg.pose.point = [self.position(0), self.position(1), 0]
        msg.pose.quaternion = [self.position(0), self.position(1), 0, theta]

        self.cloud_pub.publish(msg)
        pass

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
