#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

import numpy as np
import traceback
import tf

from sensor_msgs.msg import  LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
import scipy
from scipy.stats import circmean
from geometry_msgs.msg import PoseArray


class ParticleFilter:
    # Jank as hell thread safety
    lidar_lock = True
    odom_lock = True
    odom_prev_time = 0


    # TODO: tune! This is the noise that sets the starting cloud
    init_noise = [2., 2., np.pi]
    # TODO: tune! This is the noise that shifts our points around when processing odometry particle
    particle_noise = [0.5,0.5,0.2]
    odom_noise = [0.5,0.5,0.2]
    

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
                                          self.lidar_callback,
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odom_callback,
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_init_callback,
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        self.cloud_pub  = rospy.Publisher("/pf/pose/cloud", PoseArray, queue_size = 1)
        self.listener = tf.TransformListener()
        
        # Initialize the models
        self.motion_model = MotionModel()
        self.sensor_model = SensorModel()

        self.odom_prev_time = rospy.get_time()
        self.particles = np.zeros([self.num_particles,3])
        self.lidar_lock = False
        self.odom_lock = False
        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        self.s = "/home/racecar/error_log_localization.csv"

        with open(self.s, "w") as self.error_log:
            self.error_log.write("")
        self.error_log = open(self.s, "a")
    
    def lidar_callback(self, data):

        # an instance of the odom function is already running, wait for it to finish
        while self.odom_lock:
            rospy.sleep(0.001)

        # an instance of the lidar function is already running, don't evaluate this lidar message
        if self.lidar_lock:
            return

        # sensor model is not initialized
        if not self.sensor_model.map_set:
            return
        
        # claim the lidar lock. No other processes will be created until this lock is released
        self.lidar_lock = True

        try:
            # downsample lidar data to num_beams_per_particle 
            thetas = np.linspace(-self.scan_field_of_view/2., self.scan_field_of_view/2., self.num_beams_per_particle)

            # retrieve ranges at desired angles
            angle_range = data.angle_max - data.angle_min
            n = len(data.ranges)

            theta_indices = ((thetas - data.angle_min) * n / angle_range).astype(int)
            observation = np.array(data.ranges)[theta_indices]

            # compute probabilities
            probabilities = self.sensor_model.evaluate(self.particles, observation)
            
            # normalize
            probabilities = probabilities / np.sum(probabilities)   
                    
            # resample point cloud
            particle_indices = np.random.choice(self.num_particles, p=probabilities, size=self.num_particles).astype(int)
            self.particles = self.particles[particle_indices]

        except Exception as e:
            rospy.logwarn("lidar_callback threw an exception")
            rospy.logwarn(e)
            traceback.print_exc()
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
            odom = np.array([data.twist.twist.linear.x, data.twist.twist.linear.y, data.twist.twist.angular.z]) * dt
            odom[0] += np.random.normal()*self.odom_noise[0]
            odom[1] += np.random.normal()*self.odom_noise[1]
            odom[2] += np.random.normal()*self.odom_noise[2]
            # update the point cloud
            self.particles = self.motion_model.evaluate(self.particles, odom)

            # add noise
            if not self.deterministic:
                self.particles += self.generate_noise(self.particle_noise)

            # publish results
            self.publish_poses()
        except Exception as e:
            rospy.logwarn("odom_callback threw an exception")
            rospy.logwarn(e)
            traceback.print_exc()
        finally:
            self.odom_lock = False
    
    def log_error(self):
		self.listener.waitForTransform("base_link", "map", rospy.Time(), rospy.Duration(4.0))
		t = self.listener.getLatestCommonTime("base_link", "map")
		exp_position, exp_quaternion = self.listener.lookupTransform("base_link", "map", t)
        exp_angle = tf.euler_from_quaternion(exp_quaternion)[2]
        act_position=[self.avg_x, self.avg_y, 0]
        act_angle = avg_theta
        x_offset = act_position[0]-exp_position[0]
        y_offset = act_position[1]-exp_position[1]
        theta_offset = act_angle-exp_angle
        self.error_log.write("x offset: ", x_offset)
        self.error_log.write("y offset: ", y_offset)
        self.error_log.write("theta offset: ", theta_offset)

    def pose_init_callback(self, data):
        # Pull position from data
        pose = data.pose.pose
        q = pose.orientation

        theta = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        center_particle = [pose.position.x, pose.position.y, theta]

        # Pull covariance from data
        # covariance = data.pose.covariance
        # c_x = covariance[0]      +1
        # c_y = covariance[1]      +1
        # c_theta = covariance[5]  +1
        
        # Combine to set particle array
        noise = self.generate_noise(self.init_noise)
        self.particles = noise + center_particle
    
    def generate_noise(self, weights):
        noise = np.random.normal(size=[self.num_particles, 3])
        return noise * weights

    def publish_poses(self):
        # publish the average point
        self.avg_x = np.mean(self.particles[:,0])
        self.avg_y = np.mean(self.particles[:,1])
        self.avg_theta = scipy.stats.circmean(self.particles[:,2])
        
        msg = Odometry()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"
        # msg.child_frame_id = ""
        msg.pose.pose.position.x = self.avg_x
        msg.pose.pose.position.y = self.avg_y
        msg.pose.pose.position.z = 0
        quat = tf.transformations.quaternion_from_euler(0,0,self.avg_theta)
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]
        self.odom_pub.publish(msg)

        # Publish transform
        br = tf.TransformBroadcaster()
        br.sendTransform((self.avg_x, self.avg_y, 0),
            tf.transformations.quaternion_from_euler(0, 0, self.avg_theta),
            rospy.Time.now(),
            "base_link_pf",
            "map")

        # Publish all the particles
        msg = PoseArray()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"
        # msg.child_frame_id = ""
        msg.poses = []
        for i in range(self.num_particles):
            msg.poses.append( Pose() )
            msg.poses[i].position.x = self.particles[i][0]
            msg.poses[i].position.y = self.particles[i][1]
            quat = tf.transformations.quaternion_from_euler(0, 0, self.particles[i][2])
            msg.poses[i].orientation.x = quat[0]
            msg.poses[i].orientation.y = quat[1]
            msg.poses[i].orientation.z = quat[2]
            msg.poses[i].orientation.w = quat[3]
        self.cloud_pub.publish(msg)
        log_error()


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    # rospy.sleep(3)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     rospy.sleep(0.5)
