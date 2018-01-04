#!/usr/bin/env python

'''
Lab 5 Starter Code

- Outlines a basic implementation of particle filter localization algorithm
- Initializes RangeLibc
- Uses ROS params for easier configuration
- Only sends visualization if someone is listening
- Uses locks to avoid concurreny errors
- Includes code for visualizing discretized sensor models with matplotlib
- Includes helper functions
    - Timer
    - CircularArray
    - Utils
        - coordinate space conversions
        - useful ROS object construction functions

While developing, be careful of:
    - coordinate conversions
    - vectorization with numpy
    - caching and reusing large buffers
    - concurrency problems
    - implement visualization early to help debugging

To launch:

    first start the map server: 
    $ roslaunch lab5 map_server.launch
    then launch this code with:
    $ roslaunch lab5 localize.launch

Written by Corey Walsh for Spring 2017 6.141 Lab 5

'''

import rospy
import numpy as np

from std_msgs.msg import String, Header, Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped,Polygon, Point32, PoseWithCovarianceStamped, PointStamped
from nav_msgs.msg import Odometry
from nav_msgs.srv import GetMap
import tf.transformations
import tf
import matplotlib.pyplot as plt
import range_libc
import time

from threading import Lock

from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter

class ParticleFiler():
    def __init__(self):

        # YOUR CODE - initialize anything else you need here

        # parameters
        self.MAX_PARTICLES = int(rospy.get_param("~max_particles"))
        self.MAX_VIZ_PARTICLES = int(rospy.get_param("~max_viz_particles"))
        self.MAX_RANGE_METERS = float(rospy.get_param("~max_range"))
        self.MAX_RANGE_PX = None
        # cddt and glt range methods are discrete, this defines number of discrete thetas
        self.THETA_DISCRETIZATION = int(rospy.get_param("~theta_discretization"))
        self.WHICH_RANGE_METHOD = rospy.get_param("~range_method", "cddt")

        # various data containers used in the MCL algorithm
        self.map_info = None
        self.map_initialized = False
        self.range_method = None

        # use this lock for controlling accesses to the particles
        # necessary for avoiding concurrency errors
        self.state_lock = Lock()

        # when possible, use these variables to cache large arrays and only make them once
        self.queries = None
        self.ranges = None
        self.sensor_model_table = None
        self.laser_angles = None

        # particle poses and weights - particles should be N by 3
        self.particles = np.zeros((self.MAX_PARTICLES, 3))
        # uniform prior
        self.weights = np.ones(self.MAX_PARTICLES) / float(self.MAX_PARTICLES)

        # initialize the state
        self.get_omap()
        self.precompute_sensor_model()

        # these topics are for visualization, feel free to add, remove, or change as you see fit
        self.pose_pub      = rospy.Publisher("/pf/viz/inferred_pose", PoseStamped, queue_size = 1)
        self.particle_pub  = rospy.Publisher("/pf/viz/particles", PoseArray, queue_size = 1)
        self.pub_fake_scan = rospy.Publisher("/pf/viz/fake_scan", LaserScan, queue_size = 1)
        
        # use this for your inferred transformations
        self.pub_tf = tf.TransformBroadcaster()

        # these topics are to receive data from the racecar
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/scan"), LaserScan, self.lidarCB, queue_size=1)
        self.odom_sub  = rospy.Subscriber(rospy.get_param("~odometry_topic", "/odom"), Odometry, self.odomCB, queue_size=1)

        # this is to send an odometry message with the inferred pose, useful for compatibility with
        # the simulator (since simulator odometry data is perfect) and also REQUIRED for autograding
        self.odom_pub = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 2)
        
        # these integrate with RViz
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.clicked_pose, queue_size=1)
        self.click_sub = rospy.Subscriber("/clicked_point", PointStamped, self.clicked_pose, queue_size=1)

        print "Finished initializing, waiting on messages..."

    def get_omap(self):
        # this way you could give it a different map server as a parameter
        map_service_name = rospy.get_param("~static_map", "static_map")
        print("getting map from service: ", map_service_name)
        rospy.wait_for_service(map_service_name)
        map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map

        self.map_info = map_msg.info
        oMap = range_libc.PyOMap(map_msg)
        # this value is the max range used internally in RangeLibc
        # it also should be the size of your sensor model table
        self.MAX_RANGE_PX = int(self.MAX_RANGE_METERS / self.map_info.resolution)

        # initialize range method
        print "Initializing range method:", self.WHICH_RANGE_METHOD
        if self.WHICH_RANGE_METHOD == "bl":
            self.range_method = range_libc.PyBresenhamsLine(oMap, self.MAX_RANGE_PX)
        elif "cddt" in self.WHICH_RANGE_METHOD:
            self.range_method = range_libc.PyCDDTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
            if self.WHICH_RANGE_METHOD == "pcddt":
                print "Pruning..."
                self.range_method.prune()
        elif self.WHICH_RANGE_METHOD == "rm":
            self.range_method = range_libc.PyRayMarching(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "rmgpu":
            self.range_method = range_libc.PyRayMarchingGPU(oMap, self.MAX_RANGE_PX)
        elif self.WHICH_RANGE_METHOD == "glt":
            self.range_method = range_libc.PyGiantLUTCast(oMap, self.MAX_RANGE_PX, self.THETA_DISCRETIZATION)
        print "Done loading map"

         # 0: permissible, -1: unmapped, large value: blocked
        array_255 = np.array(map_msg.data).reshape((map_msg.info.height, map_msg.info.width))

        # 0: not permissible, 1: permissible
        # this may be useful for global particle initialization - don't initialize particles in non-permissible states
        self.permissible_region = np.zeros_like(array_255, dtype=bool)
        self.permissible_region[array_255==0] = 1
        self.map_initialized = True

    def publish_tf(self,pose, stamp=None):
        """ Publish a tf from map to base_link. """
        if stamp == None:
            stamp = rospy.Time.now()

        # http://wiki.ros.org/tf/Tutorials/Adding%20a%20frame%20(Python)
        # YOUR CODE - publish inferred pose as a transformation frame
        # self.pub_tf...
        
        # YOU MUST PUBLISH TO /pf/pose/odom FOR AUTOGRADING TO WORK
        odom = Odometry()
        odom.header = Utils.make_header("/map", stamp)
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        odom.pose.pose.orientation = Utils.angle_to_quaternion(pose[2])
        self.odom_pub.publish(odom)

    def visualize(self):
        # YOUR CODE - visualize anything you want, the following is provided to get you started
        if self.pose_pub.get_num_connections() > 0 and isinstance(self.inferred_pose, np.ndarray):
            ps = PoseStamped()
            ps.header = Utils.make_header("map")
            # FILL OUT THE POSE 
            # Utils.angle_to_quaternion() will probably be useful
            self.pose_pub.publish(ps)

        if self.particle_pub.get_num_connections() > 0:
            if self.MAX_PARTICLES > self.MAX_VIZ_PARTICLES:
                # randomly downsample particles to avoid killing RViz with tons of particles
                proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES, p=self.weights)
                # proposal_indices = np.random.choice(self.particle_indices, self.MAX_VIZ_PARTICLES)
                self.publish_particles(self.particles[proposal_indices,:])
            else:
                self.publish_particles(self.particles)

        if self.pub_fake_scan.get_num_connections() > 0 and isinstance(self.ranges, np.ndarray):
            # generate the scan from the point of view of the inferred position for visualization
            # this should always align with the map, if not then you are probably using RangeLibc wrong 
            # or you have a coordinate space issue
            self.viz_queries[:,0] = self.inferred_pose[0]
            self.viz_queries[:,1] = self.inferred_pose[1]
            self.viz_queries[:,2] = self.downsampled_angles + self.inferred_pose[2]
            self.range_method.calc_range_many(self.viz_queries, self.viz_ranges)
            self.publish_scan(self.downsampled_angles, self.viz_ranges)

    def publish_particles(self, particles):
        pa = PoseArray()
        pa.header = Utils.make_header("map")
        pa.poses = Utils.particles_to_poses(particles)
        self.particle_pub.publish(pa)

    def publish_scan(self, angles, ranges):
        ls = LaserScan()
        ls.header = Utils.make_header("laser", stamp=self.last_stamp)
        ls.angle_min = np.min(angles)
        ls.angle_max = np.max(angles)
        ls.angle_increment = np.abs(angles[0] - angles[1])
        ls.range_min = 0
        ls.range_max = np.max(ranges)
        ls.ranges = ranges
        self.pub_fake_scan.publish(ls)

    def lidarCB(self, msg):
        if not isinstance(self.laser_angles, np.ndarray):
            print "...Received first LiDAR message"
            # YOUR CODE - initialize any cached numpy arrays, likely including the following
            self.laser_angles = None
            self.viz_queries = None
            self.viz_ranges = None

        # store anything used in MCL update

        self.lidar_initialized = True

    # Odometry data is accumulated via dead reckoning, so it is very inaccurate
    # this function determines relative shift in the coordinate space of the car
    # on the simulator, the odometry data is perfect state information
    def odomCB(self, msg):
        # you will need to convert the global coordinates of the odometry into local coordinate space deltas
        # probably useful: 
        #    Utils.quaternion_to_angle
        #    Utils.rotation_matrix
        
        # YOUR CODE - store anything necessary later on

        # this topic is slower than lidar, so you may want to do MCL update in response to this callback
        self.update()

    def clicked_pose(self, msg):
        # YOUR CODE - initialize particle filter when a pose is clicked in RViz 
        # either a PointStamped or PoseWithCovarianceStamped depending on which RViz tool is used
        pass

    def precompute_sensor_model(self):
        print "Precomputing sensor model"

        # this should be the size of your precomputed table if you are using RangeLibc (which you should)
        # if your table is not this size, you may encounter at best weirdness and at worst segfaults
        # MAX_RANGE_PX is simply the max range in meters scaled by the resolution of the map: self.map_info.resolution
        table_width = int(self.MAX_RANGE_PX) + 1
        self.sensor_model_table = np.zeros((table_width,table_width))


        # YOUR CODE - compute your sensor model here


        # upload the sensor model to RangeLib for ultra fast resolution later
        self.range_method.set_sensor_model(self.sensor_model_table)

        # you may want to visualize your sensor model here, potentially with matplotlib
        # these things should help you get started...

        # code to generate visualizations of the sensor model
        if False:
            # visualize the sensor model
            fig = plt.figure()
            ax = fig.gca(projection='3d')

            # Make data.
            X = np.arange(0, table_width, 1.0)
            Y = np.arange(0, table_width, 1.0)
            X, Y = np.meshgrid(X, Y)

            # Plot the surface.
            surf = ax.plot_surface(X, Y, self.sensor_model_table, cmap="bone", rstride=2, cstride=2,
                                   linewidth=0, antialiased=True)

            ax.text2D(0.05, 0.95, "Precomputed Sensor Model", transform=ax.transAxes)
            ax.set_xlabel('Ground truth distance (in px)')
            ax.set_ylabel('Measured Distance (in px)')
            ax.set_zlabel('P(Measured Distance | Ground Truth)')

            plt.show()
        elif False:
            plt.imshow(self.sensor_model_table * 255, cmap="gray")
            plt.show()
        elif False:
            plt.plot(self.sensor_model_table[:,140])
            plt.plot([139,139],[0.0,0.08], label="test")
            plt.ylim(0.0, 0.08)
            plt.xlabel("Measured Distance (in px)")
            plt.ylabel("P(Measured Distance | Ground Truth Distance = 140px)")
            plt.show()

    # proposal dist should be N by 3 numpy array, action is probably a size 3 numpy vector
    def motion_model(self, proposal_dist, action):
        # YOUR CODE - compute your sensor model here
        # Perturb each particle by the amount given by the last action
        # Add randomness
        # BE WARY OF COORDINATE SPACES
        # Feel free to do a logical implmentation first to get started, but we highly recommend you use no for loops here
        # instead use numpy slice indexing and functions to do your processing to every element at the same time
        pass

    def sensor_model(self, proposal_dist, obs, weights):
        # YOUR CODE - implement your sensor model here, you should modify the weights in place and not return anything

        # only allocate buffers once to avoid slowness
        if self.first_sensor_update:
            # Initialize and store any large reused arrays here
            # if you are using calc_range_many
            # self.queries = np.zeros((num_rays*self.MAX_PARTICLES,3), dtype=np.float32)
            # OR if you are using calc_range_repeat_angles (faster)
            # self.queries = np.zeros((self.MAX_PARTICLES,3), dtype=np.float32)
            # num_queries = self.queries.shape[0]
            # self.ranges = np.zeros(num_queries, dtype=np.float32)
            self.first_sensor_update = False

        # use calc range like this:
        # self.range_method.calc_range_many(self.queries, self.ranges)
        # or this way, which is faster: (angles is a numpy array of angles relative to the query theta that will be ray cast for each query (x,y))
        # self.range_method.calc_range_repeat_angles(self.queries, angles, self.ranges)
        # self.range_method.eval_sensor_model(obs, self.ranges, self.weights, [number of ray casts per particle: should be angles.shape[0]], [number of particles])
        # for calc_range_repeat_angles all ranges for a single particle are next to each other, so the quickly changing axis is angles
        # if you have 6 particles, each of which does 3 ray casts (obviously you will have more) it would be like this:
        # ranges: [p1a1,p1a2,p1a3,p2a1,p2a2,p2a3,p3a1,p3a2,p3a3,p4a1,p4a2,p4a3,p5a1,p5a2,p5a3,p6a1,p6a2,p6a3]
        # input to ranges and queries must be continous float32 numpy arrays

    # this function is on the critical path
    def MCL(self, a, o):
        # YOUR CODE - implement the MCL algorithm here
        pass

    # returns the expected value of the pose given the particle distribution
    def expected_pose(self):
        # YOUR CODE - find the expected value of your particle distribtion
        pass

    # feel free to change this up, but make sure to consider concurrency related issues!
    def update(self):
        if self.lidar_initialized and self.odom_initialized and self.map_initialized:
            if self.state_lock.locked():
                print "Concurrency error avoided"
            else:
                self.state_lock.acquire()
                
                # YOUR CODE - call MCL with the correct arguements

                self.inferred_pose = self.expected_pose()
                self.state_lock.release()
                self.visualize()

if __name__=="__main__":
    rospy.init_node("lab5_localization")
    pf = ParticleFiler()
    rospy.spin()
