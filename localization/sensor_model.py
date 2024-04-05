import numpy as np
import math
import sys
from scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D` 
# if any error re: scan_simulator_2d occurs

from tf_transformations import euler_from_quaternion

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class SensorModel:

    def __init__(self, node):
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('num_beams_per_particle', "default")
        node.declare_parameter('scan_theta_discretization', "default")
        node.declare_parameter('scan_field_of_view', "default")
        node.declare_parameter('lidar_scale_to_map_scale', 1)

        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.num_beams_per_particle = node.get_parameter('num_beams_per_particle').get_parameter_value().integer_value
        self.scan_theta_discretization = node.get_parameter(
            'scan_theta_discretization').get_parameter_value().double_value
        self.scan_field_of_view = node.get_parameter('scan_field_of_view').get_parameter_value().double_value
        self.lidar_scale_to_map_scale = node.get_parameter(
            'lidar_scale_to_map_scale').get_parameter_value().double_value

        ####################################
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

        node.get_logger().info("%s" % self.map_topic)
        node.get_logger().info("%s" % self.num_beams_per_particle)
        node.get_logger().info("%s" % self.scan_theta_discretization)
        node.get_logger().info("%s" % self.scan_field_of_view)

        # Precompute the sensor model table
        self.sensor_model_table = np.empty((self.table_width, self.table_width))
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
            self.num_beams_per_particle,
            self.scan_field_of_view,
            0,  # This is not the simulator, don't add noise
            0.01,  # This is used as an epsilon
            self.scan_theta_discretization)

        # Subscribe to the map
        self.map = None
        self.map_set = False
        self.map_subscriber = node.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.

        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A

        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        alpha_hit = self.alpha_hit
        alpha_short = self.alpha_short
        alpha_max = self.alpha_max
        alpha_rand = self.alpha_rand
        sigma_hit = self.sigma_hit
        eta = 1


        z_max = self.table_width-1 #assumes that the longest dist (in px) is the last entry in the table
        d_array = np.linspace(0.000001,z_max,self.table_width)
        z_array = np.linspace(0.000001,z_max,self.table_width)

        # Compute raw p_hit
        diff_squared = (z_array[:, np.newaxis] - d_array[np.newaxis, :]) ** 2
        exponent = - diff_squared / (2 * sigma_hit**2)
        constant = eta * 1 / (2 * math.pi * sigma_hit**2)**0.5
        p_hit = constant * np.exp(exponent)
        #normalize p_hit
        column_sums = np.sum(p_hit, axis=0) 
        p_hit_normalized = p_hit / column_sums[np.newaxis, :] 
        p_hit = p_hit_normalized

        #compute p_short
        constant = 2/d_array[np.newaxis,:]
        multiplier = np.where(d_array[np.newaxis,:]>z_array[:,np.newaxis],1-z_array[:,np.newaxis]/d_array[np.newaxis,:],0)
        p_short = constant*multiplier

        #compute p_max
        p_max = np.where(z_array[:,np.newaxis]==z_max,1,0)

        #compute p_rand
        p_rand = np.full((self.table_width,self.table_width),1/z_max)

        #compute total probability
        p_table = alpha_hit*p_hit+alpha_max*p_max+alpha_rand*p_rand+alpha_short*p_short
        #normalize the p_table by d value (columns i think)
        total_column_sums = np.sum(p_table, axis=0)
        p_table_normalized = p_table / total_column_sums[np.newaxis, :]
        #trouble shooting
        sums = np.sum(p_table_normalized,axis=0)
        self.sensor_model_table = p_table_normalized


    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar. THIS IS Z_K. Each range in Z_K is Z_K^i

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        particle_scans = self.scan_sim.scan(particles)

        #convert scans from meters to pixels and round to the nearest int
        particle_scans_px = particle_scans/(self.resolution*self.lidar_scale_to_map_scale)
        particle_scans_px = np.round(particle_scans_px)
        particle_scans_px = particle_scans_px.astype(int)
        particle_scans_px = np.clip(particle_scans_px,0,(self.table_width-1))

        #downsample the lidar
        stride = max(1, observation.shape[0] // particle_scans_px.shape[1])
    
        # Slice the array to select every `stride`-th element
        downsmpled_observation = observation[::stride]

        #convert the downsampled observation to px and round
        px_observation = downsmpled_observation/(self.resolution*self.lidar_scale_to_map_scale)
        px_observation = np.round(px_observation)
        px_observation = px_observation.astype(int)
        px_observation = np.clip(px_observation,0,(self.table_width-1))


        #calculate the probability of each vector
        particle_probs = self.sensor_model_table[px_observation,particle_scans_px]

        #total each particles probability
        total_particle_probs = np.prod(particle_probs, axis=1)

        return total_particle_probs


        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = euler_from_quaternion((
            origin_o.x,
            origin_o.y,
            origin_o.z,
            origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)  # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")








