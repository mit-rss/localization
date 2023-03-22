from __future__ import division
import numpy as np
from localization.scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D` 
# if any error re: scan_simulator_2d occurs

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale", 1)

        # Tunable Parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0
        self.table_width = 201
        self.z_max = self.table_width - 1.0
        self.eta = 1.0
        self.squashing_parameter = 1.0/2.2


        # Precompute the sensor model table
        self.sensor_model_table = None
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 

        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

        # Initialize Particles
        

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
        # Initialize Lookup Table
        self.sensor_model_table = np.zeros((self.table_width, self.table_width))
        p_max_array = np.zeros((self.table_width, self.table_width))
        p_hit_array = np.zeros((self.table_width, self.table_width))
        p_short_array = np.zeros((self.table_width, self.table_width))
        p_rand_array = np.zeros((self.table_width, self.table_width))
        
        # Construct Lookup Table
        for z_k in range(self.table_width):
            for d in range(self.table_width):
                # Calculate p_max
                p_max = 0
                if (z_k == self.z_max):
                    p_max =  1
                
                # Calculate p_hit
                p_hit = 0
                if ((0 <= z_k) and (z_k <= self.z_max)):
                    p_hit = 1/np.sqrt(2.0 * np.pi * np.power(self.sigma_hit,2)) * np.exp((-np.power((z_k-d),2) )/ (2.0*np.power(self.sigma_hit,2)))
                
                # Calculate p_short
                p_short = 0
                if ((0 <= z_k) and (z_k <= d) and (d != 0)):
                    p_short =  (2.0/d) * (1.0-z_k/d)
                
                # Calculate p_rand
                p_rand = 0
                if ((0 <= z_k) and (z_k <= self.z_max)):
                    p_rand =  1.0/self.z_max

                p_max_array[z_k, d] = p_max
                p_hit_array[z_k, d] = p_hit
                p_short_array[z_k, d] = p_short
                p_rand_array[z_k, d] = p_rand
            
        # Normalize p_hit values
        p_hit_array /= p_hit_array.sum(axis=0)

        # Compute probability table
        self.sensor_model_table = sum([p_max_array*self.alpha_max,p_hit_array*self.alpha_hit, p_short_array*self.alpha_short, p_rand_array*self.alpha_rand])

        # Normalize probability table
        self.sensor_model_table /= self.sensor_model_table.sum(axis=0)


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
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        # Get ray tracing
        scans = self.scan_sim.scan(particles)

        # Downsample LIDAR data (has to be done in particle_filter.py)
        #observation = observation[np.linspace(0, len(observation) - 1, self.num_beams_per_particle, endpoint=True, dtype="int")]

        # Convert meters to pixels
        observation /= (self.map_resolution*self.lidar_scale_to_map_scale)
        np.clip(observation, 0, self.z_max)
        observation = np.round(observation).astype("int")

        scans /= (self.map_resolution*self.lidar_scale_to_map_scale)
        np.clip(scans, 0, self.z_max)
        scans = np.round(scans).astype("int")
       
        # Gather particle beam probabilities
        beam_data = self.sensor_model_table[scans, observation]

        # Compute particle probabilities
        return np.power(np.prod(beam_data, axis=1), self.squashing_parameter)


        

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.0
        self.map = np.clip(self.map, 0, 1)
        self.map_resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
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
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        self.map_set = True

        print("Map initialized")
