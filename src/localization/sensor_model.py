import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

from test_init import *
from mpl_toolkits.mplot3d import axes3d
import matplotlib.pyplot as plt

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle", 100)
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization", 500)
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view", 4.71)
        self.lidar_scale_to_map_scale = rospy.get_param("~lidar_scale_to_map_scale", 1.)

        ####################################
        # TODO: tune these parameters
        # self.alpha_hit = 1
        # self.alpha_short = 0
        # self.alpha_max = 0
        # self.alpha_rand = 0
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12

        self.sigma_hit = 8.0

        # TODO: tune!
        self.squash_power = 1/4.5

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        ####################################

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
        
        hit_table = np.zeros([self.table_width, self.table_width])
        short_table = np.zeros([self.table_width, self.table_width])

        # Precompute random and max probability tables
        max_table = np.concatenate((
            np.zeros([self.table_width-1, self.table_width]),
            np.ones([1,self.table_width])),
        axis=0)
        rand_table = np.ones([self.table_width, self.table_width]) / (self.table_width-1)
        
        # 2*sigma^2. For convenience
        ssigma2 = 2*(self.sigma_hit**2)

        # TODO: Vectorize
        for z_k in range(self.table_width): # rows
            for d in range(self.table_width): # columns
                hit_table[z_k,d] = np.exp( -((z_k-d)**2)/(ssigma2) ) / np.sqrt(np.pi * ssigma2)

                short_table[z_k,d] = (2./d * (1. - float(z_k)/d)) if (d > 0 and z_k <= d) else 0

        # normalize p_hit
        hit_sums = np.sum(hit_table, axis=0)
        hit_table = np.divide(hit_table, hit_sums)
        
        self.sensor_model_table = self.alpha_hit*hit_table + self.alpha_short*short_table + self.alpha_max*max_table + self.alpha_rand*rand_table
        
        # normalize table
        table_sums = np.sum(self.sensor_model_table, axis=0)
        self.sensor_model_table = np.divide(self.sensor_model_table, table_sums)
        

        # if __name__ == '__main__':
        #     # Plot our data
        #     fig = plt.figure(num=1, clear=True)
        #     (x,y) = np.meshgrid(range(self.table_width), range(self.table_width))
            
        #     ax = fig.add_subplot(1, 2, 1, projection='3d')
        #     ax.plot_surface(x,y,self.sensor_model_table)

        #     # Plot the correct data
        #     ax2 = fig.add_subplot(1, 2, 2, projection='3d')
        #     ax2.plot_surface(x,y,np.array(TEST_PRECOMPUTED_TABLE))
        #     # ax2.plot_surface(x,y,np.ones([self.table_width, self.table_width]))
            
        #     fig.tight_layout()
        #     plt.show()
    
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

        ####################################
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 
        n = len(particles)
        scans = self.scan_sim.scan(particles)
        
        # TODO: Map resolution? Who? Where?
        map_scale =  self.map_resolution * self.lidar_scale_to_map_scale

        # convert to pixels
        scans_indices = np.round((scans / map_scale)).astype(int)
        obs_indices = np.round((observation / map_scale)).astype(int)

        # bound scans and observation
        scans_indices = np.clip( scans_indices, 0, self.table_width - 1 )
        obs_indices = np.clip( obs_indices, 0, self.table_width - 1 )
        
        
        probabilities = np.zeros(n)

        # foreach particle, compute probabilities
        # TODO: Vectorize
        for i in range(n):
            scan = scans_indices[i,:]


            probabilities[i] = np.exp2(
                self.squash_power * np.sum(
                    np.log2( self.sensor_model_table[(obs_indices,scan)] )
                )
            )
            
            # probabilities[i] = np.product(
            #     np.power( self.sensor_model_table[(obs_indices,scan)], self.squash_power)
            # )


        # i = 31
        # rospy.logwarn(np.log( probabilities[i]) )
        # rospy.logwarn(np.log( TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES[i]) )

        # if False:
        #     # Plot our data
        #     fig = plt.figure(num=1, clear=True)
        #     x = range(n)
        #     r = [0,3 * 10**-80]
            
        #     ax = fig.add_subplot(2, 2, 1)
        #     ax.plot(x,probabilities)
        #     ax = fig.add_subplot(2,2,3)
        #     ax.plot(x,np.log(probabilities))
        #     # ax.set_ylim(r)

        #     # Plot the correct data
        #     ax2 = fig.add_subplot(2, 2, 2)
        #     ax2.plot(x,TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES)
        #     ax2 = fig.add_subplot(2, 2, 4)
        #     ax2.plot(x,np.log(TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES))
        #     # ax2.set_ylim(r)

        #     fig.tight_layout()
        #     plt.show()

        return probabilities

        ####################################

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

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

        # HEY I ADDED THIS, POSSIBLY ILLEGALLY
        self.map_resolution = map_msg.info.resolution

        print("Map initialized")

if __name__ == '__main__':
    s = SensorModel()
