import numpy as np
from scan_simulator_2d import PyScanSimulator2D

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

        ####################################
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0
        self.alpha_short = 0
        self.alpha_max = 1
        self.alpha_rand = 0
        # self.alpha_hit = 0.74
        # self.alpha_short = 0.07
        # self.alpha_max = 0.07
        # self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        self.squash_power = 1/2.2

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

        self.sensor_model_table = np.zeros([self.table_width, self.table_width])
        
        # normalization parameter. Acquired by setting alpha_hit to 1 and all other alpha's to 0, then summing over the columns of the sensor_model_table. I then took the value from the middle of that list
        # these probabilities sums were _not_ all the same; some gaussians were more clipped than others. This is why I took the value from the center of the table where d=zmax/2

        #___                #        ___        #             __#
        #   \               #       /   \       #            /  #
        #    \______________#______/     \______#___________/   #
        # 
        # d=0               d = z_max/2         d = z_max
        # sum of p ~= 0.7   sum of p = 1        sum of p ~= 0.7
        eta = 1/9.407395

        #overall normalization factor
        eta2 = 1/1.0

        # not really sure what this value should be. Does it matter?
        z_max = 10.0

        # 2*sigma^2. For convenience
        ssigma2 = 2*self.sigma_hit**2


        for i in range(self.table_width): # rows
            z_k = i*z_max / (self.table_width - 1)

            for j in range(self.table_width): # columns
                d = j*z_max / (self.table_width - 1)

                p_hit = eta/np.sqrt(np.pi*ssigma2) * np.exp( -(z_k-d)**2/(ssigma2) )

                # TODO: This is very broken
                p_short = np.piecewise(d, d > 0, [lambda d: 2/d * (1 - z_k/d), 0])

                # 1 if z_k == z_max, 0 otherwise
                p_max = np.piecewise(z_k, z_k == z_max, [1, 0])

                # This is 1/table_width instead of 1/z_max, due to discretization nonsense
                p_rand = 1.0/self.table_width

                p = self.alpha_hit*p_hit + self.alpha_short*p_short + self.alpha_max*p_max + self.alpha_rand*p_rand

                self.sensor_model_table[i,j] = eta2 * p
        
        
        # rospy.logwarn( np.shape(self.sensor_model_table))
        # rospy.logwarn( self.sensor_model_table[0,1] )
        # rospy.logwarn( self.sensor_model_table[1,0] )
        # rospy.logwarn( np.sum(self.sensor_model_table, axis=0) )
        rospy.logwarn( np.sum(self.sensor_model_table, axis=0) ) # by column
        

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
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        scans = self.scan_sim.scan(particles)
        return 1
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

        print("Map initialized")
