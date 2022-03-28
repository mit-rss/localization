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
        # TODO
        # Adjust these parameters
        # self.alpha_hit = 1
        # self.alpha_short = 0
        # self.alpha_max = 0
        # self.alpha_rand = 0
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12

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

    def load_etas(self):
        self.eta_hit = np.divide(np.ones(self.table_width), [
            7.92333912, 7.9503748 , 7.97722893, 8.00389909, 8.03038284, 8.05667777,
            8.08278147, 8.10869155, 8.13440562, 8.15992132, 8.18523628, 8.21034818,
            8.23525468, 8.25995346, 8.28444223, 8.30871869, 8.33278058, 8.35662565,
            8.38025164, 8.40365635, 8.42683756, 8.44979308, 8.47252075, 8.4950184,
            8.51728389, 8.53931512, 8.56110997, 8.58266637, 8.60398226, 8.62505558,
            8.64588432, 8.66646647, 8.68680006, 8.70688311, 8.72671369, 8.74628988,
            8.76560978, 8.78467152, 8.80347324, 8.8220131,  8.84028932, 8.85830009,
            8.87604365, 8.89351828, 8.91072225, 8.92765388, 8.9443115,  8.96069347,
            8.97679818, 8.99262405, 9.00816949, 9.02343299, 9.03841302, 9.05310811,
            9.06751679, 9.08163763, 9.09546923, 9.10901021, 9.12225922, 9.13521494,
            9.14787608, 9.16024137, 9.17230958, 9.18407949, 9.19554994, 9.20671975,
            9.21758783, 9.22815307, 9.23841442, 9.24837083, 9.25802131, 9.26736489,
            9.27640063, 9.28512761, 9.29354495, 9.3016518,  9.30944735, 9.31693081,
            9.32410141, 9.33095843, 9.33750119, 9.34372901, 9.34964126, 9.35523735,
            9.3605167 , 9.36547879, 9.37012311, 9.37444918, 9.37845657, 9.38214487,
            9.38551371, 9.38856273, 9.39129165, 9.39370016, 9.39578804, 9.39755506,
            9.39900105, 9.40012587, 9.40092939, 9.40141153, 9.40157225, 9.40141153,
            9.40092939, 9.40012587, 9.39900105, 9.39755506, 9.39578804, 9.39370016,
            9.39129165, 9.38856273, 9.38551371, 9.38214487, 9.37845657, 9.37444918,
            9.37012311, 9.36547879, 9.3605167 , 9.35523735, 9.34964126, 9.34372901,
            9.33750119, 9.33095843, 9.32410141, 9.31693081, 9.30944735, 9.3016518,
            9.29354495, 9.28512761, 9.27640063, 9.26736489, 9.25802131, 9.24837083,
            9.23841442, 9.22815307, 9.21758783, 9.20671975, 9.19554994, 9.18407949,
            9.17230958, 9.16024137, 9.14787608, 9.13521494, 9.12225922, 9.10901021,
            9.09546923, 9.08163763, 9.06751679, 9.05310811, 9.03841302, 9.02343299,
            9.00816949, 8.99262405, 8.97679818, 8.96069347, 8.9443115,  8.92765388,
            8.91072225, 8.89351828, 8.87604365, 8.85830009, 8.84028932, 8.8220131,
            8.80347324, 8.78467152, 8.76560978, 8.74628988, 8.72671369, 8.70688311,
            8.68680006, 8.66646647, 8.64588432, 8.62505558, 8.60398226, 8.58266637,
            8.56110997, 8.53931512, 8.51728389, 8.4950184,  8.47252075, 8.44979308,
            8.42683756, 8.40365635, 8.38025164, 8.35662565, 8.33278058, 8.30871869,
            8.28444223, 8.25995346, 8.23525468, 8.21034818, 8.18523628, 8.15992132,
            8.13440562, 8.10869155, 8.08278147, 8.05667777, 8.03038284, 8.00389909,
            7.97722893, 7.9503748 , 7.92333912
        ])
        self.eta_short = np.divide(np.ones(self.table_width), [
            0.,          40.,         30.,         26.66666667, 25.,         24.,
            23.33333333, 22.85714286, 22.5,        22.22222222, 22.,         21.81818182,
            21.66666667, 21.53846154, 21.42857143, 21.33333333, 21.25,       21.17647059,
            21.11111111, 21.05263158, 21.,         20.95238095, 20.90909091, 20.86956522,
            20.83333333, 20.8,        20.76923077, 20.74074074, 20.71428571, 20.68965517,
            20.66666667, 20.64516129, 20.625,      20.60606061, 20.58823529, 20.57142857,
            20.55555556, 20.54054054, 20.52631579, 20.51282051, 20.5,        20.48780488,
            20.47619048, 20.46511628, 20.45454545, 20.44444444, 20.43478261, 20.42553191,
            20.41666667, 20.40816327, 20.4,        20.39215686, 20.38461538, 20.37735849,
            20.37037037, 20.36363636, 20.35714286, 20.35087719, 20.34482759, 20.33898305,
            20.33333333, 20.32786885, 20.32258065, 20.31746032, 20.3125,     20.30769231,
            20.3030303,  20.29850746, 20.29411765, 20.28985507, 20.28571429, 20.28169014,
            20.27777778, 20.2739726,  20.27027027, 20.26666667, 20.26315789, 20.25974026,
            20.25641026, 20.25316456, 20.25,       20.24691358, 20.24390244, 20.24096386,
            20.23809524, 20.23529412, 20.23255814, 20.22988506, 20.22727273, 20.2247191,
            20.22222222, 20.21978022, 20.2173913,  20.21505376, 20.21276596, 20.21052632,
            20.20833333, 20.20618557, 20.20408163, 20.2020202,  20.2,        20.1980198,
            20.19607843, 20.19417476, 20.19230769, 20.19047619, 20.18867925, 20.18691589,
            20.18518519, 20.18348624, 20.18181818, 20.18018018, 20.17857143, 20.17699115,
            20.1754386,  20.17391304, 20.17241379, 20.17094017, 20.16949153, 20.16806723,
            20.16666667, 20.16528926, 20.16393443, 20.16260163, 20.16129032, 20.16,
            20.15873016, 20.15748031, 20.15625,    20.15503876, 20.15384615, 20.15267176,
            20.15151515, 20.15037594, 20.14925373, 20.14814815, 20.14705882, 20.1459854,
            20.14492754, 20.14388489, 20.14285714, 20.14184397, 20.14084507, 20.13986014,
            20.13888889, 20.13793103, 20.1369863,  20.13605442, 20.13513514, 20.13422819,
            20.13333333, 20.13245033, 20.13157895, 20.13071895, 20.12987013, 20.12903226,
            20.12820513, 20.12738854, 20.12658228, 20.12578616, 20.125,      20.1242236,
            20.12345679, 20.12269939, 20.12195122, 20.12121212, 20.12048193, 20.11976048,
            20.11904762, 20.1183432,  20.11764706, 20.11695906, 20.11627907, 20.11560694,
            20.11494253, 20.11428571, 20.11363636, 20.11299435, 20.11235955, 20.11173184,
            20.11111111, 20.11049724, 20.10989011, 20.10928962, 20.10869565, 20.10810811,
            20.10752688, 20.10695187, 20.10638298, 20.10582011, 20.10526316, 20.10471204,
            20.10416667, 20.10362694, 20.10309278, 20.1025641,  20.10204082, 20.10152284,
            20.1010101,  20.10050251, 20.1
        ])
        self.eta_overall = np.divide(np.ones(self.table_width), [
            0.93,       3.73,       3.03,       2.79666667, 2.68,       2.61,
            2.56333333, 2.53,       2.505,      2.48555556, 2.47,       2.45727273,
            2.44666667, 2.43769231, 2.43,       2.42333333, 2.4175,     2.41235294,
            2.40777778, 2.40368421, 2.4,        2.39666667, 2.39363636, 2.39086956,
            2.38833333, 2.386,      2.38384615, 2.38185185, 2.38,       2.37827586,
            2.37666667, 2.37516129, 2.37375,    2.37242424, 2.37117647, 2.37,
            2.36888889, 2.36783784, 2.3668421,  2.36589744, 2.365,      2.36414634,
            2.36333333, 2.36255814, 2.36181818, 2.36111111, 2.36043478, 2.35978723,
            2.35916667, 2.35857143, 2.358,      2.35745098, 2.35692308, 2.35641509,
            2.35592593, 2.35545455, 2.355,      2.3545614,  2.35413793, 2.35372881,
            2.35333333, 2.35295082, 2.35258065, 2.35222222, 2.351875,   2.35153846,
            2.35121212, 2.35089552, 2.35058823, 2.35028986, 2.35,       2.34971831,
            2.34944444, 2.34917808, 2.34891892, 2.34866667, 2.34842105, 2.34818182,
            2.34794872, 2.34772152, 2.3475,     2.34728395, 2.34707317, 2.34686747,
            2.34666667, 2.34647059, 2.34627907, 2.34609195, 2.34590909, 2.34573034,
            2.34555556, 2.34538462, 2.34521739, 2.34505376, 2.34489362, 2.34473684,
            2.34458333, 2.34443299, 2.34428571, 2.34414141, 2.344,      2.34386139,
            2.34372549, 2.34359223, 2.34346154, 2.34333333, 2.34320755, 2.34308411,
            2.34296296, 2.34284404, 2.34272727, 2.34261261, 2.3425,     2.34238938,
            2.3422807,  2.34217391, 2.34206897, 2.34196581, 2.34186441, 2.34176471,
            2.34166667, 2.34157025, 2.34147541, 2.34138211, 2.34129032, 2.3412,
            2.34111111, 2.34102362, 2.3409375,  2.34085271, 2.34076923, 2.34068702,
            2.34060606, 2.34052632, 2.34044776, 2.34037037, 2.34029412, 2.34021898,
            2.34014493, 2.34007194, 2.34,       2.33992908, 2.33985915, 2.33979021,
            2.33972222, 2.33965517, 2.33958904, 2.33952381, 2.33945946, 2.33939597,
            2.33933333, 2.33927152, 2.33921053, 2.33915033, 2.33909091, 2.33903226,
            2.33897436, 2.3389172,  2.33886076, 2.33880503, 2.33875,    2.33869565,
            2.33864197, 2.33858896, 2.33853659, 2.33848485, 2.33843374, 2.33838323,
            2.33833333, 2.33828402, 2.33823529, 2.33818713, 2.33813953, 2.33809249,
            2.33804598, 2.338,      2.33795455, 2.3379096,  2.33786517, 2.33782123,
            2.33777778, 2.33773481, 2.33769231, 2.33765027, 2.3376087,  2.33756757,
            2.33752688, 2.33748663, 2.33744681, 2.33740741, 2.33736842, 2.33732984,
            2.33729167, 2.33725389, 2.33721649, 2.33717949, 2.33714286, 2.3371066,
            2.33707071, 2.33703518, 2.337
        ])


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
        

        if __name__ == '__main__':
            # Plot our data
            fig = plt.figure(num=1, clear=True)
            (x,y) = np.meshgrid(range(self.table_width), range(self.table_width))
            
            ax = fig.add_subplot(1, 2, 1, projection='3d')
            ax.plot_surface(x,y,self.sensor_model_table)

            # Plot the correct data
            ax2 = fig.add_subplot(1, 2, 2, projection='3d')
            ax2.plot_surface(x,y,np.array(TEST_PRECOMPUTED_TABLE))
            # ax2.plot_surface(x,y,np.ones([self.table_width, self.table_width]))
            
            fig.tight_layout()
            plt.show()
        
        

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
        scans_indices = (scans / map_scale).astype(int)
        obs_indices = (observation / map_scale).astype(int)

        # bound scans and observation
        scans_indices = np.clip( scans_indices, 0, self.table_width - 1 )
        obs_indices = np.clip( obs_indices, 0, self.table_width - 1 )
        
        
        probabilities = np.zeros(n)

        # foreach particle, compute probabilities
        # TODO: Vectorize
        for i in range(len(scans)):
            scan = scans_indices[i]

            # probabilities[i] = np.exp( np.sum( np.log( self.sensor_model_table[(scan,obs_indices)] ) ) )
            probabilities[i] = np.product( self.sensor_model_table[(scan,obs_indices)] )

        return np.power( probabilities, self.squash_power )
        
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
