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
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8

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
        probtable = np.empty((self.table_width, self.table_width), dtype=np.float64)
        phittable = np.empty((self.table_width, self.table_width), dtype=np.float64)

        for zk in range(self.table_width):
            for d in range(self.table_width):
                phittable[zk][d] = (np.exp(-(zk-d)**2/(2*self.sigma_hit**2)))/(2*3.14*self.sigma_hit**2)**0.5
        print(phittable)
        np.transpose(phittable)
        print(phittable)
        for i in range(len(phittable)):
            phittable[i] /= sum(phittable[i])     
        np.transpose(phittable)

        for zk in range(self.table_width):
            for d in range(self.table_width):
                pmax = 1 if zk == (self.table_width - 1) else 0
                pshort = 2.0*(1-zk/d)/d if (0 <= zk and zk <= d and d != 0) else 0
                prand = 1.0/self.table_width
                probtable[zk][d] = self.alpha_hit*phittable[zk][d] + self.alpha_short*pshort + self.alpha_max*pmax + self.alpha_rand*prand
        np.transpose(probtable)
        for i in range(len(probtable)):
            probtable[i] /= sum(probtable[i]) 
        np.transpose(probtable)

        self.sensor_model_table = probtable
        

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
        observation = [x*1.0/self.map_resolution*lidar_scale_to_map_scale for x in observation if 0 <= x <= self.table_width]
        probabilities = [self.sensor_model_table[obs][obs] for obs in observation]

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

# def main():
#     self.sensor_model = SensorModel()
#     map_topic = rospy.get_param("~map_topic")
#     map_msg = rospy.wait_for_message(map_topic, OccupancyGrid)
#     self.sensor_model.map_callback(map_msg)
#     self.sensor_model.alpha_hit = 0.74
#     self.sensor_model.alpha_short = 0.07
#     self.sensor_model.alpha_max = 0.07
#     self.sensor_model.alpha_rand = 0.12
#     self.sensor_model.sigma_hit = 8.0
#     self.sensor_model.table_width = 201
#     self.sensor_model.precompute_sensor_model()


# if __name__ == "__main__":
#     main()
