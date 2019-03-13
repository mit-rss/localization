import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:

    map_topic = rospy.get_param("~map_topic")
    num_simulated_beams = rospy.get_param("~num_simulated_beams")
    field_of_view rospy.get_param("~field_of_view")

    def __init__(self):

        ####################################
        # Precompute the sensor model here


        ####################################

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_simulated_beams,
                self.field_of_view,
                0, # This is not the simulator, don't add noise
                0.001) # This is used as an epseilon

        # Subscribe to the map
        map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data of
                length N

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        # Perform ray tracing from all the particle locations
        # This produces a matrix of size N x num_beams 
        scans = self.scan_sim.scan(particles)

        pass


    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        map_ = np.array(map_msg.data, np.double)/255.

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
                map_,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free

        # Make the map set
        map_set = True
