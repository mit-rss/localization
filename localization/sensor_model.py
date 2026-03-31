import numpy as np
from scan_simulator_2d import PyScanSimulator2D
# Try to change to just `from scan_simulator_2d import PyScanSimulator2D`
# if any error re: scan_simulator_2d occurs

from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import OccupancyGrid

import sys

np.set_printoptions(threshold=sys.maxsize)


class SensorModel:

    def __init__(self, node):
        node.declare_parameter('map_topic', "default")
        node.declare_parameter('num_beams_per_particle', 1)
        node.declare_parameter('scan_theta_discretization', 1.0)
        node.declare_parameter('scan_field_of_view', 1.0)
        node.declare_parameter('lidar_scale_to_map_scale', 1.0)

        self.map_topic = node.get_parameter('map_topic').get_parameter_value().string_value
        self.num_beams_per_particle = node.get_parameter('num_beams_per_particle').get_parameter_value().integer_value
        self.scan_theta_discretization = node.get_parameter(
            'scan_theta_discretization').get_parameter_value().double_value
        self.scan_field_of_view = node.get_parameter('scan_field_of_view').get_parameter_value().double_value
        self.lidar_scale_to_map_scale = node.get_parameter(
            'lidar_scale_to_map_scale').get_parameter_value().double_value

        ####################################

        # sensor_model_test.py 
        # we need to change this later for the real car
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
        Build a 201x201 lookup table so we don't have to do expensive probability
        math at runtime. Each cell table[z, d] stores the probability of the lidar
        measuring distance z when the true wall distance is d. 

        self.sensor_model_table[z, d] = probability

        We compute four probability components (hit, short, max, rand) from the handout, 
        blend them with the alpha weights, and normalize each column to sum to 1.

        Directly modifies self.sensor_model_table.
        """

        z_max = self.table_width - 1

        # Rows = measured distance z, columns = true distance d (both 0..200).
        # Shaped for broadcasting so all math produces a (201, 201) grid.
        z = np.arange(self.table_width).reshape((-1, 1))
        d = np.arange(self.table_width).reshape((1, -1))

        # p_hit: Gaussian centered on the true distance.
        # We skip the 1/sqrt(2*pi*sigma^2) constant since we normalize right after.
        p_hit = np.exp(-0.5 * ((z - d) / self.sigma_hit) ** 2)
        p_hit /= p_hit.sum(axis=0, keepdims=True)

        # p_short: models unexpected obstacles blocking the beam before reaching d.
        # d_safe avoids division by zero in the d=0 column.
        d_safe = np.where(d > 0, d, 1.0)
        p_short = np.where(
            (z <= d) & (d > 0),
            (2.0 / d_safe) * (1.0 - z / d_safe),
            0.0
        )

        # p_max: spike at max range for missed/reflected beams.
        p_max = np.zeros((self.table_width, self.table_width))
        p_max[z_max, :] = 1.0

        # p_rand: small uniform probability for random noise.
        p_rand = np.full((self.table_width, self.table_width), 1.0 / z_max)

        # Mix the four components with the alpha weights, then normalize each column.
        self.sensor_model_table = (
            self.alpha_hit   * p_hit +
            self.alpha_short * p_short +
            self.alpha_max   * p_max +
            self.alpha_rand  * p_rand
        )
        self.sensor_model_table /= self.sensor_model_table.sum(axis=0, keepdims=True)

    def evaluate(self, particles, observation):
        """
        Scores each particle by comparing what the lidar actually measured
        (observation) against what each particle would expect to see (via ray
        tracing on the map). Converts both to pixel units, looks up probabilities
        from the precomputed table, and multiplies across all beams to get a
        single weight per particle. Higher weight = better match to reality.

        args:
            particles: Nx3 matrix of [x, y, theta] particle poses.
            observation: 1D array of lidar ranges (meters) from the real sensor.

        returns:
            probabilities: length-N array of particle weights.
        """

        if not self.map_set:
            return

        # Ray trace from each particle to get what it would see on the map.
        scans = self.scan_sim.scan(particles)

        # Convert from meters to pixel units so we can index into the table.
        scale = self.resolution * self.lidar_scale_to_map_scale
        obs = observation / scale
        scans = scans / scale

        # Clip to table bounds and discretize to ints for indexing.
        z_max = self.table_width - 1
        obs = np.clip(obs, 0, z_max).astype(int)
        scans = np.clip(scans, 0, z_max).astype(int)

        # Look up p(measured z | true d) for every particle and beam at once.
        probs = self.sensor_model_table[obs, scans]

        # Each particle's weight is the product of all its beam probabilities.
        weights = np.prod(probs, axis=1)
        return weights

    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double) / 100.
        self.map = np.clip(self.map, 0, 1)

        self.resolution = map_msg.info.resolution

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation

        quat = [origin_o.x, origin_o.y, origin_o.z, origin_o.w]
        yaw = R.from_quat(quat).as_euler("xyz")[2]

        origin = (origin_p.x, origin_p.y, yaw)

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
            self.map,
            map_msg.info.height,
            map_msg.info.width,
            map_msg.info.resolution,
            origin,
            0.5)

        self.map_set = True
        print("Map initialized")
