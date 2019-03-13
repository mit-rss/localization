class SensorModel:

    def __init__(self, map_msg):

        # Precompute the sensor model here

        pass

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
        
        pass
