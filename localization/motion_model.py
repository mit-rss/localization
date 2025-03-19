import numpy as np

class MotionModel:

    def __init__(self, node:str, noise:float=0.1, deterministic:bool=False):
        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        self.deterministic = deterministic
        self.noise = noise
        self.std = np.sqrt(noise)
        self.node = node

        ####################################

    def evaluate(self, particles: np.ndarray[np.ndarray], odometry: np.ndarray) -> np.ndarray:
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        
        particle: np.ndarray
        # Update each particle
        for particle in particles:
            # By calculating the rotation matrix of the particle movement.
            rotation_matrix: np.ndarray = np.array([[np.cos(particle[2]), -np.sin(particle[2])],
                                                    [np.sin(particle[2]), np.cos(particle[2])]])
            # Adding noise if the motion model is not deterministic.
            if self.deterministic:
                noise: np.ndarray = np.zeros(self.std)
            else:
                noise: np.ndarray = np.random.normal(0, self.noise, self.std)
            # And updating the particle.
            particle[:2] += rotation_matrix @ (odometry[:2] + noise[:2])
            particle[2] += odometry[2] + noise[2]
        
        return particles

        ####################################
