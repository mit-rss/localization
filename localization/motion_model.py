
import numpy as np
class MotionModel:

    def __init__(self, node):
        self.node = node

        # Motion noise parameters
        self.sigma_x = 0.02
        self.sigma_y = 0.02
        self.sigma_theta = 0.01

        pass

        ####################################

    def evaluate(self, particles, odometry):
        dx, dy, dtheta = odometry
        n = particles.shape[0]

        # Add Gaussian noise to the odometry for each particle
        noisy_dx = dx + np.random.normal(0.0, self.sigma_x, n)
        noisy_dy = dy + np.random.normal(0.0, self.sigma_y, n)
        noisy_dtheta = dtheta + np.random.normal(0.0, self.sigma_theta, n)

        # Current heading of each particle
        theta = particles[:, 2]

        # Convert body-frame motion into world-frame motion
        particles[:, 0] += np.cos(theta) * noisy_dx - np.sin(theta) * noisy_dy
        particles[:, 1] += np.sin(theta) * noisy_dx + np.cos(theta) * noisy_dy
        particles[:, 2] += noisy_dtheta

        return particles

        ####################################
