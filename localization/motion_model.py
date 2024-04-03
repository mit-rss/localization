import numpy as np

class MotionModel:

    def __init__(self, node):
        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        self.position_variance = 0.01
        self.angle_variance = 0.001

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable future states given the odometry data.

        Args:
            particles: An Nx3 matrix of the form:
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]
            odometry: A 3-vector [dx dy dtheta]

        Returns:
            Updated particles: An updated matrix of the same size.
        """
        N = len(particles)
        dx, dy, dtheta = odometry

        position_noise = np.random.normal(loc=0, scale=np.sqrt(self.position_variance), size=(N,2))
        angle_noise = np.random.normal(loc=0, scale=np.sqrt(self.angle_variance), size=N)

        x_prime = particles[:, 0] + dx * np.cos(particles[:, 2]) + position_noise[:, 0]
        y_prime = particles[:, 1] + dy * np.sin(particles[:, 2]) + position_noise[:, 1]
        theta_prime = particles[:, 2] + dtheta + angle_noise

        # Combine the updates into a new array
        updated_particles = np.stack((x_prime, y_prime, theta_prime), axis=-1)

        return updated_particles


if __name__ == '__main__':
    # Example test set of particles
    particles = np.array([
        [1.0, 2.0, np.pi / 4],  # x, y, theta
        [2.0, 3.0, np.pi / 6],  # x, y, theta
    ])
    
    # Example odometry data: dx, dy, dtheta
    odometry = np.array([0.5, 0.5, 0])

    # Create an instance of the MotionModel class
    motion_model = MotionModel(None)

    # Evaluate the particles with the given odometry
    updated_particles = motion_model.evaluate(particles, odometry)
    print("Updated Particles:\n", updated_particles)
