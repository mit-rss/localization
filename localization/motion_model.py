import numpy as np
import math
#from nav_msgs.msg import Odometry

class MotionModel:

    def __init__(self, node):
        ####################################
        # Do any precomputation for the motion
        # model here.

        # #adding noise will happen here eventually, coefficients here
        self.a1 = 1
        self.a2 = 1
        self.a3 = 1
        self.a4 = 1
        self.node = node

        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y1 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        particles_new_rows = []

        #note these can be negative, account for accordingly in noise calc
        dx = odometry[0]
        dy = odometry[1]
        d0 = odometry[2]

        # Extract the last column (thetas)
        thetas = particles[:, -1]

        # Create 3x3 transformation matrices using broadcasting
        cos_thetas = np.cos(thetas)
        sin_thetas = np.sin(thetas)

        # Create the transformation matrices
        transformation_matrices = np.zeros((len(thetas), 3, 3))
        transformation_matrices[:, 0, 0] = cos_thetas
        transformation_matrices[:, 0, 1] = sin_thetas
        transformation_matrices[:, 1, 0] = -sin_thetas
        transformation_matrices[:, 1, 1] = cos_thetas
        transformation_matrices[:, 2, 2] = 1
        
        flattened_matrices = transformation_matrices.reshape(-1, 3)


        sample = lambda b : np.random.normal(loc=0.0, scale=np.sqrt(b), size=particles.shape[0])

        #use odometry controls to influence random noise generation
        #abs to remove neg. square root error
        #creates 3xn noise vector
        rand_noise = np.array([sample(self.a1*abs(dx)+self.a2*abs(d0)),
                            sample(self.a1*abs(dy)+self.a2*abs(d0)),
                            sample(self.a3*abs(d0)+self.a4*abs(dx+dy))])

        


        particles_new = flattened_matrices.dot(odometry).reshape(-1,3) + particles 
        self.node.get_logger().info(f"{rand_noise.shape}")
        self.node.get_logger().info(f"{particles_new.shape}")
        #self.get_logger().info("did most stuff")
        particles_new = particles_new + rand_noise.T

        # #calculate random noise (b is stand. dev, passing in variance)
        # sample = lambda b : np.random.normal(loc=0.0, scale=np.sqrt(b), size=None)

        # for particle in particles:
        #     theta = particle[2]
        #     #use odometry controls to influence random noise generation
        #     #abs to remove neg. square root error
        #     rand_noise = np.array([[sample(self.a1*math.abs(dx)+self.a2*math.abs(d0))],
        #                        [sample(self.a1*math.abs(dy)+self.a2*math.abs(d0))],
        #                        [sample(self.a3*math.abs(d0)+self.a4*math.abs(dx+dy))]])

        #     T = np.array([[math.cos(-theta), -math.sin(-theta), 0],[math.sin(-theta), math.cos(-theta), 0],[0,0,1]])
        #     T_inv = np.linalg.inv(T)
        #     new_particle = T_inv*odometry.T+rand_noise+particle.T
        #     particles_new_rows.append(new_particle.T)
        
        # particles_new = np.concatenate(particles_new_rows, axis=0)
        return particles_new

        ####################################
 