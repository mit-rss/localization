import numpy as np
import math
#from nav_msgs.msg import Odometry

class MotionModel:

    def __init__(self, node):
        ####################################
        # Do any precomputation for the motion
        # model here.

        # #adding noise will happen here eventually, coefficients here
        self.a1 = 0.1
        self.a2 = 0.1
        self.a3 = 0.1
        self.a4 = 0.1
        self.node = node
        self.deterministic = False

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
        
        # Creates a rotation + translation matrix
        T = lambda x: np.array([[np.cos(x[2]), -np.sin(x[2]), x[0]], [np.sin(x[2]), np.cos(x[2]), x[1]], [0, 0, 1]])

        dT = T(odometry)
        for i, particle in enumerate(particles):
            t = np.matmul(T(particle), dT)
            particles[i, 0] = t[0, 2]
            particles[i, 1] = t[1, 2]
            particles[i, 2] = np.arctan2(t[1, 0], t[0, 0])
        particles += 0.01 * np.random.normal(size=particles.shape)
        
        return particles

        #note these can be negative, account for accordingly in noise calc
        dx = odometry[0]
        dy = odometry[1]
        d0 = odometry[2]

        # particles[:, 0] += dx
        # particles[:, 1] += dy

        # return particles

        # Extract the last column (thetas)
        thetas = particles[:, -1]

        # Create 3x3 transformation matrices using broadcasting, use - theta for opposite direction rotation
        cos_thetas = np.cos(-thetas)
        sin_thetas = np.sin(-thetas)

        # Create the transformation matrices
        transformation_matrices = np.zeros((len(thetas), 3, 3))
        transformation_matrices[:, 0, 0] = cos_thetas
        transformation_matrices[:, 0, 1] = sin_thetas
        transformation_matrices[:, 1, 0] = -sin_thetas
        transformation_matrices[:, 1, 1] = cos_thetas
        # transformation_matrices[:, 2, 0] = dx
        # transformation_matrices[:, 2, 1] = dy
        transformation_matrices[:, 2, 2] = 1
        
        flattened_matrices = transformation_matrices.reshape(-1, 3)


        sample = lambda b : np.random.normal(loc=0.0, scale=np.sqrt(b))

        # #use odometry controls to influence random noise generation
        # #abs to remove neg. square root error
        # #creates 3xn noise vector
        rand_noise = np.array([sample(self.a1*abs(dx)+self.a2*abs(d0)),
                            sample(self.a1*abs(dy)+self.a2*abs(d0)),
                            sample(self.a3*abs(d0)+self.a4*abs(dx+dy))])

        
        # dx, dy, d-theta
        new_odometry = odometry if self.deterministic else (odometry + rand_noise)
        particles_new = flattened_matrices.dot(new_odometry).reshape(-1,3) + particles 
       
        return particles_new

        ####################################
 