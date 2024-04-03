import numpy as np
import math
from nav_msgs.msg import Odometry

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

        #calculate random noise (b is stand. dev, passing in variance)
        sample = lambda b : np.random.normal(loc=0.0, scale=np.sqrt(b), size=None)

        #use odometry controls to influence random noise generation
        #abs to remove neg. square root error
        rand_noise = np.array([[sample(self.a1*math.abs(dx)+self.a2*math.abs(d0))],
                               [sample(self.a1*math.abs(dy)+self.a2*math.abs(d0))],
                               [sample(self.a3*math.abs(d0)+self.a4*math.abs(dx+dy))]])

        for particle in particles:
            theta = particle[2]

            T = np.array([[math.cos(-theta), -math.sin(-theta), 0],[math.sin(-theta), math.cos(-theta), 0],[0,0,1]])
            T_inv = np.linalg.inv(T)
            new_particle = T_inv*odometry.T+rand_noise+particle.T
            particles_new_rows.append(new_particle.T)
        
        particles_new = np.concatenate(particles_new_rows, axis=0)
        return particles_new

        ####################################
