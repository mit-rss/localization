import numpy as np
import math

class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
        ####################################
        pass

    def evaluate(self, particles, odometry):
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
        # TODO

        # Apply noise
        noise_x = np.random.normal(loc=0.0,scale=1.0)
        noise_y = np.random.normal(loc=0.0,scale=1.0)
        noise_theta = np.random.normal(loc=0.0,scale=1.0)
        odometry[0] += noise_x
        odometry[1] += noise_y
        odometry[2] += noise_theta

        # Get new poses
        dxy, dtheta = odometry[0:2], odometry[2]
        new_particles = np.zeros_like(particles)
        for i,particle in enumerate(particles):
            old_theta_body = particle[2]
            old_xy = particle.reshape(2,1)

            rotation_matrix = np.array([math.cos(old_theta_body), -math.sin(old_theta_body), \
                math.sin(old_theta_body), math.cos(old_theta_body)]).reshape(2,2)

            # Put odometry into world frame
            world_dxy = np.matmul(rotation_matrix,dxy)
            new_world_xy = world_dxy+old_xy
            new_particles[i,:] = np.array([new_world_xy[0], new_world_xy[1], old_theta_body+dtheta]) # add new pose

        return new_particles
        ####################################
