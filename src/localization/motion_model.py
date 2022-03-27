import rospy
import numpy as np
class MotionModel:

    def __init__(self):
        ####################################
        self.deterministic = rospy.get_param("~deterministic", True)

        ####################################

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

        # number of particles
        n = len(particles)

        # extract columns from particles
        x_old = particles[:,0]
        y_old = particles[:,1]
        theta_old = particles[:,2]

        c = np.cos(theta_old) # column of all cos's of thetas
        s = np.sin(theta_old) # column of all sin's of thetas
        
        # calculate delta column vectors explicitly instead of using rotation matrices
        d_x = np.reshape( odometry[0]*c - odometry[1]*s ,(n,1))
        d_y = np.reshape( odometry[0]*s + odometry[1]*c ,(n,1))

        # make column vector of d_theta
        d_thetas = np.tile(odometry[2], (n,1))
        
        # combine all the delta column vectors into an n*3 matrix
        # np.stack defaults to an n*3*1 matrix for whatever reason
        delta = np.reshape( np.stack([d_x, d_y,d_thetas], axis=1), (n,3) )

        # add noise
        if not self.deterministic:
            # TODO: Play with this scale value. Make it something reasonable
            delta = delta + np.random.normal(scale=0.1, size=(n, 3)) 
        
        return particles + delta
        
        ####################################
