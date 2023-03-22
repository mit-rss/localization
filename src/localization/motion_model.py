class MotionModel:

    def __init__(self):
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
        # Derivation of Matrix Multiplication
        """
        #Y_k1: = cos(Theta_(k-1)1)*dX1 - sin(Theta_(k-1)1) + X_{k-1}1
        #X_k1: sin(Theta_(k-1)1)*dX1 + cos(Theta_(k-1)1) + Y_{k-1}1
        #Theta_k1: acos(cos(Theta_(k-1)1)*cos(dX1) - sin(Theta_(k-1)1)*sin(dX1)) = acos(cos(Theta_(k-1)1  + dX1)) = Theta_(k-1)1  + dX1
        """
        
        # Evaluate above Derivation
        particles[:,0] = np.cos(particles[:,2])*odometry[0] - np.sin(particles[:,2])*odometry[1] + particles[:,0]
        particles[:,1] = np.sin(particles[:,2])*odometry[0] + np.cos(particles[:,2])*odometry[1] + particles[:,1]
        particles[:,2] = particles[:,2] + odometry[2]
        
        return particles
        
