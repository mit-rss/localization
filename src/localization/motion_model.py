class MotionModel:

    def __init__(self):

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        pass

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
        #Derivation of Matrix Multiplication
        """
        #Y_k1: = cos(Theta_(k-1)1)*dX1 - sin(Theta_(k-1)1) + X_{k-1}1
        #X_k1: sin(Theta_(k-1)1)*dX1 + cos(Theta_(k-1)1) + Y_{k-1}1
        #Theta_k1: cos(Theta_(k-1)1)*cos(dX1) - sin(Theta_(k-1)1)*sin(dX1)
        """

        #Augment particle array
        
