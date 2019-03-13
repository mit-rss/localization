class MotionModel:

    def __init__(self):

        # Do any precomputation for the motion model here.

        pass

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        This will be faster if you update the particles
        in place rather than returning them.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]
        """
        
        particles
