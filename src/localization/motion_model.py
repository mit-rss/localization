import rospy
import numpy as np
class MotionModel:

    def __init__(self):
        ####################################
        # self.deterministic = rospy.get_param("~deterministic", False)
        self.deterministic = True

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

        n = len(particles)
        rospy.logwarn(n)
        theta_old = particles[:,2] # column of all thetas
        c = np.cos(theta_old) # column of all coss of thetas
        s = np.sin(theta_old) # column of all sins of thetas
        R = np.array([[c,-s],[s,c]])

        rospy.logwarn(np.shape( np.transpose(odometry[0:2]) ))
        
        dx = np.repeat(np.transpose( np.array([odometry[0:2]]))[:,:,np.newaxis], n, axis =2)
        rospy.logwarn(np.shape(R))
        rospy.logwarn(np.shape(dx))
        new_x = np.dot(R,dx)

        d_thetas = np.tile(odometry[2], (n,1))
        # rospy.logwarn(new_x)
        # rospy.logwarn("poop")
        # rospy.logwarn(thetas)
        delta_x = np.concatenate(new_x,d_thetas,axis = 1)
        # repeated = np.tile(odometry, (n, 1))
        noise = np.random.normal(scale=0.1, size=(n, 3))
        # deltaX = repeated if self.deterministic else repeated+noise 
        
        return particles + delta_x
        # new_theta = old_theta + d_theta
        # new_x = old_x_vec + R(old_theta) * d_x_vec

        ####################################
