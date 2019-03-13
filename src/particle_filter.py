import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

class ParticleFilter:

    def __init__(self):
        # Implement the MCL algorithm
        # using the sensor model and the motion model

        pass

if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFiler()
    rospy.spin()
