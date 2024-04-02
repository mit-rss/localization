import numpy as np
import rclpy
from rclpy.node import Node

from localization.motion_model import MotionModel
from localization.test import TEST_PARTICLES, TEST_MOTION_MODEL_ODOM, TEST_MOTION_MODEL_RESULTS


def wrap_to_pi(angles):
    angles %= 2 * np.pi
    angles -= 2 * np.pi * (angles > np.pi)
    return angles


class MotionModelTest(Node):
    def __init__(self):
        super().__init__('particle_filter')
        try:
            self.motion_model = MotionModel(self)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MotionModel :( {e}")
            exit()

        self.particles = np.array(TEST_PARTICLES)
        self.odom = np.array(TEST_MOTION_MODEL_ODOM)
        self.expected = np.array(TEST_MOTION_MODEL_RESULTS)

        self.tol = 0.05

    def test_evaluate_motion_model(self):
        
        self.motion_model.deterministic = True

        try:
            actual = self.motion_model.evaluate(self.particles, self.odom)
            actual_xy = actual[:, :2] 
            actual_theta = actual[:, 2:] % (2 * np.pi)
        except Exception as e:
            self.get_logger().error(f"Motion model errored out :( {e}")
            exit()

        expected_xy = self.expected[:, :2]
        expected_theta = self.expected[:, 2:] % (2 * np.pi)

        assert np.allclose(actual_xy,
                           expected_xy,
                           rtol=self.tol), f" XYs are not quite right! Expected {expected_xy}, got {actual_xy}"

        assert np.allclose(actual_theta,
                           expected_theta,
                           rtol=self.tol), f"Thetas are not quite right (modulo 2pi)! Expected {expected_theta}, got {actual_theta}"

        self.get_logger().info("Motion model test passed!")
        exit()


def main(args=None):
    rclpy.init(args=args)
    motion_model_test = MotionModelTest()
    motion_model_test.test_evaluate_motion_model()
    rclpy.spin(motion_model_test)
    rclpy.shutdown()
