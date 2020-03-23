#!/usr/bin/env python2

import unittest
import numpy as np

import rospy
import rostest
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from localization.sensor_model import SensorModel
from __init__ import TEST_MAP_ARRAY, TEST_PRECOMPUTED_TABLE, TEST_PARTICLES_2, \
    TEST_SENSOR_MODEL_INPUT_SCANS, TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES

class TestSensorModel(unittest.TestCase):
    def setUp(self):
        self.sensor_model = SensorModel()
        
        map_topic = rospy.get_param("~map_topic")
        map_msg = rospy.wait_for_message(map_topic, OccupancyGrid)
        self.sensor_model.map_callback(map_msg)

        self.sensor_model.alpha_hit = 0.74
        self.sensor_model.alpha_short = 0.07
        self.sensor_model.alpha_max = 0.07
        self.sensor_model.alpha_rand = 0.12
        self.sensor_model.sigma_hit = 8.0
        self.sensor_model.table_width = 201
        self.sensor_model.precompute_sensor_model()

        self.tol = 1e-6

    def tearDown(self):
        pass

    def test_precompute_sensor_model(self):
        expected_table = np.array(TEST_PRECOMPUTED_TABLE)
        actual_table = self.sensor_model.sensor_model_table

        self.assertTrue(actual_table.shape, expected_table.shape)
        np.testing.assert_allclose(expected_table, actual_table, rtol=self.tol)

    def test_evaluate(self):
        expected_probabilities = np.array(TEST_SENSOR_MODEL_OUTPUT_PROBABILITIES)
        actual_probabilities = self.sensor_model.evaluate(
                np.array(TEST_PARTICLES_2), np.array(TEST_SENSOR_MODEL_INPUT_SCANS))

        np.testing.assert_allclose(expected_probabilities,
                                   actual_probabilities,
                                   rtol=self.tol)

    def test_map_callback(self):
        expected_map = np.array(TEST_MAP_ARRAY)
        actual_map = self.sensor_model.map
        np.testing.assert_allclose(expected_map, actual_map, rtol=self.tol)


if __name__ == "__main__":
    rospy.init_node("sensor_model_test")
    rostest.rosrun("localization", 'test_sensor_model', TestSensorModel)
