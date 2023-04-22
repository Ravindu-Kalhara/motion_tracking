import unittest
from unittest.mock import MagicMock
from sensor import Sensor
import numpy as np


class TestSensor(unittest.TestCase):

    def setUp(self):
        self.serial = MagicMock()
        self.sensor = Sensor("sensor1", self.serial)

    def test_init(self):
        self.assertEqual(self.sensor.name, b"sensor1")
        self.assertEqual(self.sensor.raw_data.shape, (3, 3))
        self.assertEqual(self.sensor.calibrated_data.shape, (3, 3))
        self.assertEqual(self.sensor.angles.shape, (3,))
        self.assertIsInstance(self.sensor.serial, MagicMock)
        self.assertEqual(len(self.sensor.accel_calibration_matrix), 2)
        self.assertEqual(self.sensor.accel_calibration_matrix[0].shape, (3, 3))
        self.assertEqual(self.sensor.accel_calibration_matrix[1].shape, (3,))
        self.assertEqual(len(self.sensor.gyro_calibration_matrix), 2)
        self.assertEqual(self.sensor.gyro_calibration_matrix[0].shape, (3, 3))
        self.assertEqual(self.sensor.gyro_calibration_matrix[1].shape, (3,))
        self.assertEqual(len(self.sensor.magne_calibration_matrix), 2)
        self.assertEqual(self.sensor.magne_calibration_matrix[0].shape, (3, 3))
        self.assertEqual(self.sensor.magne_calibration_matrix[1].shape, (3,))

    def test_convert_data(self):
        self.sensor.convert_data("1,2,3,4,5,6,7,8,9\n")
        expected_data = np.array([[1, 2, 3], [4, 5, 6], [7, 8, 9]])
        np.testing.assert_array_equal(self.sensor.raw_data, expected_data)

    def test_calibrate(self):
        self.sensor.accel_calibration_matrix = [np.ones((3, 3)), np.ones(3)]
        self.sensor.gyro_calibration_matrix = [np.ones((3, 3)), np.ones(3)]
        self.sensor.magne_calibration_matrix = [np.ones((3, 3)), np.ones(3)]
        self.sensor.raw_data = np.ones((3, 3))
        self.sensor.calibrate()
        expected_data = np.stack((
            np.zeros((3, 3)),
            np.zeros((3, 3)),
            np.zeros((3, 3))
        ))
        np.testing.assert_array_equal(self.sensor.calibrated_data, expected_data)

    def test_roll_pitch_calculation(self):
        self.sensor.calibrated_data = np.array([[0, 1, 0], [0, 0, 0], [0, 0, 0]])
        self.sensor.angle_calculation()
        np.testing.assert_almost_equal(self.sensor.angles[0], 0, decimal=5)
        np.testing.assert_almost_equal(self.sensor.angles[1], 0, decimal=5)
        np.testing.assert_almost_equal(self.sensor.angles[2], 0, decimal=5)

if __name__ == '__main__':
    unittest.main()