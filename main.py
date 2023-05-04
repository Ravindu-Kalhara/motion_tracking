import serial
import time
import numpy as np
from datetime import datetime
from Sensor import Sensor
from Essentials import Essentials


def get_kalman_angles(essentials: Essentials, kalman_angles: list,
                      omega: np.ndarray, kalman_measurement: tuple, dt: float) -> list:
    """Apply kalman filter to all euler angles and get the kalman angles."""

    for i in range(len(kalman_angles)):
        kalman_angles[i] = essentials.kalman_filter(
            kalman_angles[i],
            np.deg2rad(omega[i]),
            kalman_measurement[i],
            delta_t=dt)
    return kalman_angles


# Thease are the clibration matrices.
sensor1_acc_calib_matrices = (np.array([
    [1.003192, 0.002244, -0.002029],
    [0.002244, 1.000812, -0.002718],
    [-0.002029, -0.002718, 1.007279]
]),
    np.array([
        0.157448,
        0.181007,
        -0.172725
    ])
)

sensor1_gyro_calib_matrices = (np.identity(3),
                               np.array([-1.0, 1, 0.0]))


sensor1_magne_calib_matrices = (np.array([
    [1.030710, 0.004980, 0.002201],
    [0.004980, 1.096441, -0.033818],
    [0.002201, -0.033818, 0.984654]]),
    np.array(
    [2.650613, 2.696443, -20.614620])
)

Ess = Essentials()
Sensor1 = Sensor(
    "1",
    serial.Serial("/dev/ttyACM0", 115200)
)
Sensor1_kalman_angles = [[0, 4], [0, 4], [0, 4]]

prevtime = datetime.now()
while True:
    raw_data = Sensor1.get_data()
    calibrated_data = Sensor1.calibrate(raw_data, sensor1_acc_calib_matrices,
                                        sensor1_gyro_calib_matrices, sensor1_magne_calib_matrices)
    angles = Sensor1.angle_calculation(calibrated_data)

    dt = (datetime.now() - prevtime).total_seconds()
    Sensor1_kalman_angles = get_kalman_angles(Ess, Sensor1_kalman_angles, calibrated_data[1], angles, dt)
    prevtime = datetime.now()

    print(Sensor1_kalman_angles[0][0], Sensor1_kalman_angles[1][0], Sensor1_kalman_angles[2][0])
    time.sleep(1/30)
