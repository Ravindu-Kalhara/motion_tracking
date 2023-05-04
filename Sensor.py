import numpy as np
import serial


class Sensor():
    def __init__(self, _name: str, _serial: serial.Serial) -> None:
        self.name = _name.encode()
        self.serial = _serial
        self.initial_psi = 0
        self.serial.timeout = 3  # set the serial read timeout.

    def get_data(self) -> tuple:
        """This function send the name of the sensors to Arduino and collect the data
        from that sensor through the Serial communication then format them."""

        raw_data = tuple(np.zeros(3) for _ in range(3))
        try:
            self.serial.write(self.name)
            self.serial.flush()
            line = self.serial.readline().decode()  # read the serial input as a string
            data = line.strip().split(',')  # split the serial reading with the ','
            if len(data) == 9:
                numeric_data = tuple(map(float, data))
                raw_data = (np.array(numeric_data[0:3]), np.array(numeric_data[3:6]), np.array(numeric_data[6:9]))
            else:
                raise Exception
        except Exception:
            self.get_data()

        return raw_data

    def calibrate(self, raw_data: tuple, acc_calib_matrices: tuple, gyro_calib_matrices: tuple,
                  magne_calib_matrices: tuple) -> tuple:
        """Calibrate the raw data according to pre-defines calibration matrices."""

        acc_calibrated_data = np.matmul(acc_calib_matrices[0], raw_data[0] - acc_calib_matrices[1])
        gyro_calibrated_data = np.matmul(gyro_calib_matrices[0], raw_data[1] - gyro_calib_matrices[1])
        magne_calibrated_data = np.matmul(magne_calib_matrices[0], raw_data[2] - magne_calib_matrices[1])
        return (acc_calibrated_data, gyro_calibrated_data, magne_calibrated_data)

    def angle_calculation(self, calibrated_data: tuple) -> tuple:
        """Calculate roll, pitch and yaw from calibrated data."""

        g, m = calibrated_data[0], calibrated_data[2]
        phi = np.arctan2(g[1], g[2])
        theta = np.arctan(-g[0] / (g[1] * np.sin(phi) + g[2] * np.cos(phi)))
        part1 = np.sin(phi) * m[2] - np.cos(phi) * m[1]
        part2 = np.cos(
            theta) * m[0] + np.sin(theta) * np.sin(phi) * m[1] + np.sin(theta) * np.cos(phi) * m[2]
        psi = np.arctan2(part1, part2)
        return (phi, theta, psi)
