import numpy as np
import serial


class Sensor():
    def _init_(self, _name: str, _serial: serial.Serial, _accel_calibration_matrix: np.array,
               _gyro_calibration_matrix: np.array, _magne_calibration_matrix: np.array):
        self.name = _name.encode()
        self.raw_data = np.zeros([3, 3])
        self.calibrated_data = np.zeros([3, 3])
        self.angles = np.zeros([3])
        self.serial = _serial
        self.accel_calibration_matrix = _accel_calibration_matrix
        self.gyro_calibration_matrix = _gyro_calibration_matrix
        self.magne_calibration_matrix = _magne_calibration_matrix
        self.initial_psi = 0

    def set_initial_psi(self, sample_size=10) -> None:
        """Get the initial yaw rotaion. Acoording to the 'sample_size', take that number
        readings and get mean of those readings, take that as the initial yaw position."""

        initail_psi_sum = 0
        for i in range(sample_size):
            self.get_data()
            self.calibrate()
            self.angle_calculation()
            initail_psi_sum += self.angles[2]
        self.initial_psi = initail_psi_sum / sample_size

    def get_data(self):
        """This function send the name of the sensors to Arduino and collect the data
        from that sensor through the Serial communication."""

        try:
            self.serial.write(self.name)
            line = self.serial.readline().decode()
        except Exception as e:
            line = ''

        self.convert_data(line)
        return self.raw_data.reshape(9)

    def convert_data(self, line: str):
        """Convert recieved data from the sensor, into 3x3 matix containg tree componets
        of each gravitaional acceleration, angular velocity around three axis and geomagnetic
        field strenght."""

        data = line.strip().split(',')  # split the serial reading with the ','
        numeric_data = []
        for i in data:
            try:
                numeric_data.append(float(i))
            except Exception as e:
                pass

        if len(numeric_data) == 9:  # check the data has numbers
            self.raw_data = np.array(numeric_data).reshape(3, 3)
        else:
            print("**** Wrong data type ****")

    def calibrate(self):
        """Calibrate the raw data according to pre-defines calibration matrix."""

        acc_calibrated_data = np.matmul(self.accel_calibration_matrix[0],
                                        self.raw_data[0] - self.accel_calibration_matrix[1])
        gyro_calibrated_data = np.matmul(self.gyro_calibration_matrix[0],
                                         self.raw_data[1] - self.gyro_calibration_matrix[1])
        magne_calibrated_data = np.matmul(self.magne_calibration_matrix[0],
                                          self.raw_data[2] - self.magne_calibration_matrix[1])
        self.calibrated_data = [acc_calibrated_data, gyro_calibrated_data, magne_calibrated_data]
        return np.array(self.calibrated_data).reshape(9)

    def angle_calculation(self, orient):
        """Calculate roll, pitch and yaw from calibrated data."""
        phi = np.arctan2(orient[1], orient[2])
        theta = np.arctan(-orient[0] / (orient[1] * np.sin(phi) + orient[2] * np.cos(phi)))
        part1 = -np.cos(phi) * orient[7] + np.sin(phi) * orient[8]
        part2 = np.cos(theta) * orient[6] + np.sin(theta) * np.sin(phi) * orient[7] + np.sin(theta) * np.cos(phi) * \
                orient[8]
        psi = np.arctan2(part1, part2)
        self.angles = np.array([phi, theta, psi])
        return self.angles