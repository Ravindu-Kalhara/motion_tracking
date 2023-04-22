import numpy as np
import serial


class Sensor():
    def __init__(self, _name: str, _serial: serial.Serial, _accel_calibration_matrix: np.array, _gyro_calibration_matrix: np.array, _magne_calibration_matrix: np.array) -> None:
        self.name = _name
        self.raw_data = np.zeros([3, 3])
        self.calibrated_data = np.zeros([3, 3])
        self.angles = np.zeros([3])
        self.serial = _serial
        self.accel_calibration_matrix = _accel_calibration_matrix
        self.gyro_calibration_matrix = _gyro_calibration_matrix
        self.magne_calibration_matrix = _magne_calibration_matrix
        self.initial_psi = 0

    def set_initial_psi(self, sample_size: int) -> None:
        """Get the initial yaw rotaion. Acoording to the 'sample_size', take that number 
        readings and get mean of those readings, take that as the initial yaw position."""

        initail_psi_sum = 0
        for i in range(sample_size):
            self.get_data()
            self.calibrated_data()
            self.angle_calculation()
            initail_psi_sum += self.angles[2]
        self.initial_psi = initail_psi_sum/sample_size

    def get_data(self) -> None:
        """This function send the name of the sensors to Arduino and collect the data
        from that sensor through the Serial communication."""

        self.serial.write(self.name)
        # read data until the end line character comes and decode bytes to str
        line = self.serial.readline().decode()
        self.__convert_data(line)

    def __convert_data(self, line: str) -> None:
        """Convert recieved data from the sensor, into 3x3 matix containg tree componets
        of ecah gravitaional acceleration, angular velocity around three axis and geomagnetic
        field strenght."""

        data = line.strip().split(',')  # split the serial reading with the ','
        numeric_data = []
        for i in data:
            try:
                numeric_data.append(float(i))
            except:
                pass
        if len(numeric_data) == 9:  # check the data has numbers
            # set the readings to raw data format
            self.raw_data = np.array(numeric_data).reshape(3, 3)
        else:
            print("*********** Wrong data type ***********")

    def calibrate(self) -> None:
        """Calibrate the raw data according to pre-defines calibration matrix."""

        acc_calibrated_data = self.accel_calibration_matrix[0]*(self.raw_data[0] - self.accel_calibration_matrix[1])
        gyro_calibrated_data = self.gyro_calibration_matrix[0]*(self.raw_data[1] - self.gyro_calibration_matrix[1])
        magne_calibrated_data = self.magne_calibration_matrix[0]*(self.raw_data[2] - self.magne_calibration_matrix[1])
        self.calibrated_data = np.stack((acc_calibrated_data, gyro_calibrated_data, magne_calibrated_data))

    def angle_calculation(self) -> None:
        """Calculate roll, pitch and yaw from calibrated data."""

        phi = np.arctan2(self.calibrated_data[0, 1], self.calibrated_data[0, 2])
        theta = np.arctan(self.calibrated_data[0, 0], (self.calibrated_data[0, 1]*np.sin(phi) + self.calibrated_data[0, 2]*np.cos(phi)))
        part1 = -np.cos(phi)*self.calibrated_data[2, 1] + np.sin(phi)*self.calibrated_data[2, 2]
        part2 = -np.cos(theta)*self.calibrated_data[2, 0] + np.sin(phi)*np.sin(theta)*self.calibrated_data[2, 1] + np.sin(theta)*np.sin(phi)*self.calibrated_data[2, 2]
        psi = np.arctan2(part1, part2) - self.initial_psi
        self.angles = np.array([phi, theta, psi])
