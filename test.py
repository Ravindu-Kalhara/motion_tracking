import serial
import numpy as np
from Essentials import Essentials
from sensor import Sensor
import time
"""
def convert_data(line: str):
    data = line.split(',')  # split the serial reading with the ','
    numeric_data = []
    for i in data:
        try:
            numeric_data.append(float(i))
        except:
            pass

    if len(numeric_data) == 9:  # check the data has numbers
        numeric_data = np.array(numeric_data).reshape(3, 3)  # set the readings to raw data format
        print(numeric_data)
        print(numeric_data[0,0]+3)
    else:
        print('********** Wrong data type ***********')


line = "1,2,3,4,-5,6,7,8,9"
convert_data(line)


def calibrate(self):
    
    acc_calibrated_data = self.accel_calibration_matrix[0] * (self.raw_data[0] - self.accel_calibration_matrix[1])
    gyro_calibrated_data = self.gyro_calibration_matrix[0] * (self.raw_data[1] - self.gyro_calibration_matrix[1])
    magne_calibrated_data = self.magne_calibration_matrix[0] * (self.raw_data[2] - self.magne_calibration_matrix[1])
    calibrated_data = np.stack((acc_calibrated_data, gyro_calibrated_data, magne_calibrated_data))
    print(calibrated_data)


class Mock:
    def __init__(self):
        self.accel_calibration_matrix = [np.identity(3), np.array([1,2,3])]
        self.gyro_calibration_matrix = [np.identity(3), np.array([1, 2, 3])]
        self.magne_calibration_matrix = [np.identity(3), np.array([1, 2, 3])]
        self.raw_data = np.array([[7,8,9],[4,5,6],[1,2,3]])
        self.calibrated_data=np.array([[0.442,0.789,0.17854],[4,5,6],[1,2,3]])

mock =Mock()
calibrate(mock)


def roll_pitch_calculation(self):
    phi = np.arctan2(self.calibrated_data[0, 1], self.calibrated_data[0, 2])
    theta = np.arctan(self.calibrated_data[0, 0] / (
                self.calibrated_data[0, 1] * np.sin(phi) + self.calibrated_data[0, 2] * np.cos(phi)))
    print(phi,theta)


roll_pitch_calculation(mock)
"""

# check part by part
# initializing objects
serial_port = serial.Serial(Essentials.get_comport(), 115200)
serial_port.timeout=1


acc_cali_matrix1 =[np.array([[1.003192, 0.002244, -0.002029],[0.002244, 1.000812, -0.002718],[-0.002029, -0.002718, 1.007279]]),
                    np.array([0.157448, 0.181007, -0.172725])]

gyro_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]

magno_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]
sensor_1 = Sensor('1', serial_port, acc_cali_matrix1, gyro_cali_matrix1, magno_cali_matrix1)



acc_cali_matrix2 =[np.array([[1.010752, 0.036523, 0.001839],[0.036523, 0.996652, 0.003651],[0.001839, 0.003651, 0.985828],]),
                    np.array([0.364060, -0.155569, -0.120483]) ]

gyro_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]
magno_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]

sensor_2 = Sensor('2', serial_port,acc_cali_matrix2, gyro_cali_matrix2, magno_cali_matrix2)



"""
sensor_1.get_data()
sensor_1.angle_calculation()
sensor_1_angles = sensor_1.angles
print('___________________________________________________________')
print(sensor_1_angles)

"""
while True:
    sensor_1.get_data()
    sensor_1.calibrate()
    # print(sensor_1.calibrated_data)
    sensor_1.angle_calculation()


    sensor_2.get_data()
    sensor_2.calibrate()
    sensor_2.angle_calculation()
    sensor_2_angles = sensor_2.angles

    print('***********************************************')
    print(sensor_2.angles, sensor_1.angles)
    print('***********************************************')
