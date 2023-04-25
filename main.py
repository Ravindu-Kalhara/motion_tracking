from Essentials import Essentials
from sensor import Sensor

import bpy
import time
import math
import numpy as np
import serial

# initializing objects
serial_port = Essentials.get_comport()

acc_cali_matrix1 =[np.array([[1.013323,-0.002636,0.000692],[-0.002636,1.008997,-0.002638],[0.000692,-0.002638,1.005562]]),
                    np.array([0.014765,0.014737,-0.037898]) ]

gyro_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]
magno_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]

sensor_1 = Sensor('1', serial.Serial(serial_port), acc_cali_matrix1, gyro_cali_matrix1, magno_cali_matrix1)


acc_cali_matrix2 =[np.array([[1.013323,-0.002636,0.000692],[-0.002636,1.008997,-0.002638],[0.000692,-0.002638,1.005562]]),
                    np.array([0.014765,0.014737,-0.037898]) ]

gyro_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]
magno_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]

sensor_2 = Sensor('2', serial.Serial(serial_port),acc_cali_matrix2, gyro_cali_matrix2, magno_cali_matrix2)


# calibrate objects
sensor_1.calibrate()
sensor_1.set_initial_psi(20)

sensor_2.calibrate()
sensor_2.set_initial_psi(20)


while True:
    sensor_1.get_data()
    sensor_1.angle_calculation()
    sensor_1_angles = sensor_1.angles


    sensor_2.get_data()
    sensor_2.angle_calculation()
    sensor_2_angles = sensor_2.angles



""" *********************** **************************************************888"""

