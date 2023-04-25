from Essentials import Essentials
from sensor import Sensor

import bpy
import time
import math
import numpy as np
import serial

# initializing objects
serial_port = serial.Serial(Essentials.get_comport())

acc_cali_matrix1 =[np.array([[1.013323,-0.002636,0.000692],[-0.002636,1.008997,-0.002638],[0.000692,-0.002638,1.005562]]),
                    np.array([0.014765,0.014737,-0.037898]) ]

gyro_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]
magno_cali_matrix1 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]

sensor_1 = Sensor('1', serial_port, acc_cali_matrix1, gyro_cali_matrix1, magno_cali_matrix1)


acc_cali_matrix2 =[np.array([[1.013323,-0.002636,0.000692],[-0.002636,1.008997,-0.002638],[0.000692,-0.002638,1.005562]]),
                    np.array([0.014765,0.014737,-0.037898]) ]

gyro_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([-1.0,0.0,0.0])]
magno_cali_matrix2 =[np.array([[1.0,0,0],[0,1.0,0.0],[0.0,0.0,1.0]]), np.array([0.0,0.0,0.0])]

sensor_2 = Sensor('2', serial_port,acc_cali_matrix2, gyro_cali_matrix2, magno_cali_matrix2)


# calibrate objects
sensor_1.calibrate()
sensor_1.set_initial_psi(20)

sensor_2.calibrate()
sensor_2.set_initial_psi(20)

"""*************************** prabodha's code ********************************"""


#creating var for nodes
larmR=bpy.data.objects['lowerarm.R']
uarmR=bpy.data.objects['upperarm.R']
#shoulderR=bpy.data.objects['shoulder.R']

n=0
pos=[0,0,0,0,0,0]
offset_temp=[0,0,0]
offset_uarmR=[0,0,0]
iter_1=10
byte = str.encode('g\n')



"""*******************************************************************"""


while True:
    sensor_1.get_data()
    sensor_1.angle_calculation()
    sensor_1_angles = sensor_1.angles


    sensor_2.get_data()
    sensor_2.angle_calculation()
    sensor_2_angles = sensor_2.angles



#********************** **************************************************888
    # creating var for nodes

    orient =np.concatenate(sensor_1_angles, sensor_2_angles)

    orient = math.pi * (orient / 180)

    # Position calculation

    # larmR
    pos[0] = 1.07 * math.cos(orient[1]) * math.cos(orient[2])
    pos[1] = 1.34 * math.cos(orient[1]) * math.sin(orient[2])
    pos[2] = 1.07 * math.sin(orient[1])

    # uarmR
    #        pos[3]=1.34*math.cos(orient[4])*math.cos(orient[5])
    #        pos[4]=1.34*math.cos(orient[4])*math.sin(orient[5])
    #        pos[5]=1.34*math.sin(orient[4])

    # temp uarmR readings from larmR(remove this once implemented)
    pos[3] = 1.07 * math.cos(orient[1] * 2) * math.cos(orient[2])
    pos[4] = 1.34 * math.cos(orient[1]) * math.sin(orient[2])
    pos[5] = 1.07 * math.sin(orient[1] * 2)

    # Roll calculation & assignment
    larmR.rotation_euler.y = orient[0]
    #        larmR.rotation_euler.y=(math.pi/2-math.pi*(orient[1])/180)
    #        larmR.rotation_euler.z=0

    # Position assignment

    uarmR.location.x = offset_uarmR[0] = 0.561501 + pos[0]
    uarmR.location.y = offset_uarmR[1] = pos[1]
    uarmR.location.z = offset_uarmR[2] = 5.70503 + pos[2]

    larmR.location.x = offset_uarmR[0] + pos[3]
    larmR.location.y = offset_uarmR[1] + pos[4]
    larmR.location.z = offset_uarmR[2] + pos[5]
