import bpy
import time
import math
import numpy as np
import serial
from Sensor import Sensor


ser = serial.Serial("/dev/cu.usbmodem14201", "115200")
x = [np.ones([3, 3]), np.ones(3)]
sensor = Sensor('1', ser, x, x, x)
# ser=serial.Serial("COM3","115200")
# sensor = Sensor()

time.sleep(3)

# creating var for nodes
larmR = bpy.data.objects['lowerarm.R']
uarmR = bpy.data.objects['upperarm.R']
# shoulderR=bpy.data.objects['shoulder.R']

n = 0
pos = [0, 0, 0, 0, 0, 0]
offset_temp = [0, 0, 0]
offset_uarmR = [0, 0, 0]
iter_1 = 10
byte = str.encode('1')


def get_angles(orient: np.array) -> np.array:
    phi = np.arctan2(orient[1], orient[2])
    theta = np.arctan(-orient[0] / (orient[1] * np.sin(phi) + orient[2] * np.cos(phi)))
    part1 = -np.cos(phi) * orient[7] + np.sin(phi) * orient[8]
    part2 = np.cos(theta) * orient[6] + np.sin(theta) * np.sin(phi) * orient[7] + np.sin(theta) * np.cos(phi) * orient[
        8]
    psi = np.arctan2(part1, part2)
    return np.array([phi, theta, psi])


# main loop

for _ in range(20):
    ser.write(byte)
    orient = ser.readline()
    time.sleep(0.01)
    sensor.get_data()
    time.sleep(0.01)
    # if orient=="":
    #    break

    orient = orient.decode().strip().split(',')
    if orient != "" and len(orient) == 9:
        orient = sensor.get_data()
        orient = sensor.calibrate()
        orient = sensor.angle_calculation(orient)
        # orient = np.array(sensor.calibrated_data).reshape(9)
        # orient=np.array(list(map(float, orient)))
        # orient=get_angles(orient)
        #       print(orient)
        #       larm.rotation_euler.x=(math.pi/2-math.pi*(orient[0])/180)
        #       larm.rotation_euler.y=(math.pi/2-math.pi*(orient[1])/180)
        #       larm.rotation_euler.z=0
        # orient[2]=0

        # DEG to RAD
        # orient=math.pi*(orient/180)

        # orient[0]=roll=x
        # orient[1]=pitch=y
        # orient[2]=yaw=z

        # Position calculation

        # larmR
        pos[0] = 1.07 * math.cos(orient[1]) * math.cos(orient[2])
        pos[1] = 1.34 * math.cos(orient[1]) * math.sin(orient[2])
        pos[2] = 1.07 * math.sin(orient[1])

        # uarmR
        #        pos[3]=1.34*math.cos(orient[4])*math.cos(orient[5])
        #        pos[4]=1.34*math.cos(orient[4])*math.sin(orient[5])
        #        pos[5]=1.34*math.sin(orient[4])

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

        file = open("/Users/prabodaratnayake/Desktop/motion_track/animation.txt", "a")
        file.write(str(larmR.location.x) + "," + str(larmR.location.y) + "," + str(larmR.location.z) + "\n")
        file.close()

    time.sleep(1 / 30)
    bpy.ops.wm.redraw_timer(type='DRAW_WIN_SWAP', iterations=1)

ser.close()

# while n==True:larm=
#       
#    #larm.location.x=x_displacment
#    #larm.location.y=y_displacment
#    #larm.location.z=z_displacment
#    
#    larm.rotation_euler.x=(math.pi*ax/180)
#    larm.rotation_euler.y=(math.pi*ay/180)
#    larm.rotation_euler.z=(math.pi*az/180)