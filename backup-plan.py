import serial
import numpy as np

def angle_calculation(data):
    """Calculate roll, pitch and yaw from calibrated data.
    data= [
    [1,2,3],
    [1,2,3],
    [1,2,3]
    ]
    """

    phi = np.arctan2(data[0,1], data[0,2])
    theta = np.arctan([data[0,0] / data[0,1] * np.sin(phi) + data[0,2] * np.cos(phi)])
    part1 = -np.cos(phi) * data[2,1] + np.sin(phi) * data[2,2]
    part2 = -np.cos(theta) * data[2,0] + np.sin(phi) * np.sin(theta) * data[2,1] + np.sin(theta) * np.sin(phi) * data[2,2]
    psi = np.arctan2(part1, part2)
    return [phi,theta, psi]

ser = serial.Serial('COM8')

##################################################################

line = ser.readline().strip().decode()

data = line.split(',')

numeric_data = []
for i in data:
    try:
        numeric_data.append(float(i))
    except Exception as e:
        pass


numeric_data_nested =[numeric_data[:3],numeric_data[3:6], numeric_data[6:]]
angles=angle_calculation(numeric_data_nested)
print(angles)

