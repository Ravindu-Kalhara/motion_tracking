
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



