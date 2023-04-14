STD_DEV_GYRO = 8   # This is the standard deviation of the gyroscope readings. Change this manually to reduce the uncertainity of the measurement.
STD_DEV_ACC = 2    # This is the standard deviation of the accelerometer readings. Change this manually to reduce the uncertainity of the measurement.
DELTA_T = 0.01

def kalmanFilter(kalmanAng, kalmanInput, kalmanMeasurement, ):
    # 1D Kalman filter implementation
    # kalmanInput = gyro reading, kalmanMeasurement = tilt angle by the acceleration

    kalmanAng[0] = kalmanAng[0] + DELTA_T * kalmanInput                                   # prediction with gyro
    kalmanAng[1] = kalmanAng[1] + DELTA_T * DELTA_T * STD_DEV_GYRO * STD_DEV_GYRO          # uncertainty of prediction
    kalmanGain = kalmanAng[1] * 1 / (1 * kalmanAng[1] + STD_DEV_ACC * STD_DEV_ACC)         # kalman gain
    kalmanAng[0] = kalmanAng[0] + kalmanGain * (kalmanMeasurement - kalmanAng[0])          # new prediction with tilt angle by acceleration associated with kalman gain
    kalmanAng[1] = (1 - kalmanGain) * kalmanAng[1]                                        # new uncertainty prediction

    return kalmanAng
