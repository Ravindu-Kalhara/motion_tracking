import serial.tools.list_ports


class Essentials:
    def __init__(self) -> None:
        pass

    def get_comport(self, silence=True) -> None:
        """ this function returns the arduino com-port """

        # Get a list of all available serial ports
        available_ports = list(serial.tools.list_ports.comports())

        # If no serial ports are found, print an error message and exit the program
        if len(available_ports) == 0:
            print("No available serial ports found.")
            exit()
        if not silence:
            # Print the available serial ports to the console
            print("Available serial ports:")
            for port in available_ports:
                print(port.device)

        # Choose the Arduino serial port
        arduino_port = None
        for port in available_ports:
            if "Arduino" in port.description:
                arduino_port = port.device
                break

        # If an Arduino serial port is not found, print an error message and exit the program
        if arduino_port is None:
            print("Arduino not found on any serial port.")
            exit()

        # Print the chosen Arduino serial port to the console
        print("Arduino found on serial port:", arduino_port)
        return arduino_port  # type: ignore

    def kalman_filter(self, kalmanAng: list, kalmanInput: float, kalmanMeasurement: float,
                      std_dev_gyro=8.0, std_dev_acc=2.0, delta_t=0.01) -> list:
        # 1D Kalman filter implementation
        # kalmanAngle = [angle prediction, unsertainity of prediction]
        # kalmanInput = gyro reading, kalmanMeasurement = calculated angle by the acceleration / magnetometer

        kalmanAng[0] = kalmanAng[0] + delta_t * kalmanInput  # prediction angle
        kalmanAng[1] = kalmanAng[1] + (delta_t ** 2) * (std_dev_gyro ** 2)  # uncertainty of angle prediction
        kalmanGain = kalmanAng[1] / (kalmanAng[1] + std_dev_acc ** 2)  # kalman gain
        kalmanAng[0] = kalmanAng[0] + kalmanGain * (kalmanMeasurement - kalmanAng[0])  # new angle prediction
        kalmanAng[1] = (1 - kalmanGain) * kalmanAng[1]  # new uncertainty prediction
        return kalmanAng

