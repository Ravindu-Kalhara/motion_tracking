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

        return arduino_port
    
    def kalmanFilter(self):
        pass

