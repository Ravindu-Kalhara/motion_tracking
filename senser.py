import numpy as np
import serial.tools.list_ports


def select_port(silence=True):
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



class Senser():
    def __init__(self,_name,_serial, gyro_cali_data:list):

        self.name = _name.encode('ascii')
        self.raw_data = np.zeros([3,3])
        self.calibrated_data = np.zeros([3,3])
        self.angles = np.zeros([3])
        self.serial = _serial
        self.gyro_cali_data= np.array([[0,0,0],gyro_cali_data,[0,0,0]]) # set gyro calibrated data to (3,3) array it should be this shape [1,3,2]



    def get_data(self):
        """ this function send the name of the sensors to collect the data and read the bytes """
        self.serial.write(self.name)
        line = self.serial.readline()
        self.convert_data(line)

    def convert_data(self,line:str):

        data =[float(i) for i in line.split(',')] # split the serial reading with the ','
        self.raw_data = np.array(data).reshape(3, 3) # set the readings to raw data format

    def gyroscope_calibration(self):
        self.raw_data += self.gyro_cali_data  # add calibrated giro values to row data sine other are Zeros it would not affect










