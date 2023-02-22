import os
import time
import sys, traceback
import rospy
from serial.serialutil import SerialException
from serial import Serial
from std_msgs.msg import UInt16MultiArray

# TODO: cite this as from arduino_driver.py
def recv(port, timeout=0.5):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = ''
        value = ''
        attempts = 0
        interCharTimeout = timeout / 30
        while c != '\r':
            c = port.read(1)
            value += c
            attempts += 1
            if attempts * interCharTimeout > timeout:
                return None

        value = value.strip('\r')

        return value

def serial_read():
    try:
        serial_pub = rospy.Publisher("/serial_data", UInt16MultiArray, latch=True, queue_size=5)
    except:
        pass
    value = recv(port, 0.5)
    print(value)
    parsed = value.split(' ')
    data_vals = [int(val) for val in parsed[1:]]
    msg = UInt16MultiArray(data=data_vals)
    serial_pub.publish(msg)

if __name__ == '__main__':
    print("peach")
    try: 
        rospy.init_node("serial_read")
    except:
        pass
    try:
        port = Serial(port="/dev/ttyACM0", baudrate=9600, timeout=1.0, writeTimeout=1.0)
        time.sleep(1)
        print("Arduino is ready")
    except:
        print("error connecting to arduino")
    while (1):
        serial_read()
        rospy.sleep(0.01)
