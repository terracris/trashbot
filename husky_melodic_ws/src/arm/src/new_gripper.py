#!/usr/bin/env python3

import serial
import time

class Gripper():

    def __init__(self):
        # Open serial port
        # TODO need to find out device number
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        self.state = 0  # 0 is open, 1 is closed
        # Wait for serial port to open
        time.sleep(2)

    # message is either open or close
    def communicate(self, msg):
            
        try:
            self.ser.write(msg.encode())
            print("Sent:", msg.strip())
            
            # Wait for a second
            time.sleep(1)
        except KeyboardInterrupt:
                self.ser.close()

if __name__ == '__main__':
    try:
        gripper = Gripper()

        while True:
            msg = input("open or close: ")
            gripper.communicate(msg)
    
    except KeyboardInterrupt:
         gripper.ser.close()

        
