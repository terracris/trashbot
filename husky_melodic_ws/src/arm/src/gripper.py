import serial
import time

class Gripper():

    def __init__(self):
        # Open serial port
        self.ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)
        # by default we start open
        self.state = 0  # 0 is open, 1 is closed
        # Wait for serial port to open
        time.sleep(2)

    def open(self):
        try:
            # if we are already open we do not need to do anything
            if self.state == 0:
                return
            
            msg = "open"
            self.ser.write(msg.encode())
            print("Sent:", msg.strip())
            
            # Wait for a second
            time.sleep(1)
        except KeyboardInterrupt:
                self.ser.close()

    def close(self):
        try:
            # if we are already closed we do not need to do anything
            if self.state == 1:
                return
            msg = "close"
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
            if msg == "open":
                gripper.open()
            elif msg == "close":
                gripper.close()
    
    except KeyboardInterrupt:
         gripper.ser.close()

        