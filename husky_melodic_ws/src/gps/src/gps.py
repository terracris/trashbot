#!/usr/bin/env python3

import rospy
import serial
import time
import pynmea2
from sensor_msgs.msg import NavSatFix

def start_gps():
    
    rospy.init_node('gps')
    pub = rospy.Publisher("gps_coordinates", NavSatFix ,queue_size=10)
    
    # Define the serial port and baud rate
    serial_port = "/dev/ttyACM0"  # Use the correct serial port
    baud_rate = 9600

    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    try:
        while True:
            # Read a line from the GPS module
            try:
                gps_data = ser.readline().decode("ascii", errors='ignore').strip() # convert byte string to ascii
            except UnicodeError:
                continue
            # Check if the line is not empty
            if gps_data.startswith("$"):
                try:
                    msg = pynmea2.parse(gps_data) # parse the line from gps module
                    if isinstance(msg, pynmea2.types.talker.RMC):
                        
                        latitude = msg.latitude
                        longitude = msg.longitude
                    
                        curr_location = NavSatFix()
                        curr_location.latitude = latitude
                        curr_location.longitude = longitude
                        curr_location.altitude = 0 # we have no way to tell this yet. right?
                       
                        #print(f'Latitude: {latitude}, Longitude: {longitude}')
                        pub.publish(curr_location)  # publish the current gps location -> need path planner
                        time.sleep(0.5)  # only delay when you have to publish the data
                    
                except pynmea2.ParseError as e:
                    pass
                    #print(f'Parse error: {e}')

    except KeyboardInterrupt:
        # Close the serial port when the script is interrupted
        ser.close()


if __name__ == '__main__':
    start_gps()

