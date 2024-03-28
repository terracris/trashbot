import rospy
import serial
import time
import pynmea2
from std_msgs.msg import String

def start_gps():
    
    rospy.init_node('gps')
    pub = rospy.Publisher("gps_coordinates", String ,queue_size=10)
    
    # Define the serial port and baud rate
    serial_port = "/dev/ttyTHS1"  # Use the correct serial port
    baud_rate = 9600

    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate, timeout=1)

    try:
        while True:
            # Read a line from the GPS module
            gps_data = ser.readline().decode('utf-8').strip()
            print(gps_data)
            # Check if the line is not empty
            if gps_data:
                # Process the GPS data as needed
                print(gps_data)
                # Parse NMEA data
                msg = pynmea2.parse(gps_data)
                if isinstance(msg, pynmea2.types.talker.GNS):
                    # Process GNS messages
                    latitude = msg.latitude
                    longitude = msg.longitude
                    print("Latitude:", latitude, "Longitude:", longitude)

        time.sleep(0.5)  # Adjust the delay based on your requirements

    except KeyboardInterrupt:
        # Close the serial port when the script is interrupted
        ser.close()


if __name__ == '__main__':
    start_gps()