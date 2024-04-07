#!/usr/bin/env python3

# this class will handle sending the robot commands to move to points
# I think this should be a service?
import rospy
from sensor_msgs.msg import NavSatFix
import nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from haversine import haversine


class Navigator:

    def __init__(self):
        rospy.init_node('navigator', anonymous=True)
        self.latitude = 0
        self.longitude = 0
        self.calibrate = True
        self.fixed_point = (0,0)
        rospy.Subscriber('gps_coordinates', NavSatFix, self.update_position)  # subscriber
        self.navigation_service = rospy.Service('navigate', GetPlan, self.calculate_navigation)  # service declaration

    # update our current position
    def update_position(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

        if self.calibrate:
            self.fixed_point = self.find_closest_point_for_reference()
            self.calibrate = False

    def find_closest_point_for_reference(self):
        closest_point = (0, 0)
        closest_distance = 1e100000
        with open('quad.txt', 'r') as file:
            for line in file:
                # locate the closest point to where we are currently
                p1_lat, p1_lon = map(float, line.strip().split(','))
                dist = haversine(self.latitude, self.longitude, p1_lat, p1_lon)
                if dist < closest_distance:
                    closest_point = (p1_lat, p1_lon)
                    closest_distance = dist

        return closest_point



    def calculate_navigation(self):
        pass



