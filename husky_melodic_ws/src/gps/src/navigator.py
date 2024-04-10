#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from haversine import haversine, law_cosine_angle
from coordinate import GpsCoordinate
from std_msgs.msg import Float64
from math import cos, sin, radians
import tf


class Navigator:

    def __init__(self, list_map):

        self.map = list_map
        self.current_position = None
        self.calibrate = True
        self.reference_point = None
        self.starting_index = 0
        self.points_traveled = 0
        self.current_heading = 0
        self.x_pose = 0
        self.y_pose = 0
        rospy.init_node('navigator', anonymous=True)
        rospy.Subscriber('/gps_coordinates', NavSatFix,
                         self.update_position)  # subscriber
        rospy.Subscriber('imu/heading', Float64, self.update_heading)

        self.navigation_service = rospy.Service(
            'navigate', GetPlan, self.calculate_navigation)  # service declaration

    def update_position(self, msg):
        self.current_position = GpsCoordinate(msg.latitude, msg.longitude)
        if self.calibrate:
            self.starting_index, self.reference_point = self.find_closest_point(
                self.current_position)
            self.calibrate = False

    def update_heading(self, heading):
        # Ensure angle is in the range [0, 360)
        angle = heading.data % 360
        # Map angles from 0 to 180 to the range from 0 to -pi
        if angle == 0:
            self.current_heading = 0
        elif angle <= 180:
            self.current_heading = -radians(angle)
        # Map angles from 180 to 360 to the range from 0 to pi
        else:
            self.current_heading = radians(360 - angle)

    def calculate_navigation(self, req):
        self.points_traveled += 1

        # if we have traveled the full map, we should stop
        if self.points_traveled == len(self.map):
            empty_path = Path()
            empty_path.poses = []
            return empty_path

        next_point_idx = (self.starting_index +
                          self.points_traveled) % len(self.map)
        next_point = self.map[next_point_idx]
        oc = haversine(self.reference_point, self.current_position)
        op = haversine(self.reference_point, next_point)
        cp = haversine(self.current_position, next_point)
        phi = law_cosine_angle(cp, oc, op) # phi is in radians
        delta_theta = phi - self.current_heading

        pose = PoseStamped()
        negated_phi = -phi
        #self.x_pose += cp * cos(delta_theta) # cos and sin use radians
        #self.y_pose += cp * sin(delta_theta)
        orientation = Quaternion()
        q = tf.transformations.quaternion_from_euler(0, 0, negated_phi) # phi is in radians

        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        pose.pose.position.x = cp
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation

        pose_list = []
        pose_list.append(pose)

        path = Path()
        path.poses = pose_list

        response = GetPlanResponse()
        response.plan = path

        return response

    def find_closest_point(self, ref_coordinate):
        closest_point = None
        closest_distance = 9e1000  # big number!
        closest_index = None
        for idx, coordinate in enumerate(self.map):
            dist = haversine(ref_coordinate, coordinate)
            if dist < closest_distance:
                closest_point = coordinate
                closest_index = idx
                closest_distance = dist

        print(closest_point)

        return closest_index, closest_point

    @staticmethod
    def generate_map(map_filename):
        gps_map = []
        with open(map_filename, 'r') as file:
            for line in file:
                p_lat, p_lon = map(float, line.strip().split(','))
                coordinate = GpsCoordinate(p_lat, p_lon)
                gps_map.append(coordinate)

        return gps_map

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    map_file = 'quad.txt'
    quad_map = Navigator.generate_map(map_file)
    navigator = Navigator(quad_map)
    navigator.run()
