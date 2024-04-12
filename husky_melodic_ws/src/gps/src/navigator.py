#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from haversine import haversine, law_cosine_angle
from coordinate import GpsCoordinate
from std_msgs.msg import Float64, String
from math import cos, sin, radians, pi
import tf


class Navigator:

    def __init__(self, list_map):

        self.map = list_map
        self.current_position = None
        self.calibrate_fixed_position = True
        self.reference_point = None
        self.z_reference_point = None
        self.calibrate_z_position = True
        self.starting_point = None
        self.starting_index = 0
        self.points_traveled = 0
        self.current_heading = 0
        rospy.init_node('navigator', anonymous=True)
        rospy.Subscriber('/gps_coordinates', NavSatFix,
                         self.update_position)  # subscriber

        self.navigation_service = rospy.Service(
            'navigate', GetPlan, self.calculate_navigation)  # service declaration
        
        self.previous_position = None

    def update_position(self, msg):
        self.current_position = GpsCoordinate(msg.latitude, msg.longitude)
        
        if self.calibrate_fixed_position:
            self.reference_point = self.current_position
            self.calibrate_fixed_position = False
            print("fixed position", self.reference_point)


    def calculate_first_point(self):
        
        """
        a: oc --> fixed reference point to current position (z reference)
        b: cp --> current position (z reference) to desired point (starting point)
        c: op --> fixed reference point to desired point
        """
        
        self.starting_index, self.starting_point = self.find_closest_point(self.current_position)
        print(self.starting_index)
        next_point = self.map[self.starting_index]

 
        # i want to remove this z point. can just current position
        oc = haversine(self.reference_point, self.current_position)
        cp = haversine(self.current_position, next_point)
        op = haversine(self.reference_point, next_point)
        
        C = law_cosine_angle(oc, cp, op) # radians
        turning_angle = pi - C
        pose = PoseStamped()
        orientation = Quaternion()
        q = tf.transformations.quaternion_from_euler(0, 0, turning_angle) # turning_angle is in radians

        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        pose.pose.position.x = cp
        print(pose.pose.position.x)
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation

        pose_list = []
        pose_list.append(pose)

        path = Path()
        path.poses = pose_list

        response = GetPlanResponse()
        response.plan = path

        self.previous_position = self.current_position

        return response


    def calculate_navigation(self, req):
        
        if self.points_traveled == 0:
            self.points_traveled += 1
            return self.calculate_first_point()
 

        # if we have traveled the full map, we should stop
        if self.points_traveled == len(self.map):
            empty_path = Path()
            empty_path.poses = []
            return empty_path

        next_point_idx = (self.starting_index +
                          self.points_traveled) % len(self.map)
        next_point = self.map[next_point_idx]
        """
        a: oc --> fixed reference point to current position (z reference)
        b: cp --> current position (z reference) to desired point (starting point)
        c: op --> fixed reference point to desired point
        """
 
        # desired triangle
        curr_oc = haversine(self.reference_point, self.current_position)
        curr_op = haversine(self.reference_point, next_point)
        curr_cp = haversine(self.current_position, next_point)
        
        # previous triangle
        prev_oc = haversine(self.reference_point, self.previous_position)
        prev_op = haversine(self.reference_point, self.current_position)
        prev_cp = haversine(self.previous_position, self.current_position)


        # TODO change order
        desired_c =  law_cosine_angle(curr_oc, curr_cp, curr_op) # radians
        previous_a = law_cosine_angle(prev_cp, prev_op, prev_oc) # radians
        turning_angle = 180 - desired_c - previous_a

        pose = PoseStamped()
        orientation = Quaternion()
        q = tf.transformations.quaternion_from_euler(0, 0, turning_angle) # phi is in radians

        orientation.x = q[0]
        orientation.y = q[1]
        orientation.z = q[2]
        orientation.w = q[3]

        pose.pose.position.x = curr_cp
        print(pose.pose.position.x)
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.0
        pose.pose.orientation = orientation

        pose_list = []
        pose_list.append(pose)

        path = Path()
        path.poses = pose_list

        response = GetPlanResponse()
        response.plan = path
        
        self.points_traveled += 1
        self.previous_position = self.current_position

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
