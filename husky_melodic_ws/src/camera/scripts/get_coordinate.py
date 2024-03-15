#!/usr/bin/env python
import rospy
import pyrealsense2
import numpy
from realsense_depth import *
from geometry_msgs.msg import Point



class coordinate_all():
    def __init__(self):
        xy_pixel_sub = rospy.Subscriber('/pixel_xy_coordinate', Point, self.calc_coordinate)

        self.final_coordinate_pub = rospy.Publisher('/object_coordinate',Point, queue_size=1)
        self.dc = DepthCamera()


    def calc_coordinate(self, point):
        x_pixel = point.x
        y_pixel = point.y
        x_pixel = 50
        y_pixel = 200
        ret, depth_frame, color_frame = self.dc.get_frame()
        distance = depth_frame[x_pixel, y_pixel]
        print(distance)
        #might need to change the order of x and y?
        point3d = self.dc.pixel_to_point(depth_frame, [x_pixel, y_pixel])
        object_coord = Point()
        object_coord.x = point3d[0]
        object_coord.y = point3d[1]
        object_coord.z = point3d[2]
        self.final_coordinate_pub.publish(object_coord)



if __name__ == '__main__':
    rospy.init_node("coordinate_all")
    coordinate_all()
    rospy.spin()

