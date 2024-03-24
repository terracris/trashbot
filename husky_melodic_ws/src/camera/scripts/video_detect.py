#!/usr/bin/env python
#coding=utf-8
#import roslib;roslib.load_manifest('beginner_tutorials')
import cv2
import detectros
import rospy
import numpy as np
##from beginner_tutorials.msg import Num
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PoseArray, Pose
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')
from cv_bridge import CvBridge 


class yolo_detect():
    def __init__(self):
        self.a = detectros.detectapi(weights='/home/trashbot/ros_cv_bridge/src/beginner_tutorials/scripts/weights/best.pt')
        im_sub = rospy.Subscriber('/camera/color/image_raw',Image, self.detectimg, queue_size = 1, buff_size=2**24)
        self.img_pub = rospy.Publisher('/cv/image_raw', Image, queue_size=1) 
        self.coordinate_pub = rospy.Publisher('/pixel_xy_coordinate',PoseArray, queue_size=1)
        self.bridge = CvBridge()
    def detectimg(self, img):
        frame = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        result, names, xy_coords = self.a.detect([frame])
        xy_coords_array = PoseArray()
        image_detect = result[0][0]
        names_detect = result[0][1]
        #print(names_detect)
        if names_detect:
            print(names[(names_detect[0][0])])
            # publish the poseArray

            for x_center, y_center in xy_coords:
                pose = Pose()
                pose.position.x = x_center  # Set x-coordinate
                pose.position.y = y_center  # Set y-coordinate
                xy_coords_array.poses.append(pose)
            
            self.coordinate_pub.publish(xy_coords_array)


        else:
            print("No object detected")

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(image_detect, "bgr8"))



if __name__ == '__main__':
    rospy.init_node("yolo_detect")
    rospy.loginfo("yolo_detect node started")
    yolo_detect()
    rospy.spin()
    
#    while True:
#    	rec, img = cap.read()
#    	result, names = a.detect([img])
#    	img = result[0][0]
#    	cv2.imshow("vedio", img)
#    	if cv2.waitKey(1)==27:
#    		break
