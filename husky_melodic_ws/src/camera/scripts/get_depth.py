#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion, PoseArray
from std_msgs.msg import Header
import numpy as np



class pixel2depth():
    def __init__(self):
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.convert_depth_image, queue_size=1)
        rospy.Subscriber('/pixel_xy_coordinate', PoseArray, self.find_the_closest_xy)

        self.final_coordinate_pub = rospy.Publisher('/3d_coordinate',PoseStamped, queue_size=1)

        self.depth_array = []
        self.bridge = CvBridge()
        self.x_center = 0
        self.y_center = 0
        self.ppx = 322.2703857
        self.ppy = 250.9405212
        self.fx = 607.4005127
        self.fy = 607.2220459

    def find_the_closest_xy(self, xy_PoseArray):
        smallest_depth_value = 10000000
        xy_list = []
        for pose in xy_PoseArray.poses:
            xy_list.append([int(pose.position.x), int(pose.position.y)])
        
        if xy_list != []:
            for x_center, y_center in xy_list:
                print(y_center, x_center)
                depth_value = self.depth_array[y_center, x_center]
                print(depth_value)
                if depth_value < smallest_depth_value:
                    smallest_depth_value = depth_value
                    best_x_center = x_center
                    best_y_center = y_center
        else:
            print("nothing in xy_list!!!!!!")

        if smallest_depth_value != 0:
            x_coord = smallest_depth_value*(best_x_center+4 - self.ppx)/self.fx
            y_coord = smallest_depth_value*(best_y_center+8 - self.ppy)/self.fy
            z_coord = smallest_depth_value
            object_coord = PoseStamped()

            object_coord.header = Header()
            object_coord.header.stamp = rospy.Time.now()
            object_coord.header.frame_id = "3d_coordinate"

            object_coord.pose = Pose()
            object_coord.pose.position = Point()
            object_coord.pose.orientation = Quaternion()

            object_coord.pose.position.x = x_coord/1000 # unit: m
            object_coord.pose.position.y = y_coord/1000 # unit: m
            object_coord.pose.position.z = z_coord/1000 # unit: m

            object_coord.pose.orientation.x = 0.0
            object_coord.pose.orientation.y = 0.0
            object_coord.pose.orientation.z = 0.0
            object_coord.pose.orientation.w = 0.0

            print("x coord:", x_coord/1000, "m")
            print("y coord:", y_coord/1000, "m")
            print("z coord:", z_coord/1000, "m")
            print("===============")
            self.final_coordinate_pub.publish(object_coord)


    def convert_depth_image(self, ros_image):
         # Use cv_bridge() to convert the ROS image to OpenCV format
         #Convert the depth image using the default passthrough encoding
        depth_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding='passthrough')

         #Convert the depth image to a Numpy array
        self.depth_array = np.array(depth_image, dtype=np.float32)


if __name__ == '__main__':
    rospy.init_node('pixel2depth',anonymous=True)
    pixel2depth()
    rospy.spin()
