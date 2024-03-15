# File: service_test/scripts/point_service_client.py
#!/usr/bin/env python3

import rospy
from service_test.srv import MyService
from geometry_msgs.msg import Point

def point_service_client():
    rospy.wait_for_service('my_service')
    try:
        my_service = rospy.ServiceProxy('my_service', MyService)
        point = Point()
        point.x = float(input("Enter x: "))
        point.y = float(input("Enter y: "))
        point.z = float(input("Enter z: "))
        response = my_service(point)
        rospy.loginfo("Service response: %s", response.success)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == "__main__":
    rospy.init_node('point_service_client')
    point_service_client()

