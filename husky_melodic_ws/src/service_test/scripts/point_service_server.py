# File: service_test/scripts/point_service_server.py
#!/usr/bin/env python3

import rospy
from service_test.srv import MyService
from geometry_msgs.msg import Point

def handle_point_service(req):
    rospy.loginfo("Received point: %s", req.point)
    # Process the received point
    return True

def point_service_server():
    rospy.init_node('point_service_server')
    rospy.Service('my_service', MyService, handle_point_service)
    rospy.loginfo("Point service ready.")
    rospy.spin()

if __name__ == "__main__":
    point_service_server()

