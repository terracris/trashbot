#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import String

class NavigationProxy():

    def __init__(self):
        rospy.init_node('navigation_proxy', anonymous=True)
        rospy.wait_for_service('navigate')
        self.navigate = rospy.ServiceProxy('navigate', GetPlan)
        rospy.Subscriber('navigate_request', String, self.navigate)

    def navigate(self, request):
        
