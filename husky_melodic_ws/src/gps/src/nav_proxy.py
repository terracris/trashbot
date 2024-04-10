#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import String


# yup, the proxy should be in the husky motion node
class NavigationProxy():

    def __init__(self):
        # create the node
        rospy.init_node('navigation_proxy', anonymous=True)
        # wait for service to come alive
        rospy.wait_for_service('navigate')
        
        # create a callable service
        self.navigate = rospy.ServiceProxy('navigate', GetPlan) # this returns a pose stamped
        
        # the husky kinematics will call the the navigator to get a new points
        rospy.Subscriber('navigate_request', String, self.navigate)

    def navigate(self, request):
        # wait a second, shouldn't this proxy live in the husky


