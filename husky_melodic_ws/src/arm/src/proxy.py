#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan, GetPlanRequest

class ArmCamProxy:

    def __init__(self):
        # create the node
        rospy.init_node("arm_cam_proxy", anonymous=True)

        # subscribe to the camera point topic
        rospy.Subscriber('/3d_coordinate', PoseStamped, self.move_to)

        # service client for requesting trajectory planning
        self.arm_service = rospy.ServiceProxy('manipulate', GetPlan)

        # variables to store start and end goal
        self.end_goal = None
        self.occupied = False

    def move_to(self, point):
        # check if the point is within our workspace --> requires knowledge of where the husky is and if we can actually grab it

        # need to create a check if that is in the workspace
        # that will be handled by the service. if the pose is not
        # in the workspace, it will return an empty path

        rospy.wait_for_service('manipulate')
                
        print("service is active")
        if not self.occupied:
            self.end_goal = point
            self.occupied = True

        # are we homing every time we have to pick something up?

        request = GetPlanRequest()
        print("making request")
        request.start = PoseStamped()  # None, we will make the arm home first
        request.goal = self.end_goal # poseStamped object

        path = self.arm_service(request)
        print(path)

        rospy.loginfo("Path successfully requested")
        
        # reset end point and occupied
        self.end_goal = None
        self.occupied = False

    def run(self):
        rospy.spin()    

if __name__ == '__main__':
    listener = ArmCamProxy()
    listener.run()

