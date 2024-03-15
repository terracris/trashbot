#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

class ArmCamProxy:

	def __init__(self):
		rospy.init_node("arm_cam_proxy", anonymous=True)

		# subscribe to the camera point topic
		rospy.Subscriber('/3d_coordinate, Point, self.move_to)
		

		# service client for requesting trajectory planning

		self.arm_service = rospy.ServiceProxy('plan_path', GetPlan)

		# variables to store start and end goal
		self.end_goal = None
		self.occupied = False

	def move_to(self, point):
		# check if the point is within our workspace --> requires knowledge of where the husky is and if we can actually grab it

		if(self.in_workspace(point)) and not self.occupied:
			self.end_goal = point
			self.occupied = True

		# are we homing every time we have to pick something up?

		request = GetPlanRequest()
		request.start = None  # if None, we will make the arm home first
		request.goal = self.end_goal

		path = self.arm_service(request)

		rospy.loginfo("Path successfully requested")
		
		# reset end point
		self.end_goal = None

	def run(self):
		rospy.spin()	

if __name__ == '__main__':
	listener = ArmCamProxy()
	listener.run()
