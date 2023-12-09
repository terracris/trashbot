#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from math import sin, cos, atan2
from decimal import Decimal, getcontext

# We need to increase the accuracy of my values to improve the accuracy of my tolerances and odometry

getcontext().prec = 10  # 10 decimal place precision

class HuskyOdometry:
    # velocity threshold for determining equivalence. 
    # currently set to 0.5 cm per second
    VELOCITY_THRESHOLD = 0.005

    def __init__(self):
        rospy.init_node('a100_odometry', anonymous=True)

        # initialize variables
        # pose of the robot
        self.px = 0
        self.py = 0
        self.pth = 0
        self.last_timestamp = rospy.Time.now()
        self.odom_message_count = 0
        
        # Husky A100 parameters [m]
        self.wheel_seperation = 0.50 # number retrieved from manual on the A100
        self.wheel_radius = 0.1143  # wheels are 9 inches in Diameter

        # subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # publishers
        self.odometry_pub = rospy.Publisher('a100/odometry', Odometry, queue_size=10)

    def joint_states_callback(self, joint_states):
        current_timestamp = joint_states.header.stamp

        # Calculate time interval
        delta_t = (current_timestamp - self.last_timestamp).to_sec()

        # Extract angular positions and velocities
        left_motor_angular_velocity = joint_states.velocity[0]
        right_motor_angular_velocity = joint_states.velocity[1]

        # Convert motor angular velocity to linear velocity for each wheel
        left_wheel_linear_velocity = left_motor_angular_velocity * self.wheel_radius
        right_wheel_linear_velocity = right_motor_angular_velocity * self.wheel_radius
        
        print("left wheel velocity: " + str(left_wheel_linear_velocity))
        print("right wheel velocity: " + str(right_wheel_linear_velocity))
        
        # Update odometry
        self.update_odometry(left_wheel_linear_velocity, right_wheel_linear_velocity, delta_t)

        # Save timestamp for the next iteration
        self.last_timestamp = current_timestamp

    def update_odometry(self, left_wheel_velocity, right_wheel_velocity, delta_t):
        # Update odometry here using the left_wheel_velocity, right_wheel_velocity, and delta_t

        # Create Odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.seq = self.odom_message_count
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'

        (delta_x, delta_y, delta_theta, husky_linear_velocity, angular_velocity) = self.calculate_kinematics(left_wheel_velocity, right_wheel_velocity, delta_t)
        
        # Populate odometry message fields
        odometry_msg.twist.twist.linear.x = husky_linear_velocity
        odometry_msg.twist.twist.angular.z = angular_velocity

        updated_x = self.px + delta_x
        updated_y = self.py + delta_y
        updated_theta = self.pth + delta_theta

        new_pose_orientation = self.calculate_orientation(updated_theta) # quaternion

        odometry_msg.pose.pose.position = Point(updated_x, updated_y, 0)
        odometry_msg.pose.pose.orientation = new_pose_orientation
        
        # update fields
        self.px = updated_x
        self.py = updated_y
        self.pth = updated_theta

        # Publish odometry message
        self.odometry_pub.publish(odometry_msg)
        self.odom_message_count += 1

    def calculate_orientation(self, updated_theta):
        # verify that you only need to update the z quaternion to determine the orientation.
        quaternion = Quaternion()
        quaternion.z = updated_theta

        return quaternion

    def calculate_kinematics(self, left_wheel_velocity, right_wheel_velocity, delta_t):
        # Calculate linear and angular velocities of the Husky
        husky_linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0
        husky_angular_velocity = (right_wheel_velocity - left_wheel_velocity) / self.wheel_seperation
        
        # define return variables
        delta_x = 0
        delta_y = 0
        delta_theta = 0

	# check if velocities are equal (within some tolerance)
	if self.have_same_sign(left_wheel_velocity, right_wheel_velocity) and self.in_tolerance(left_wheel_velocity, right_wheel_velocity):
    	    delta_x = husky_linear_velocity * (cos(self.pth) * delta_t)
    	    delta_y = husky_linear_velocity * (sin(self.pth) * delta_t)

	elif not self.have_same_sign(left_wheel_velocity, right_wheel_velocity) and self.in_tolerance(left_wheel_velocity, right_wheel_velocity):
	    
        # check if velocities are the opposite of each other (within some tolerance)
            delta_theta = (2 * husky_linear_velocity * delta_t) / self.wheel_seperation
	else:
	    R = husky_linear_velocity / husky_angular_velocity
	    current_theta = self.pth

	    delta_x = (-R * sin(current_theta)) + (R * sin(current_theta + (husky_angular_velocity * delta_t)))
	    delta_y = (R * cos(current_theta)) - (R * cos(current_theta + (husky_angular_velocity * delta_t)))
	    delta_theta = husky_angular_velocity * delta_t

	return delta_x, delta_y, delta_theta, husky_linear_velocity, husky_angular_velocity

    def have_same_sign(self, left_wheel, right_wheel):
	    # if two numbers have the same sign, their multiplication will result in a positive number
	    # include zero in case both wheel velocities are zero and are accelerating
            print("have same sign?: " + str(left_wheel * right_wheel >= 0))
            return (left_wheel * right_wheel) >= 0

    def in_tolerance(self, left_wheel, right_wheel):
	    # determines if two wheels are within a given tolerance of each other
            print("in tolerance?: " + str(abs(left_wheel - right_wheel) <= HuskyOdometry.VELOCITY_THRESHOLD))
	    return abs(left_wheel - right_wheel) <= HuskyOdometry.VELOCITY_THRESHOLD

    def run(self):
	print("running")    
        rospy.spin()

if __name__ == '__main__':
    odom = HuskyOdometry()
    odom.run()
