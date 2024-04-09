#!/usr/bin/env python2.7

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point
from math import sin, cos, atan2, radians
from std_msgs.msg import Float64
import tf
# We need to increase the accuracy of my values to improve the accuracy of my tolerances and odometry

class HuskyOdometry:
    # velocity threshold for determining equivalence. 
    # currently set to 1 mm per second
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
        HUSKY_A100_WHEEL_RADIUS = 0.1143 # radians
        HUSKY_A200_WHEEL_RADIUS = 0.1651
        HUSKY_A100_WHEEL_TRACK = 0.505 # 50 cm wheel track
        HUSKY_A200_WHEEL_TRACK = 0.555 # 55.5 cm wheel track
        SIM = True

        # Husky A100 parameters [m]
        self.wheel_seperation = HUSKY_A200_WHEEL_TRACK if SIM else HUSKY_A100_WHEEL_TRACK # number retrieved from manual on the A100
        self.wheel_radius = HUSKY_A200_WHEEL_RADIUS if SIM else HUSKY_A100_WHEEL_RADIUS  # wheels are 9 inches in Diameter for our husky

        # subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.Subscriber('/imu/heading', Float64, self.update_heading)

        # publishers
        self.odometry_pub = rospy.Publisher('a100/odometry', Odometry, queue_size=10)


    def update_heading(self, heading):
        # Ensure angle is in the range [0, 360)
        angle = heading.data % 360
        # Map angles from 0 to 180 to the range from 0 to -pi
        if angle == 0:
            self.pth = 0
        elif angle <= 180:
            self.pth = -radians(angle)
        # Map angles from 180 to 360 to the range from 0 to pi
        else:
            self.pth = radians(360 - angle)
   
    def joint_states_callback(self, joint_states):
        current_timestamp = joint_states.header.stamp

        print("joint state timestamp: ", current_timestamp)

        # Calculate time interval
        delta_t = (current_timestamp - self.last_timestamp).to_sec()

        print("delta t: ", delta_t)

        # Extract angular positions and velocities
        left_motor_angular_velocity = joint_states.velocity[0]
        right_motor_angular_velocity = joint_states.velocity[1]

        # Convert motor angular velocity to linear velocity for each wheel
        left_wheel_linear_velocity = left_motor_angular_velocity * self.wheel_radius # resulting calculation should be of decimal object
        right_wheel_linear_velocity = right_motor_angular_velocity * self.wheel_radius # resulting calculation should be of decimal object
        
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

        (delta_x, delta_y, husky_linear_velocity, angular_velocity) = self.calculate_kinematics(left_wheel_velocity, right_wheel_velocity, delta_t)
        
        # Populate odometry message fields
        odometry_msg.twist.twist.linear.x = husky_linear_velocity
        odometry_msg.twist.twist.angular.z = angular_velocity

        updated_x = self.px + delta_x
        updated_y = self.py + delta_y

        print("Self.pth: ", self.pth)

        new_pose_orientation = self.calculate_orientation(self.pth) # quaternion

        odometry_msg.pose.pose.position = Point(updated_x, updated_y, 0)
        odometry_msg.pose.pose.orientation = new_pose_orientation
        
        # update fields
        self.px = updated_x
        self.py = updated_y

        # Publish odometry message
        self.odometry_pub.publish(odometry_msg)
        self.odom_message_count += 1

    def calculate_orientation(self, updated_theta):
        # verify that you only need to update the z quaternion to determine the orientation.
        q = tf.transformations.quaternion_from_euler(0, 0, updated_theta)
        
        quaternion = Quaternion()
        quaternion.x = q[0]
        quaternion.y = q[1]
        quaternion.z = q[2]
        quaternion.w = q[3]

        return quaternion

    def calculate_kinematics(self, left_wheel_velocity, right_wheel_velocity, delta_t):
        # Calculate linear and angular velocities of the Husky
        # TODO verify if these variables are also Decimal types and what their precision is
        
        
        husky_linear_velocity = (left_wheel_velocity + right_wheel_velocity) / 2.0
        husky_angular_velocity = (right_wheel_velocity - left_wheel_velocity) / self.wheel_seperation
        print("husky linear velocity: " + str(husky_linear_velocity))
        print("husky angular velocity: " + str(husky_angular_velocity))

        # define return variables
        delta_x = 0
        delta_y = 0

        # check if velocities are equal (within some tolerance)
        if self.have_same_sign(left_wheel_velocity, right_wheel_velocity) and self.in_tolerance(left_wheel_velocity, right_wheel_velocity):
            delta_x = husky_linear_velocity * (cos(self.pth) * delta_t)
            delta_y = husky_linear_velocity * (sin(self.pth) * delta_t)


        # TODO implement curved motion where wheels are not going at the same speed

        return delta_x, delta_y, husky_linear_velocity, husky_angular_velocity

    def have_same_sign(self, left_wheel, right_wheel):
        # if two numbers have the same sign, their multiplication will result in a positive number
        # include zero in case both wheel velocities are zero and are accelerating
        print("have same sign?: " + str(left_wheel * right_wheel >= 0))
        return (left_wheel * right_wheel) >= 0

    def in_tolerance(self, left_wheel, right_wheel):
	    # determines if two wheels are within a given tolerance of each other
            print("in tolerance?: " + str(abs(abs(left_wheel) - abs(right_wheel)) <= HuskyOdometry.VELOCITY_THRESHOLD))
            return abs(abs(left_wheel) - abs(right_wheel)) <= HuskyOdometry.VELOCITY_THRESHOLD

    def run(self):
        print("running")
        rospy.spin()

if __name__ == '__main__':
    odom = HuskyOdometry()
    odom.run()
