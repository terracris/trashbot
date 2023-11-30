#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose

class Odometry:
    def __init__(self):
        rospy.init_node('a100_odometry', anonymous=True)

        # initialize variables
        self.last_husky_pose = {'x': 0.0, 'y':0.0, 'orientation':0.0}
        self.last_timestamp = rospy.Time.now()

        # Husky A100 parameters [m]
        self.wheel_seperation = 0.50 # number retrieved from manual on the A100
        self.wheel_radius = 0.2286

        # subscribers
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)

        # publishers
        self.odometry_pub = rospy.Publisher('/odometry', Odometry, queue_size=10)

    def joint_states_callback(self, joint_states):
        current_timestamp = joint_states.header.stamp

        # Calculate time interval
        delta_t = (current_time - self.last_timestamp).to_sec()

        # Extract angular positions and velocities
        left_motor_angular_velocity = joint_states.velocity[0]
        right_motor_angular_velocity = joint_states.velocity[1]

        # Convert motor angular velocity to linear velocity for each wheel
        left_wheel_linear_velocity = left_motor_angular_velocity * self.wheel_radius
        right_wheel_linear_velocity = right_motor_angular_velocity * self.wheel_radius

        # Calculate linear and angular velocities of the Husky
        husky_linear_velocity = (left_wheel_linear_velocity + right_wheel_linear_velocity) / 2.0
        husky_angular_velocity = (right_wheel_linear_velocity - left_wheel_linear_velocity) / self.wheel_seperation

        # Update odometry
        self.update_odometry(linear_velocity, angular_velocity, delta_t)

        # Save timestamp for the next iteration
        self.last_timestamp = current_timestamp

    def update_odometry(self, linear_velocity, delta_t):
        # Update odometry here using the linear_velocity, angular_velocity, and delta_t

        # Create Odometry message
        odometry_msg = Odometry()
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.frame_id = 'odom'
        odometry_msg.child_frame_id = 'base_link'

        # Populate odometry message fields
        odometry_msg.twist.twist.linear.x = linear_velocity
        odometry_msg.twist.twist.angular.z = angular_velocity

        # TODO verify odometry
        #TODO do you need a conditional for determining if the wheel speeds are the same?
        # this current implementation assumes that the wheel speeds are always the same. 
        # what if we are travelling a curved path?
        # current implementation is incomplete.
        # Integrate velocities to update position and orientation
        new_pose_x = self.last_husky_pose['x'] + linear_velocity * delta_t
        new_pose_orientation = self.calculate_orientation(angular_velocity, delta_t)

        # Publish odometry message
        self.odometry_pub.publish(odometry_msg)

        def calculate_orientation(self, angular_velocity, delta_t):
            # verify that you only need to update the z quaternion to determine the orientation.
            quaternion = Quaternion
            quaternion.z = self.last_husky_pose['orientation'] + angular_velocity * delta_t

            return quaternion
