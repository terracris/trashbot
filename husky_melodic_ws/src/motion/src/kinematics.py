#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion 

from math import sqrt, atan2, pi

class Kinematics:

    def __init__(self):
        """
        Class constructor
        """

        # Initialize odometry variables to zero.
        self.px = 0
        self.py = 0
        self.pth = 0

        # Initialize node --> Name is "kinematics"
        rospy.init_node("kinematics")

        # This node publishes Twist messages on the '/husky_velocity_controller/cmd_vel'
        self.cmd_vel = rospy.Publisher('/husky_velocity_controller/cmd_vel',Twist, queue_size=10)

        # this node subscribes to Odometry messages on '/husky_velocity_controller/odom'
        # when a message is received, call self.update_odometry
        rospy.Subscriber('/husky_velocity_controller/odom', Odometry, self.update_odometry)

        # this method subscribes to PoseStamped messages on the '/move_base_simple/goal'
        # when message is received, call self.go_to
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)

    def send_speed(self, linear_speed: float, angular_speed: float):
        """
        Sends the speeds to the motors.
        :param linear_speed [float] [m/s]    The forward linear speed
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """

        # Make a new Twist message
        msg_cmd_vel = Twist()

        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0

        # Angular speed
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed

        # publish the message
        self.cmd_vel.publish(msg_cmd_vel)

    def drive(self, distance:float, linear_speed:float):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        Utilizes a trapezoidal acceleration; Accelerates --> Holds a constant speed --> Decelerates
        :param distance [float] [m]        The distance to cover.
        :param linear_speed [float] [m/s]  The maximum forward linear speed
        """

        # distance tolerable from goal [m]
        TOLERANCE = 0.05 # 5cm tolerance

        SLEEP_DURATION = 0.01
        ACCELERATION = 0.02

        # calculate the stopping distance
        stopping_distance = (linear_speed/2.0) * (linear_speed/ACCELERATION) + TOLERANCE
        if(stopping_distance > distance/2.0):
            stopping_distance = distance/2

        # initialize our starting position
        start_x = self.px
        start_y = self.py

        current_speed = 0.0
        decelerating = False

        # Drives
        while True:
            dist_traveled = sqrt(pow((self.px - start_x), 2) + pow((self.py - start_y), 2))

            # Accelerate
            if not decelerating:
                current_speed += ACCELERATION * SLEEP_DURATION
                if(current_speed > linear_speed):
                    current_speed = linear_speed

            # Decelerate
            if dist_traveled > (distance - stopping_distance):
                decelerating = True
                current_speed -= ACCELERATION * SLEEP_DURATION
                if current_speed < 0:
                    current_speed = 0

            # set speeds
            self.send_speed(current_speed, 0)

            # check if complete
            if abs(dist_traveled - distance) < TOLERANCE:
                self.send_speed(0,0)
                break

            rospy.sleep(SLEEP_DURATION)

    def rotate(self, angle:float, aspeed:float):
        """
        Rotates the robot around the body center by the given angle.
        :param angle  [float] [rad]   The angle to cover.
        :param aspeed [float] [rad/s] The angular speed.
        """
        
        # SLEEP DURATION
        SLEEP_DURATION = 0.005

        # Half a degree tolerance.
        TOLERANCE = 0.01 

        # keeps angle in range [-180, 180)
        angle = ((angle + pi) % (2*pi)) - pi

        # The angle difference to be turned
        angle_diff = (angle - self.pth)

        # Ensure's they're the same sign
        if((aspeed < 0) != (angle_diff < 0)):
            aspeed *= -1

        # If we're going the long way around, just go the other way instead
        if(abs(angle_diff) > pi):
            aspeed *= -1

        # Rotates
        while True:
            self.send_speed(0, aspeed)

            # Checks if done
            if(self.pth > angle - TOLERANCE and self.pth < angle + TOLERANCE):
                self.send_speed(0, 0) # Stops
                break

        rospy.sleep(SLEEP_DURATION)

    def go_to(self, msg: PoseStamped):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a subscriber.
        :param msg [PoseStamped] The target pose. 
        """
        
        # Control Constants
        ROTATE_SPEED = 0.3
        DRIVE_SPEED = 0.3

        angle, distance = self.angle_dist_to_point(msg.pose.position.x, msg.pose.position.y)

        # moves
        self.rotate(angle, ROTATE_SPEED)
        rospy.sleep(1)
        self.drive(distance, DRIVE_SPEED)
        rospy.sleep(1)
        (_, _, yaw) = euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])
        self.rotate(yaw, ROTATE_SPEED)


    def update_odometry(self, msg:Odometry):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """

        # Updates position
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

        # updates orientation
        quat_orig = msg.pose.pose.orientation

        (_, _, yaw) = euler_from_quaternion([quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w])

        self.pth = yaw

    def angle_dist_to_point(self, x:float, y:float):
        """
        Calculates the angle the robot has to face (heading) as well as distance to that point
        """

        # calculates differences
        x_diff = x - self.px
        y_diff = y - self.py

        # Calculates angle and distance
        angle = atan2(y_diff, x_diff)
        distance = sqrt(pow(x_diff, 2) + pow(y_diff, 2))

        return angle, distance


    def run(self):
        rospy.spin()

if __name__ == 'main':
    motion = Kinematics()
    motion.run()
