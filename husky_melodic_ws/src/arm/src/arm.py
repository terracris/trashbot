#!/usr/bin/env python3

import threading
import numpy as np
from math import radians, degrees
import modern_robotics as mr
from stepper import Stepper
from time import sleep
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Arm:
    # I am going to make the arm take in 4 different motors on startup
    def __init__(self, j1, j2, j3, j4):
        
        rospy.init_node("arm", anonymous=True)
        
        # A service that accepts messages of type GetPlan and
        # calls 'pickup' method when a message is received
        self.collection_service = rospy.Service('manipulate', GetPlan, self.pickup)

        self.joints = [j1, j2, j3, j4]
        self.joint_angles = [0, 0, 0, 0]

        # all lengths are [ m ]
        # all angles are [ rad ] 
        # home configuration of robot end effector (concepts covered in RBE 501)
        self.M = np.array([[ 1, 0, 0, 0.55912576],
                           [ 0, 1, 0, 0.0181],
                           [ 0, 0, 1, 0.598742],
                           [ 0, 0, 0, 1]])
        
        # screw axis (twist list)
        self.twist_list = np.array([[0, 0, 1,     0,        0,       0],
                                    [0, 1, 0,   0.198,      0,     -0.07],
                                    [0, 1, 0, 0.568642,     0,     -0.07],
                                    [1, 0, 0,     0,    0.598742, -0.0181]]).T

        
        self.theta_list_guess = np.array([np.pi / 2.0, np.pi / 4.0, np.pi / 4.0, np.pi / 2.0])
        
        # EE orientation error tol
        self.eomg = 0.01 
        # EE position error tol --> Tolerance is 1mm
        self.ev = 0.001

        # self.home()


    def home(self):
        # joint is the actual stepper motor class

        threads = []

        for joint in self.joints:
            thread = threading.Thread(target=self.home_joint, args=(joint,))
            thread.start()
            threads.append(thread)

        # wait for each joint to home
        for thread in threads:
            thread.join()
            threads.remove(thread) # remove thread after completion
        
        print("All threads finished")

    # blocking function to home each joint
    def home_joint(self, joint):
        print("homing joint")
        joint.home()

    # theta_list is list of joint angles 
    def fk(self, theta_list):
        fk = mr.FKinSpace(self.M, self.twist_list,theta_list)
        return fk
    
    def ik(self, desired_ee):
        ik, _ = mr.IKinSpace(self.twist_list, self.M, desired_ee, self.theta_list_guess, self.eomg, self.ev)
        return ik
    
    def trajectory_planning(self, ik):
        
        tf = 5     # time of motion [ s ]
        N = 5      # number of points in trajectory
        method = 5 # time-scaling method (quintic)
        theta_start = self.get_current_theta()
        joint_traj = mr.JointTrajectory(theta_start, ik, tf, N, method)
        return joint_traj
    
    def follow_trajectory(self, trajecory):
        start_time = Stepper.get_time()  # time resolution will be in seconds
        # trajectory is a numpy array size (N x J)
        # N: Number of points in the trajectory
        # J: Number of joints in the robot arm
        num_trajecory_points = trajecory.shape[0]

        # ALWAYS SKIP THE FIRST TRAJECTORY --> you are already there.

        for point in range(1, num_trajecory_points):
            joint_pose = trajecory[point]
            self.write_joints(joint_pose)

    def write_joints(self, joint_pose):
        zipped = zip(self.joints, joint_pose)
        threads = []
        updated_angle = []
        
        for joint, joint_angle in zipped:
            thread = threading.Thread(target=self.write_joint, args=(joint, joint_angle))
            thread.start()
            threads.append(thread)

        # wait for each joint to reach its position
        for thread in threads:
            updated_joint_angle = thread.join()
            updated_angle.append(updated_joint_angle)
            threads.remove(thread) # remove thread after completion
        
        self.update_angles(updated_angle)

    def write_joint(self, joint, joint_angle):
        # TODO make write in stepper library return the actual angle of the joint
        # because of our step angle resolution there is error --> this will help account for the error in pose
        return joint.write(degrees(joint_angle)) # writes angle to joint --> needs to be threading though

    def update_angles(self, joint_angles):
        
        # updates our joint angles to the ones actually achieved by the motors
        for x in range(len(joint_angles)):
            self.joint_angles[x] = joint_angles[x]

    
    def get_current_theta(self):

        # returns our current joint angles in radians
        theta_list = []

        for theta in self.joint_angles:
            theta_list.append(radians(theta))
        
        return theta_list
    
    # receives GetPlan message
    def pickup(self, msg):
        
        print("yoooooo, we got a request")
        goal = msg.goal.pose.position
        x, y, z = goal.x, goal.y, goal.z

        desired_ee = np.array([[ 0,  0, 0,  x],
                               [ 0,  0, 0,  y],
                               [ 0,  0,  0, z],
                               [ 0,  0,  0, 1]])

        joint_angles = arm.ik(desired_ee)
        traj = arm.trajectory_planning(joint_angles)
        
        #arm.follow_trajectory(traj)

        poses = []

        for joint_angles in traj:
            ps = PoseStamped()
            ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = joint_angles

            poses.append(ps)


        # this should return something? None on failure?
        path = Path()
        path.poses = poses

        return path
    
    def run(self):
        rospy.spin()
    
    def cleanup(self):
        for joint in self.joints:
            joint.cleanup()
            

if __name__ == '__main__':
    
    pulses_per_rev = 200
    enable_pin = 22
    
    # joint 1
    pulse_pin_j1 = 11
    dir_pin_j1 = 13
    homing_pin_j1 = 7
    gear_ratio_j1 = 4
    home_count_j1 = -140
    max_speed_j1 = 75
    max_ccw_j1 = 90
    max_cw_j1 = -90
    homing_direction_j1 = Stepper.CCW

    # joint 2
    pulse_pin_j2 = 19
    dir_pin_j2 = 21
    homing_pin_j2 = 23
    gear_ratio_j2 = 5 * 5.18
    home_count_j2 = -145
    max_speed_j2 = 75
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j2 = 135
    max_cw_j2 = -10
    homing_direction_j2 = Stepper.CCW

    # joint 3
    pulse_pin_j3 = 29
    dir_pin_j3 = 31
    homing_pin_j3 = 33
    gear_ratio_j3 = 4 * 5.18  # TODO review gear ratio
    home_count_j3 = -740  # TODO calculate home count
    max_speed_j3 = 75
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j3 = 90  # TODO calculate joint limit
    max_cw_j3 = -90  # TODO calculate joint limit
    homing_direction_j3 = Stepper.CW
 
    # joint 4
    pulse_pin_j4 = 32
    dir_pin_j4 = 38
    homing_pin_j4 = 40
    gear_ratio_j4 = 1 # TODO calculate gear ratio
    home_count_j4 = -30 # TODO calculate home count
    max_speed_j4 = 50
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j4 = 90 # TODO calculate joint limits
    max_cw_j4 = -40 # TODO calcylate joint limit
    homing_direction_j4 = Stepper.CW
 
    try:
        print("setting up the arm")
        j1 = Stepper(pulse_pin_j1, dir_pin_j1, enable_pin, homing_pin_j1, pulses_per_rev, gear_ratio_j1, max_speed_j1, max_ccw_j1, max_cw_j1, home_count_j1,homing_direction_j1, debug=True) 
        j2 = Stepper(pulse_pin_j2, dir_pin_j2, enable_pin, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2, max_ccw_j2, max_cw_j2, home_count_j2,homing_direction_j2 ,inverted=True, debug=False)
        j3 = Stepper(pulse_pin_j3, dir_pin_j3, enable_pin, homing_pin_j3, pulses_per_rev, gear_ratio_j3, max_speed_j3, max_ccw_j3, max_cw_j3, home_count_j3,homing_direction_j3,kp=0.10,kd=0.003)
        j4 = Stepper(pulse_pin_j4, dir_pin_j4, enable_pin, homing_pin_j4, pulses_per_rev, gear_ratio_j4, max_speed_j4, max_ccw_j4, max_cw_j4, home_count_j4,homing_direction_j4,kp=1,kd=0.003)
       
        arm = Arm(j1, j2, j3, j4)
        arm.run()
    
    except KeyboardInterrupt:
        j1.cleanup()
