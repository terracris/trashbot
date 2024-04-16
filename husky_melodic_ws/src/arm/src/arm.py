#!/usr/bin/env python3

import threading
import numpy as np
from math import radians, degrees, sqrt, atan2, pi, asin, cos
# import modern_robotics as mr
import modern.core as mr
from stepper import Stepper
from time import sleep
import rospy
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from ik import ik_geo

class Arm:
    # I am going to make the arm take in 4 different motors on startup
    def __init__(self, j1, j2, j3):
        
        rospy.init_node("arm", anonymous=True)
        
        # A service that accepts messages of type GetPlan and
        # calls 'pickup' method when a message is received
        self.collection_service = rospy.Service('manipulate', GetPlan, self.pickup)

        self.joints = [j1, j2, j3]
        self.joint_angles = [0, 0, 0]

        # all lengths are [ m ]
        # all angles are [ rad ] 
        # home configuration of robot end effector (concepts covered in RBE 501)
        self.M = np.array([[ 1, 0, 0, 0.55912576],
                           [ 0, 1, 0, 0.0196],
                           [ 0, 0, 1, 0.935512],
                           [ 0, 0, 0, 1]])
        
        self.camera_transformation = np.array([[ 0,-0.4067, 0.9135, 0.122497],
                                               [-1, 0,      0,      0.032445],
                                               [ 0,-0.9135,-0.4067, 0.416466],
                                               [ 0, 0,      0,      1]])

        # screw axis (twist list)
        self.twist_list = np.array([[0, 0, 1,     0,        0,       0],
                                    [0, 1, 0,   -0.53477,      0,     0.07],
                                    [0, 1, 0, -0.9054,     0,     0.07],
				    [1, 0, 0, 0,     0.935512,    -0.0196]]).T

        
        self.theta_list_guess = np.array([0, 1.57, 0, 0])
        
        # EE orientation error tol
        self.eomg = 0.01 
        # EE position error tol --> Tolerance is 1mm
        self.ev = 0.01

        # self.home()
        self.is_active = False
        self.j2_increase = False

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
        ik, success = mr.IKinSpace(self.twist_list, self.M, desired_ee, self.theta_list_guess, self.eomg, self.ev)
        return ik, success

    # target_xyz has shape (3x1)
    def ik_ana(self, target_xyz):
        # use mr.fkinSpace to find the current xyz and q for home configuration of ee
        theta_list = np.array([0, 0, 0, 0]) # hard coded TODO 
        fk = mr.FKinSpace(self.M, self.twist_list,theta_list)
        curr_x, curr_y, curr_z = fk[0:3, 3] 

	    # transpose the matrix, so has shape (3x1)
        current_xyz = np.array([curr_x, curr_y, curr_z]).T
        # 4x1 matrix
        current_q = np.array([0, 0, 0, 0]) # hard coded TODO
        max_iterations = 50
        i = 0

        identity_4 = np.eye(4)
	
        while (np.linalg.norm(target_xyz - current_xyz) > 0.001) and (i < max_iterations):
            Ja = self.J_a(current_q)
            # need seudo inverse here TODO
            print(Ja)
            delta_xyz = target_xyz - current_xyz # (3,1)
            pseudo_inv = np.linalg.pinv(Ja)
            matrix_1 = identity_4 - np.dot(pseudo_inv, Ja)
            theta_change = theta_list - current_q
            delta_q = np.dot(pseudo_inv, delta_xyz) + np.dot(matrix_1, theta_change)  # (3x1)
            current_q = current_q + delta_q.T
            T = mr.FKinSpace(self.M, self.twist_list,current_q)
            current_xyz = np.array(T[0:3,3])  # [first:last+1, element number
            i += 1 # increase iteration pass
            print(i)

        success = True
        
        if i == max_iterations:
            success = False

        return current_q, success
	

    def J_a(self, thetalist):
        Slist = self.twist_list
        M = self.M
        Js = np.array(Slist).copy().astype(float)     
        T = np.eye(4)
        for i in range(1, len(thetalist)):
            T = np.dot(T, mr.MatrixExp6(mr.VecTose3(np.array(Slist)[:, i - 1]*thetalist[i - 1])))
            Js[:, i] = np.dot(mr.Adjoint(T), np.array(Slist)[:, i])

        T = mr.FKinSpace(M, Slist, thetalist)
        J_w = Js[:3, :]
        J_v = Js[3:, :]
        w = T[:3, 3]
        p = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
        resultant = np.dot(p, J_w)
        J_a = J_v - resultant
        return J_a

   
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
        angle_deg = degrees(joint_angle)
        #print("angle for joint: ", joint.id, " ", angle_deg)
        
        updated_angle = joint.write(angle_deg)
        
        return updated_angle # writes angle to joint --> needs to be threading though

    def update_angles(self, joint_angles):
        
        # updates our joint angles to the ones actually achieved by the motors
        for x in range(len(joint_angles)):
            self.joint_angles[x] = joint_angles[x]

    
    def get_current_theta(self):

        # returns our current joint angles in radians
        theta_list = []

        for theta in self.joint_angles:
            print(theta)
            theta_list.append(radians(theta))
        
        return theta_list
    
    # receives GetPlan message
    def pickup(self, msg):
        
        if self.is_active:
            empty_path = Path()
            empty_path.poses = []
            return empty_path

        self.is_active = True
        print("yoooooo, we got a request")
        goal = msg.goal.pose.position
        
        x, y, z = goal.x, goal.y, goal.z

        print("x: ", x, "y: ", y, "z: ", z)

        # create (4x1) numpy array
        camera_point = np.array([x, y, z, 1]).T
        desired_ee_from_arm = np.dot(self.camera_transformation, camera_point) 
        trans_x, trans_y, trans_z = desired_ee_from_arm[0], desired_ee_from_arm[1], desired_ee_from_arm[2]
        
        print("trans_x: ", trans_x, "trans_y: ", trans_y, "trans_z: ", trans_z)
        x_offset = 0.07 # 7cm
        offset_x = trans_x - x_offset

        l1, l2, l3 = 0.53477, 0.37063, 0.559
        alpha = asin(x_offset/l1)
        s = trans_z - (l1*cos(alpha))
        r = sqrt((offset_x**2) + (trans_y**2))
        c3 = (r**2 + s**2 - l2**2 - l3**2) / (2*(l1*cos(alpha))*l2)
        s3 = -sqrt(1-(c3**2))
        gamma = atan2(s, r)
        phi = atan2((l3*s3), (l2+ (l3*c3)))

        #j1 = atan2(trans_y, trans_x)
        #j2 = (gamma - phi) + (pi/2)
        #j3 = atan2(s3, c3) + (pi/2)

        j1, j2, j3 = ik_geo(trans_x, trans_y, trans_z)
        if j1 < 0:
            j1 = j1*1.3
        j2 = j2*1
        j3 = j3*1.10
        joint_angles = [j1, j2, j3]
        
        print()
        [print("joint ", (joint+1),": ", degrees(angle), "degrees") for joint, angle in enumerate(joint_angles) ]

        traj = self.trajectory_planning(joint_angles)

        print("trajectory angles: ", traj)
        
        self.home()
        self.follow_trajectory(traj)

        poses = []

        #for joint_angles in traj:
        ps = PoseStamped()
        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = joint_angles

        poses.append(ps)


        # this should return something? None on failure?
        path = Path()
        path.poses = poses

        #self.is_active = False
        return path
    
    def run(self):
        rospy.spin()
    
    def cleanup(self):
        for joint in self.joints:
            joint.cleanup()
            

if __name__ == '__main__':
    
    pulses_per_rev = 200
    enable_pin = 37
    
    # joint 1
    pulse_pin_j1 = 11
    dir_pin_j1 = 13
    homing_pin_j1 = 7
    gear_ratio_j1 = 4
    home_count_j1 = -140
    max_speed_j1 = 75
    max_positive_angle_j1 = 60
    max_negative_angle_j1 = -90
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
    max_positive_angle_j2 = 115
    max_negative_angle_j2 = -10
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
    max_positive_angle_j3 = 75  # TODO calculate joint limit
    max_negative_angle_j3 = -75  # TODO calculate joint limit
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
    max_positive_angle_j4 = 90 # TODO calculate joint limits
    max_negative_angle_j4 = -40 # TODO calcylate joint limit
    homing_direction_j4 = Stepper.CW
	   	
 
    try:
        print("setting up the arm")
        j1 = Stepper(pulse_pin_j1, dir_pin_j1, enable_pin, homing_pin_j1, pulses_per_rev, gear_ratio_j1, max_speed_j1,max_positive_angle_j1,max_negative_angle_j1, home_count_j1,homing_direction_j1, stepper_id =1, debug=False) 
        j2 = Stepper(pulse_pin_j2, dir_pin_j2, enable_pin, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2,max_positive_angle_j2, max_negative_angle_j2,home_count_j2,homing_direction_j2 ,inverted=True, stepper_id=2, debug=True)
        j3 = Stepper(pulse_pin_j3, dir_pin_j3, enable_pin, homing_pin_j3, pulses_per_rev, gear_ratio_j3, max_speed_j3,max_positive_angle_j3, max_negative_angle_j3,home_count_j3,homing_direction_j3,kp=0.10,kd=0.003, stepper_id = 3)
        j4 = Stepper(pulse_pin_j4, dir_pin_j4, enable_pin, homing_pin_j4, pulses_per_rev, gear_ratio_j4, max_speed_j4,max_positive_angle_j4, max_negative_angle_j4, home_count_j4,homing_direction_j4,kp=1,kd=0.003, stepper_id= 4)
       
        arm = Arm(j1, j2, j3)
        # arm.home()

        
        # get joint angles
        # joint_angles = arm.ik(desired_ee)

        # get trajectory to follow
        # traj = arm.trajectory_planning(joint_angles)

        # follow the trajectory
        # arm.follow_trajectory(traj)

        arm.run()
    
    except KeyboardInterrupt:
        j1.cleanup()
