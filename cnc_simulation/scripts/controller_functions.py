#!/usr/bin/env python3
import rospy
import math
import matplotlib.pyplot as plt
import roboticstoolbox as rtb
from spatialmath import SE3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

"""
 @Author: Mehmet Kahraman
 @Date: 15.12.2022
"""

def get_rad_from_degree(angle):
    if angle == 0:
        return 0
    else:
        temp = 180 / angle
        rad = math.pi / temp
        return rad

def get_degree_from_rad(angle):
    if angle == 0:
        return 0
    else:
        degree = math.degrees(angle)
        return degree

def get_degrees(radians):
    degrees = []
    for i in radians:
        deg = get_degree_from_rad(i)
        degrees.append(deg)
    return degrees

def get_radians(degrees):
    radians = []
    for i in degrees:
        rad = get_rad_from_degree(i)
        radians.append(rad)
    return radians

def define_my_robot():
    #  !! without gripper !!
    my_arm = rtb.ERobot.URDF("/home/mehmet/cnc_project_ws/src/ur5e_description/urdf/UR5e.urdf")
    return my_arm

def forward_kinematics(my_arm, joint_list, input="radian"):
    if input == "degree":
        rad_joints = [math.radians(i) for i in joint_list]
    else:
        rad_joints = joint_list
    T = my_arm.fkine(rad_joints)
    print(T)
    x = round(T.t[0], 3)
    y = round(T.t[1], 3)
    z = round(T.t[2], 3)
    return [T, x, y, z]

def inverse_kinematics(robot, in_data, input_type, output="radian"):
    if input_type == "transformation_matrix":
        ik_solution = robot.ikine_LM(in_data)
    if input_type == "xyz_position":
        position = SE3(in_data[0], in_data[1], in_data[2])
        ik_solution = robot.ikine_LM(position)
    print(ik_solution)
    joints_array_rad = ik_solution.q
    if output == "degree":
        joints_degree = [math.degrees(i) for i in joints_array_rad]
        return joints_degree
    else:
        return joints_array_rad

def generate_joint_trajectory(a1_list, a2_list, step=80):
    trajectory_solution = rtb.tools.trajectory.jtraj(a1_list, a2_list, step)
    q_array = trajectory_solution.q
    qd_array = trajectory_solution.qd
    qdd_array = trajectory_solution.qdd
        
    return (q_array, qd_array, qdd_array)

def plot_joint_trajectories(a1_list, a2_list, step=80):
    trajectory_solution = rtb.tools.trajectory.jtraj(a1_list, a2_list, step)
    q_array = trajectory_solution.q
    qd_array = trajectory_solution.qd
    qdd_array = trajectory_solution.qdd
    
    plt.figure(figsize=(12,6))
    x_axes = range(step)
    
    for i in range(6):
        
        # positions
        q_data_joint = [q[i] for q in q_array]
        fig, axes = plt.subplots(nrows=2, ncols=3)
        if i<3:
            axes[0,i].plot(x_axes, q_data_joint)
            axes[0,i].set_title("Joint {}".format(str(i)))
        else:
            axes[1,i].plot(x_axes, q_data_joint)
            axes[1,i].set_title("Joint {}".format(str(i)))
        
    plt.show()

def perform_trajectory(trajectory_publisher, target_angles, input="radian", duration=5):
    
    if input == "degree":
        target_angles = get_radians(target_angles)

    rospy.loginfo("trajectory calculating...")
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = target_angles
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.accelerations = [0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration(duration)
    trajectory_msg.points.append(point)
    
    rospy.loginfo("trajectory performing...")
    rospy.sleep(0.4)
    trajectory_publisher.publish(trajectory_msg)
    rospy.sleep(duration+1)
    rospy.loginfo("trajectory execution successfully done!")
    
def perform_generated_trajectory(trajectory_publisher, target_angles, input="radian", duration=5):
    
    if input == "degree":
        target_angles = get_radians(target_angles)

    rospy.loginfo("trajectory calculating...")
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = joint_names
    point = JointTrajectoryPoint()
    point.positions = target_angles
    point.velocities = [0, 0, 0, 0, 0, 0]
    point.accelerations = [0, 0, 0, 0, 0, 0]
    point.time_from_start = rospy.Duration(duration)
    trajectory_msg.points.append(point)
    
    rospy.loginfo("trajectory performing...")
    rospy.sleep(0.4)
    trajectory_publisher.publish(trajectory_msg)
    rospy.sleep(duration+1)
    rospy.loginfo("trajectory execution successfully done!")

def open_gripper_with_param():
    
    rospy.loginfo("gripper opening...")
    rospy.sleep(1)
    rospy.set_param("/rg2_gripper_command", 1.0)
    rospy.sleep(1)
    rospy.loginfo("gripper opened!")

def open_gripper():
    
    rospy.loginfo("gripper opening...")
    gripper_publisher = rospy.Publisher("/rg2/gripper_joint_position/command", Float64, queue_size=1)
    rospy.sleep(1)
    gripper_msg = Float64()
    gripper_msg.data = 1.0
    gripper_publisher.publish(gripper_msg)
    rospy.sleep(1)
    rospy.loginfo("gripper opened!")

def close_gripper_with_param():
    
    rospy.loginfo("gripper closing...")
    rospy.sleep(1)
    rospy.set_param("/rg2_gripper_command", -0.4)
    rospy.sleep(1)
    rospy.loginfo("gripper closed!")

def close_gripper():
    
    rospy.loginfo("gripper closing...")
    gripper_publisher = rospy.Publisher("/rg2/gripper_joint_position/command", Float64, queue_size=1)
    rospy.sleep(1)
    gripper_msg = Float64()
    gripper_msg.data = -0.4
    gripper_publisher.publish(gripper_msg)
    rospy.sleep(1)
    rospy.loginfo("gripper closed!")