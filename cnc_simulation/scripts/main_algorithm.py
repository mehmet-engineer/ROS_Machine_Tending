#!/usr/bin/env python3
import rospy
from trajectory_msgs.msg import JointTrajectory
from controller_functions import *

"""
 @Author: Mehmet Kahraman
 @Date: 15.12.2022
"""

def initialization():
    rospy.init_node('cnc_trajectory_control_node')
    
    real_robot_controller = '/scaled_pos_joint_traj_controller/command'
    sim_controller = '/pos_joint_traj_controller/command'
    trajectory_publisher = rospy.Publisher(sim_controller, JointTrajectory, queue_size=1)
    
    return trajectory_publisher

def algorithm(traj_publisher):
    
    rospy.sleep(1)
    
    # GO TO HOME
    perform_trajectory(traj_publisher, target_angles=[0, -90, 0, -90, 0, 0], input="degree", duration=4)
    rospy.sleep(1)
    
    # TARGET OBJECT
    perform_trajectory(traj_publisher, target_angles=[-1.6, -1.0, 1.2, -2.2, -1.57, 0], input="radian", duration=5)  #1.48
    rospy.sleep(1)


if __name__ == '__main__':
    trajectory_pub = initialization()
    algorithm(trajectory_pub)
