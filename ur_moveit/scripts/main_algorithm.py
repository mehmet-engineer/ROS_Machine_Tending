#!/usr/bin/env python3
import rospy, sys
import moveit_commander

"""
 @Author: Mehmet Kahraman
 @Date: 22.12.2022
"""

def initialization():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("moveit_simulation", anonymous=True)
    
    group_name = "ur_arm"
    ur_group = moveit_commander.MoveGroupCommander(group_name)
    group_name = "hand"
    eef_group = moveit_commander.MoveGroupCommander(group_name)
    rospy.loginfo("robot initialized")
    rospy.sleep(1)
    
    return (ur_group, eef_group)

def go_to_target(ur_group, joint_angles):
    joints = ur_group.get_current_joint_values()
    print(joint_values)
    joint_values = joint_angles
    plan_exec = ur_group.go(joint_values, wait=True)
    if plan_exec != True:
        print("plan couldn't be executed")
        quit()
    rospy.loginfo("reached target position \n")
    rospy.sleep(1)
    
def gripper_command(eef_group, command):
    joint_values = eef_group.get_current_joint_values()
    print(joint_values)
    if command == "open":
        joint_values[0] = 1.0
    elif command == "close":
        joint_values[0] = -0.15
    else:
        rospy.loginfo("gripper command failed!")
    plan_exec = eef_group.go(joint_values, wait=True)
    if plan_exec != True:
        print("gripper couldn't be executed")
    rospy.loginfo("gripper command executed \n")
    rospy.sleep(1)
    
def algorithm(ur_group, eef_group):
    
    # GO TO HOME
    go_to_target(ur_group, joint_angles=[0, -1.57, 0, -1.57, 0, 0])
    
    # TARGET OBJECT
    go_to_target(ur_group, joint_angles=[-1.6, -1.0, 1.1, -2.2, -1.57, 0])
    gripper_command(eef_group, command="open")
    go_to_target(ur_group, joint_angles=[-1.6, -1.0, 1.48, -2.2, -1.57, 0])
    gripper_command(eef_group, command="close")
    
    # GO TO HOME
    go_to_target(ur_group, joint_angles=[0, -1.57, 0, -1.57, 0, 0])


if __name__ == '__main__':
    ur_group, eef_group = initialization()
    algorithm(ur_group, eef_group)
