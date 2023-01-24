#!/usr/bin/env python3

import rospy
import actionlib
import control_msgs.msg

def gripper_client(value):

    client = actionlib.SimpleActionClient('/gripper_controller/gripper_cmd', control_msgs.msg.GripperCommandAction)
    client.wait_for_server()

    goal = control_msgs.msg.GripperCommandGoal()
    goal.command.position = value     # From 0.0 to 0.8
    goal.command.max_effort = -1.0    # Do not limit the effort
    client.send_goal(goal)

    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    try:
        # gripper value (0.0 - 0.8)
        gripper_value = 0.5
        rospy.init_node('gripper_command')
        result = gripper_client(gripper_value)
        rospy.loginfo("finished")
        print(result)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")