#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64

def main():
    rospy.init_node('rg2_gripper_control_node')
    gripper_controller = "/rg2/gripper_joint_position/command"
    gripper_publisher = rospy.Publisher(gripper_controller, Float64, queue_size=1)
    rate = rospy.Rate(50)
    
    gripper_msg = Float64()
    
    while not rospy.is_shutdown():
        rospy.loginfo("writing gripper commands...")
        value = rospy.get_param("/rg2_gripper_command")
        gripper_msg.data = value
        gripper_publisher.publish(gripper_msg)
        rate.sleep()
    
if __name__ == '__main__':
    main()