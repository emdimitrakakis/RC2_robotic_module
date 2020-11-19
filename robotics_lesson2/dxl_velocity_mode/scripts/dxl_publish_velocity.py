#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

def dxl_talker():
    pub = rospy.Publisher('motor_vel_value', Int32, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():

        m_value = int(input("Enter motor velocity value (enter -1 to terminate):"))

        if m_value == -1:
            break;

        while m_value > 250 or m_value < -250:
            print("Motor velocity value should be between -250 and 250")
            m_value = int(input("Enter motor velocity value (enter -1 to terminate):"))

        rospy.loginfo("Motor velocity value is %d", m_value)
        pub.publish(m_value)

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
