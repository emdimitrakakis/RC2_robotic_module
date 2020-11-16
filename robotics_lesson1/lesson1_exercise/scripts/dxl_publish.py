#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int32

def dxl_talker():
    pub = rospy.Publisher('motor_value', Int32, queue_size=10)
    rospy.init_node('dxl_talker', anonymous=True)
    
    rate = rospy.Rate(0.5) # 10hz

    while not rospy.is_shutdown():

        m_value = input("Enter motor value:")

        if m_value == 0:
            break;

        while m_value > 3900 or m_value < 100:
            print("Motor value should be between 100 and 4000")
            m_value = input("Enter motor value:")

    	rospy.loginfo("Motor value is %d", m_value)
        pub.publish(m_value)

        rate.sleep()

if __name__ == '__main__':
    try:
        dxl_talker()
    except rospy.ROSInterruptException:
        pass
